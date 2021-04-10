#include "MPCSolver.hpp"

static std::ofstream foutDebug(realpath("../data/debug.txt", NULL), std::ofstream::trunc);

MPCSolver::MPCSolver(FootstepPlan* _footstepPlan) : footstepPlan(_footstepPlan) {

    ds_samples = doubleSupportSamples;

    // Matrices for cost function
    costFunctionH = Eigen::MatrixXd::Zero(2*N,2*N);
    costFunctionF = Eigen::VectorXd::Zero(2*N);

    // Matrices for stability constraint
    Aeq = Eigen::MatrixXd::Zero(2,2*N);
    beq = Eigen::VectorXd::Zero(2);

    // Matrices for ZMP constraint
    AZmp = Eigen::MatrixXd::Zero(2*N,2*N);
    bZmpMax = Eigen::VectorXd::Zero(2*N);
    bZmpMin = Eigen::VectorXd::Zero(2*N);

    // Matrices for fixed footstep constraint
    ASwingFoot = Eigen::MatrixXd::Zero(4,N*2);
    bSwingFoot = Eigen::VectorXd::Zero(4);

    // Matrices for all constraints stacked
    AConstraint = Eigen::MatrixXd::Zero(2*N+2,2*N);
    bConstraintMin = Eigen::VectorXd::Zero(2*N+2);
    bConstraintMax = Eigen::VectorXd::Zero(2*N+2);

    // Matrices for ZMP prediction
    p = Eigen::VectorXd::Ones(N);
    P = Eigen::MatrixXd::Ones(N,N)*mpcTimeStep;

    for(int i = 0; i < N; ++i) {
    	for(int j = 0; j < N; ++j) {
            if (j > i) P(i, j)=0;
        }
    }

    // Matrices for CoM velocity prediction
    double ch = cosh(omega*mpcTimeStep);
    double sh = sinh(omega*mpcTimeStep);

    Eigen::MatrixXd A_upd = Eigen::MatrixXd::Zero(3,3);
    Eigen::VectorXd B_upd = Eigen::VectorXd::Zero(3);
    A_upd<<ch,sh/omega,1-ch,omega*sh,ch,-omega*sh,0,0,1;
    B_upd<<mpcTimeStep-sh/omega,1-ch,mpcTimeStep;

     old_fsCount = 0;
     adaptation_memo = 0;
     ct = 0;

     xz_dot = 0.0;
     yz_dot = 0.0;
}

MPCSolver::~MPCSolver() {}

State MPCSolver::solve(State current, WalkState walkState, double vRefX, double vRefY) {

    itr = walkState.mpcIter;
    fsCount = walkState.footstepCounter;

    if (fsCount != old_fsCount) {
        // footstep has changed
        //std::cout << "total_adaptation " << adaptation_memo <<std::endl;
        adaptation_memo = 0;
        ds_samples = doubleSupportSamples;
        ct = 0;
    }

    std::vector<Eigen::VectorXd> fp = footstepPlan->getPlan();

    // Reset constraint matrices
    AZmp.setZero();

    // Get the pose of the support foot in the world frame
    Eigen::VectorXd supportFootPose = current.getSupportFootPose(walkState.supportFoot);

    // Construct some matrices that will be used later in the cost function and constraints
    // ************************************************************************************

    // Construct the Cc matrix, which maps iterations to predicted footsteps
    Eigen::MatrixXd CcFull = Eigen::MatrixXd::Zero(N,3+1);

    int fsAhead = 0;
    for (int i = 0; i < N; ++i) {
        // if with the prediction index i we have reach the next footstep increase fsAhead
        if (footstepPlan->getFootstepStartTiming(walkState.footstepCounter) + walkState.mpcIter + i + 1 >= footstepPlan->getFootstepEndTiming(walkState.footstepCounter + fsAhead)) fsAhead++;

        // how many samples are left till the end of the current footstep?
        int samplesTillNextFootstep = footstepPlan->getFootstepEndTiming(walkState.footstepCounter + fsAhead) - (footstepPlan->getFootstepStartTiming(walkState.footstepCounter) + walkState.mpcIter + i + 1);

        // If it is the current footstep, it does not go in Cc, but subsequent ones do
        if (samplesTillNextFootstep > ds_samples) {
            CcFull(i, fsAhead) = 1;
        } else {
            CcFull(i, fsAhead) = (double)samplesTillNextFootstep / (double)ds_samples;
            CcFull(i, fsAhead + 1) = 1.0 - (double)samplesTillNextFootstep / (double)ds_samples;
        }
    }

    Eigen::VectorXd currentFootstepZmp = CcFull.col(0);
    Eigen::MatrixXd Cc = CcFull.block(0,1,N,3);

    // Construct the Ic matrix, which removes constraints from double support phases
    Eigen::MatrixXd Ic = Eigen::MatrixXd::Identity(N,N);
    for (int i = 0; i < N; ++i) {
        if (walkState.footstepCounter == 0 && walkState.mpcIter+i <= footstepPlan->getFootstepEndTiming(0)) Ic(i,i) = 0;
    }

    // Construct the stability constraint
    // **********************************

    double stabConstrMultiplier = (1-exp(-omega*mpcTimeStep)) / omega;

    for(int i = 0; i < N; ++i) {
        Aeq(0,i)   = stabConstrMultiplier*exp(-omega*mpcTimeStep*i)/ (1-exp(-omega*mpcTimeStep*N));
        Aeq(1,N+i) = stabConstrMultiplier*exp(-omega*mpcTimeStep*i)/ (1-exp(-omega*mpcTimeStep*N));
    }

    beq << current.comPos(0) + current.comVel(0)/omega - current.zmpPos(0) ,
           current.comPos(1) + current.comVel(1)/omega - current.zmpPos(1);

    // Construct the ZMP constraint
    // ****************************

    // Construct the ZMP rotation matrix

    fsAhead = 0;
    for (int i = 0; i < N; ++i) {
        // if with the prediction index i we have reach the next footstep increase fsAhead
        if ((int)fp.at(walkState.footstepCounter)(6) + walkState.mpcIter + i + 1 >= (int)fp.at(walkState.footstepCounter + fsAhead + 1)(6)) fsAhead++;

        // how many samples are left till the end of the current footstep?
        int samplesTillNextFootstep = (int)fp.at(walkState.footstepCounter + fsAhead + 1)(6) - ((int)fp.at(walkState.footstepCounter)(6) + walkState.mpcIter + i + 1);
    }

    // Construct the A matrix of the ZMP constraint, by diagonalizing two of the same, then rotating
    Eigen::MatrixXd halfAZmp(N,N);
    halfAZmp << P;
    AZmp.block(0,0,N,N) = halfAZmp;
    AZmp.block(N,N,N,N) = halfAZmp;

    // Construct the b vector of the ZMP constraint
    Eigen::VectorXd bZmpSizeTerm = Eigen::VectorXd::Zero(2*N);
    Eigen::VectorXd bZmpStateTerm = Eigen::VectorXd::Zero(2*N);

    Eigen::VectorXd footx = Eigen::VectorXd::Zero(N);
    Eigen::VectorXd footy = Eigen::VectorXd::Zero(N);
    for(int i = 1; i <= CcFull.cols(); ++i){
      footx += CcFull.block(0,0,N,1)*fp.at(fsCount-1+i)(0);
      footy += CcFull.block(0,0,N,1)*fp.at(fsCount-1+i)(1);
    }

    bZmpMax << p*(-current.zmpPos(0)+footConstraintSquareWidth/2) + footx, p*(-current.zmpPos(1) + footConstraintSquareWidth/2) + footy;
    bZmpMin <<  -p*(-current.zmpPos(0)-footConstraintSquareWidth/2) - footx, -p*(-current.zmpPos(1) - footConstraintSquareWidth/2) - footy;

    Eigen::VectorXd initalbzmpMax = Eigen::VectorXd::Zero(2*N);
    Eigen::VectorXd initalbzmpMin = Eigen::VectorXd::Zero(2*N);

    initalbzmpMax << CcFull.block(0,0,N,1)*(-footConstraintSquareWidth/2 + initalfootConstrainSquarewidthx/2),CcFull.block(0,0,N,1)*(-footConstraintSquareWidth/2 + initalfootConstrainSquarewidthy/2);
    initalbzmpMin << CcFull.block(0,0,N,1)*(-footConstraintSquareWidth/2 + initalfootConstrainSquarewidthx/2),CcFull.block(0,0,N,1)*(-footConstraintSquareWidth/2 + initalfootConstrainSquarewidthy/2);

    if(fsCount == 0){
      bZmpMax += initalbzmpMax;
      bZmpMin += initalbzmpMin;
    }

// Construct the cost function

    // Construct the H matrix, which is made of two of the same halfH block
    Eigen::MatrixXd halfH = Eigen::MatrixXd::Zero(N, N);
    halfH << qZd*Eigen::MatrixXd::Identity(N,N);
    costFunctionH.block(0,0,N,N) = halfH;
    costFunctionH.block(N,N,N,N) = halfH;

    // Construct the F vector
    Eigen::Vector3d stateX = Eigen::Vector3d(current.comPos(0), current.comVel(0), current.zmpPos(0));
    Eigen::Vector3d stateY = Eigen::Vector3d(current.comPos(1), current.comVel(1), current.zmpPos(1));


    // Stack the constraint matrices
    // *****************************

    int nConstraints = Aeq.rows() + AZmp.rows();
    AConstraint.resize(nConstraints, 2*N);
    bConstraintMin.resize(nConstraints);
    bConstraintMax.resize(nConstraints);

    AConstraint    << Aeq, AZmp;
    bConstraintMin << beq, bZmpMin;
    bConstraintMax << beq, bZmpMax;

    // Solve QP and update state
    // *************************

    Eigen::VectorXd decisionVariables = solveQP_hpipm(costFunctionH, costFunctionF, AConstraint, bConstraintMin, bConstraintMax);

    // Split the QP solution in ZMP dot and footsteps
    Eigen::VectorXd zDotOptimalX(N);
    Eigen::VectorXd zDotOptimalY(N);

    zDotOptimalX = (decisionVariables.head(N));
    zDotOptimalY = (decisionVariables.segment(N,N));

    // if(fsCount > 6){
    //   fsCount = 66;
    // }

    std::cout << "fsCount is : " << fsCount << ", then the footstep is : " << fp.at(fsCount+3)(0) << '\n';

    Eigen::VectorXd footstepsOptimalX = Eigen::VectorXd::Zero(3);
    footstepsOptimalX << fp.at(fsCount+1)(0), fp.at(fsCount+2)(0), fp.at(fsCount+3)(0);
    Eigen::VectorXd footstepsOptimalY = Eigen::VectorXd::Zero(3);
    footstepsOptimalY << fp.at(fsCount+1)(1), fp.at(fsCount+2)(1), fp.at(fsCount+3)(1);


    // Update the com-torso state based on the result of the QP
    double ch = cosh(omega*controlTimeStep);
    double sh = sinh(omega*controlTimeStep);

    Eigen::Matrix3d A_upd = Eigen::MatrixXd::Zero(3,3);
    Eigen::Vector3d B_upd = Eigen::VectorXd::Zero(3);
    A_upd<<ch,sh/omega,1-ch,omega*sh,ch,-omega*sh,0,0,1;
    B_upd<<controlTimeStep-sh/omega,1-ch,controlTimeStep;

    Eigen::Vector3d currentStateX = Eigen::Vector3d(current.comPos(0), current.comVel(0), current.zmpPos(0));
    Eigen::Vector3d currentStateY = Eigen::Vector3d(current.comPos(1), current.comVel(1), current.zmpPos(1));
    Eigen::Vector3d nextStateX = A_upd*currentStateX + B_upd*zDotOptimalX(0);
    Eigen::Vector3d nextStateY = A_upd*currentStateY + B_upd*zDotOptimalY(0);

    State next = current; //this is deceiving but it changes all the current to next so no problem
    next.comPos = Eigen::Vector3d(nextStateX(0), nextStateY(0), comTargetHeight);
    next.comVel = Eigen::Vector3d(nextStateX(1), nextStateY(1), 0.0);
    next.zmpPos = Eigen::Vector3d(nextStateX(2), nextStateY(2), 0.0);
    next.comAcc = omega*omega * (next.comPos - next.zmpPos);
    next.torsoOrient = Eigen::Vector3d(0.0, 0.0, 0.0);

    xz_dot = zDotOptimalX(0);
    yz_dot = zDotOptimalY(0);

//Finished till here!

    // Update swing foot position

    Eigen::Vector4d footstepPredicted;
    footstepPredicted << footstepsOptimalX(0),footstepsOptimalY(0),0.0,0.0;

    // If it's the first step, or we are in double support, keep the foot on the ground
    double timeSinceFootstepStart = walkState.controlIter * controlTimeStep;
    double singleSupportEnd = (fp.at(walkState.footstepCounter + 1)(6) - fp.at(walkState.footstepCounter)(6) - ds_samples) * controlTimeStep;
    double swingFootHeight = -(4*stepHeight/pow(singleSupportEnd,2)) * timeSinceFootstepStart * (timeSinceFootstepStart - singleSupportEnd);
    int samplesTillNextFootstep = (int)fp.at(walkState.footstepCounter + 1)(6) - ((int)fp.at(walkState.footstepCounter)(6) + walkState.mpcIter);

    std::cout << "single support end " << singleSupportEnd << std::endl;

    if (walkState.footstepCounter == 0 || samplesTillNextFootstep <= ds_samples) swingFootHeight = 0.0;

    // If support foot is left, move right foot
    if (walkState.supportFoot == 0) {

        if (samplesTillNextFootstep <= ds_samples ){
        next.rightFootPos = current.rightFootPos;
        }  else  {
          next.rightFootPos += kSwingFoot * (footstepPredicted.head(3) - current.rightFootPos);
        }

        next.rightFootPos(2) = swingFootHeight;
        next.rightFootVel = Eigen::Vector3d::Zero();
        next.rightFootAcc = Eigen::Vector3d::Zero();
        next.rightFootOrient(2) += 5 * timeSinceFootstepStart * kSwingFoot * wrapToPi(footstepPredicted(3) - current.rightFootOrient(2));
        next.rightFootOrient(1) = 0*0.0523599 ;


    } else {

        if (samplesTillNextFootstep <= ds_samples ){
        next.leftFootPos = current.leftFootPos;
        } else {
	next.leftFootPos += kSwingFoot * (footstepPredicted.head(3) - current.leftFootPos);
        }

        next.leftFootPos(2) = swingFootHeight;
        next.leftFootVel = Eigen::Vector3d::Zero();
        next.leftFootAcc = Eigen::Vector3d::Zero();
        next.leftFootOrient(2) += 5 * timeSinceFootstepStart * kSwingFoot * wrapToPi(footstepPredicted(3) - current.leftFootOrient(2));

    }


    foutDebug << next.rightFootPos.transpose() << " " << next.leftFootPos.transpose() << " " << footstepPredicted.head(3).transpose() << std::endl;

    next.torsoOrient(2) = wrapToPi((next.leftFootOrient(2) + next.rightFootOrient(2)) / 2.0);

    old_fsCount = fsCount;
    ct = ct + 1;

    // Return next robot state
    return next;
}
