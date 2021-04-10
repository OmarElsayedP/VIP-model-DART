#include "MPCSolvercam.hpp"

static std::ofstream foutDebug(realpath("../data/debug.txt", NULL), std::ofstream::trunc);

MPCSolvercam::MPCSolvercam(FootstepPlan* _footstepPlan, int sim, bool CAM) : footstepPlan(_footstepPlan) {

    this-> sim = sim;
    VIP = CAM;
    ds_samples = doubleSupportSamples;

    costFunctionHcam = Eigen::MatrixXd::Zero(2*N,2*N);
    costFunctionFcam = Eigen::VectorXd::Zero(2*N);
    AZmp_cam = Eigen::MatrixXd::Zero(2*N,2*N);
    bZmpMax_cam = Eigen::VectorXd::Zero(2*N);
    bZmpMin_cam = Eigen::VectorXd::Zero(2*N);
    Aeq_cam = Eigen::MatrixXd::Zero(2,N*2);
    beq_cam = Eigen::VectorXd::Zero(2);

    // Matrices for all constraints stacked        
            AConstraintcam = Eigen::MatrixXd::Zero(2*N+2,2*N);
            bConstraintMincam = Eigen::VectorXd::Zero(2*N+2);
            bConstraintMaxcam = Eigen::VectorXd::Zero(2*N+2);
    // Matrices for ZMP prediction
    p = Eigen::VectorXd::Ones(N);
    P = Eigen::MatrixXd::Ones(N,N)*mpcTimeStep;

    for(int i = 0; i < N; ++i) {
    	for(int j = 0; j < N; ++j) {
            if (j > i) P(i, j)=0;
        }
    }

    // Matrices for CoM velocity prediction
    Vu = Eigen::MatrixXd::Zero(N,N);
    Vs = Eigen::MatrixXd::Zero(N,3);

    double ch = cosh(omega*mpcTimeStep);
    double sh = sinh(omega*mpcTimeStep);

    Eigen::MatrixXd A_upd = Eigen::MatrixXd::Zero(3,3);
    Eigen::VectorXd B_upd = Eigen::VectorXd::Zero(3);
    A_upd<<ch,sh/omega,1-ch,omega*sh,ch,-omega*sh,0,0,1;
    B_upd<<mpcTimeStep-sh/omega,1-ch,mpcTimeStep;

    Eigen::RowVectorXd Vu_newline(N);
    Eigen::RowVectorXd Vs_newline(3);
    Eigen::RowVectorXd A_midLine(3);

    A_midLine<<omega*sh,ch,-omega*sh;

    for(int i=1;i<=N;++i) {
        Vu_newline.setZero();
        Vs_newline.setZero();
        Vu_newline(i-1) = 1-ch;
        if(i>1) {
            for(int j=0;j<=i-2;++j) {
                Vu_newline.segment(j,1) = A_midLine*(matrixPower(A_upd,(i-j-2)))*B_upd;
            }
        }
        Vs_newline = A_midLine*(matrixPower(A_upd,(i-1)));
        Vu.row(i-1) = Vu_newline;
        Vs.row(i-1) = Vs_newline;
    }

  // Midpoint of the ZMP constraints for anticipative tail
    x_midpoint.resize(500);
    y_midpoint.resize(500);
    int midpointFootstepIndex = 0;
    for (int i = 0; i < 500; i++) {
        int timeTillFootstepEnd = footstepPlan->getFootstepEndTiming(midpointFootstepIndex) - i;

        if (timeTillFootstepEnd <= 0) { // is footstep finished?
            midpointFootstepIndex++;
            timeTillFootstepEnd = footstepPlan->getFootstepEndTiming(midpointFootstepIndex) - i;
        }

        if (timeTillFootstepEnd > ds_samples) { // are we in single support?
            x_midpoint(i) = footstepPlan->getFootstepPosition(midpointFootstepIndex)(0);
            y_midpoint(i) = footstepPlan->getFootstepPosition(midpointFootstepIndex)(1);
        } else { // we are in double support
            x_midpoint(i) = footstepPlan->getFootstepPosition(midpointFootstepIndex)(0) * (double)timeTillFootstepEnd / ds_samples
                            + footstepPlan->getFootstepPosition(midpointFootstepIndex + 1)(0) * (1 - (double)timeTillFootstepEnd / ds_samples);
            y_midpoint(i) = footstepPlan->getFootstepPosition(midpointFootstepIndex)(1) * (double)timeTillFootstepEnd / ds_samples
                            + footstepPlan->getFootstepPosition(midpointFootstepIndex + 1)(1) * (1 - (double)timeTillFootstepEnd / ds_samples);
        }
    }
     old_fsCount = 0;
     adaptation_memo = 0;
     ct = 0;

     xz_dot = 0.0;
     yz_dot = 0.0;
     
     if(VIP) 
    {
        xz_dot_cam = 0.0;
        yz_dot_cam = 0.0;
    }

}

MPCSolvercam::~MPCSolvercam() {}

State MPCSolvercam::solve(State current, State current_cam, WalkState walkState, double vRefX, double vRefY, double omegaRef, double fx_, double mass) {

    itr = walkState.mpcIter;
    fsCount = walkState.footstepCounter;

    if (fsCount != old_fsCount) {
        adaptation_memo = 0;
        ds_samples = doubleSupportSamples;
        ct = 0;
        std::cout << "FOOTSTEP HAS CHANGEDcam" << '\n';
        std::cout << "footstepCountercam = "<< walkState.footstepCounter << '\n';
    }

        std::cout << "footstepCountercam = "<< walkState.footstepCounter << '\n';

    std::vector<Eigen::VectorXd> fp = footstepPlan->getPlan();

    // Reset constraint matrices
    AZmp_cam.setZero();
    // Get the pose of the support foot in the world frame
    Eigen::VectorXd supportFootPose = current.getSupportFootPose(walkState.supportFoot);

    // Construct some matrices that will be used later in the cost function and constraints
    // ************************************************************************************

    // Construct the Cc matrix, which maps iterations to predicted footsteps
    Eigen::MatrixXd CcFull = Eigen::MatrixXd::Zero(N,M+1);

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
    Eigen::MatrixXd Cc = CcFull.block(0,1,N,M);

    // Construct the Ic matrix, which removes constraints from double support phases
    Eigen::MatrixXd Ic = Eigen::MatrixXd::Identity(N,N);
    for (int i = 0; i < N; ++i) {
        if (walkState.footstepCounter == 0 && walkState.mpcIter+i <= footstepPlan->getFootstepEndTiming(0)) Ic(i,i) = 0;
    }

    // Construct the difference matrix (x_j - x_{j-1})
    Eigen::MatrixXd differenceMatrix = Eigen::MatrixXd::Identity(M,M);
    for (int i = 0; i < M-1; ++i) {
        differenceMatrix(i+1,i) = -1;
    }

    // Retrieve footstep orientations over control horizon
    Eigen::VectorXd predictedOrientations(M+1); // FIXME we would like to remove conditionals for the first step
    for (int i = 0; i < M+1; i++) {
        if (walkState.footstepCounter - 2 + i >= 0) predictedOrientations(i) = footstepPlan->getFootstepOrientation(walkState.footstepCounter - 2 + i);
        else predictedOrientations(i) = footstepPlan->getFootstepOrientation(0);
    }


    // Construct the stability constraint
    // **********************************
// Periodic tail
    // double stabConstrMultiplierP = (1-exp(-omega*mpcTimeStep)) / (1-pow(exp(-omega*mpcTimeStep),N));
    // for(int i = 0; i < N; ++i) {
    //     Aeq(0,i)     = stabConstrMultiplierP * exp(-omega*mpcTimeStep*i)/omega;
    //     Aeq(1,N+M+i) = stabConstrMultiplierP * exp(-omega*mpcTimeStep*i)/omega;
    // }
    //
    // beq << current.comPos(0) + current.comVel(0)/omega - current.zmpPos(0),
    //        current.comPos(1) + current.comVel(1)/omega - current.zmpPos(1);

//Truncated tail
    // double lambda_tail = exp(-omega*mpcTimeStep);
    // for(int i = 0; i < N; ++i) {
    //     Aeq(0,i)     = (1/omega)*(1-lambda_tail)*exp(-omega*mpcTimeStep*i)-pow(lambda_tail,N)*mpcTimeStep*exp(-omega*mpcTimeStep*N);
    //     Aeq(0,N+M+i) = (1/omega)*(1-lambda_tail)*exp(-omega*mpcTimeStep*i)-pow(lambda_tail,N)*mpcTimeStep*exp(-omega*mpcTimeStep*N);
    // }
    //
    // beq << current.comPos(0) + current.comVel(0)/omega - current.zmpPos(0),
    //        current.comPos(1) + current.comVel(1)/omega - current.zmpPos(1);
    //

    int prev = 200;

    // add contribution to the tail: first contribution is the last footstep in the control horizon

    Eigen::Vector3d anticipativeTail = Eigen::Vector3d::Zero();

    // add contribution from the last footstep in the preview horizon (FIXME here we neglect the double support for simplicity)
    // construct stability constraint with anticipative tail
    for (int i = 0; i < prev; i++) {
        anticipativeTail(0) += exp(-omega*mpcTimeStep*(N + i)) * (1 - exp(-omega*mpcTimeStep)) * x_midpoint(walkState.mpcIter + N + i);
        anticipativeTail(1) += exp(-omega*mpcTimeStep*(N + i)) * (1 - exp(-omega*mpcTimeStep)) * y_midpoint(walkState.mpcIter + N + i);
    }

    anticipativeTail(0) += exp(-omega*mpcTimeStep*(N + prev)) * x_midpoint(walkState.mpcIter + N + prev);
    anticipativeTail(1) += exp(-omega*mpcTimeStep*(N + prev)) * y_midpoint(walkState.mpcIter + N + prev);


    double stabConstrMultiplier = (1-exp(-omega*mpcTimeStep)) / omega;

    for(int i = 0; i < N; ++i) {
        Aeq_cam(0,i)     = stabConstrMultiplier * exp(-omega*mpcTimeStep*i) - mpcTimeStep * exp(-omega*mpcTimeStep*N);
        Aeq_cam(1,N+i) = stabConstrMultiplier * exp(-omega*mpcTimeStep*i) - mpcTimeStep * exp(-omega*mpcTimeStep*N);
    }

    beq_cam << current_cam.comPos(0) + current_cam.comVel(0)/omega - current_cam.zmpPos(0) * (1.0 - exp(-omega*mpcTimeStep*N)) - anticipativeTail(0),
           current_cam.comPos(1) + current_cam.comVel(1)/omega - current_cam.zmpPos(1) * (1.0 - exp(-omega*mpcTimeStep*N)) - anticipativeTail(1);
     // Construct the ZMP constraint
    // ****************************


    // Construct the A matrix of the ZMP constraint, by diagonalizing two of the same, then rotating
    AZmp_cam.block(0,0,N,N) = P;
    AZmp_cam.block(N,N,N,N) = P;
    double xzcam_max = forceLimit/(mass*9.81);
    double yzcam_max = forceLimit/(mass*9.81);

        bZmpMin_cam << p*(xzcam_max-current_cam.zmpPos(0)),p*(yzcam_max-current_cam.zmpPos(1));
        bZmpMax_cam << -bZmpMin_cam;
    // Construct the kinematic constraint
    // **********************************

    // Construct the matrices that activate the left or right constraint
    Eigen::VectorXd pFr = Eigen::VectorXd::Zero(M);
    Eigen::VectorXd pFl = Eigen::VectorXd::Zero(M);
    Eigen::VectorXd pF = Eigen::VectorXd::Ones(M);
    for (int i = 0; i < M; ++i) {
        pFr(i) = (int)(walkState.supportFoot + i) % 2;
    }
    pFl = pF - pFr;

    // Vector for the current footstep position
    Eigen::VectorXd currentFootstepKinematic = Eigen::VectorXd::Zero(M);
    currentFootstepKinematic(0) = 1;

    // Construct the kinematic rotation matrix
    Eigen::MatrixXd footstepsRotationMatrix(2*M,2*M);
    Eigen::MatrixXd rCosFootsteps = Eigen::MatrixXd::Identity(M,M);
    Eigen::MatrixXd rSinFootsteps = Eigen::MatrixXd::Zero(M,M);
    for(int i=0; i<M; ++i){
        rCosFootsteps(i,i) = cos(predictedOrientations(i));
        rSinFootsteps(i,i) = sin(predictedOrientations(i));
    }
    footstepsRotationMatrix <<  rCosFootsteps, rSinFootsteps,
                   -rSinFootsteps, rCosFootsteps;

    // Construct the cost function
    // ***************************

    // Construct the H matrix, which is made of two of the same halfH block
        costFunctionHcam.block(0,0,N,N) = qZd_cam*Eigen::MatrixXd::Identity(N,N);
        costFunctionHcam.block(N,N,N,N) = qZd_cam*Eigen::MatrixXd::Identity(N,N);
    // Contruct candidate footstep vectors
    Eigen::VectorXd xCandidateFootsteps(M), yCandidateFootsteps(M);
    for (int i = 0; i < M; i++) {

        if (walkState.footstepCounter - 1 + i >= 0) {

         yCandidateFootsteps(i) = fp.at(walkState.footstepCounter - 1 + i)(1);
         xCandidateFootsteps(i) = fp.at(walkState.footstepCounter - 1 + i)(0);

        }
    }
 if (walkState.footstepCounter > 2){
         yCandidateFootsteps(0) = supportFootPose(4);
         //xCandidateFootsteps(0) = supportFootPose(3) + 0.21;
}

    if (walkState.footstepCounter == 0) { // FIXME this is ugly but it is necessary to add the fake initial step
        xCandidateFootsteps(0) = 0.0;
        yCandidateFootsteps(0) = -0.08;
    }

    // Construct the F vector
    Eigen::VectorXd Nzeros = Eigen::VectorXd::Zero(N);
    costFunctionFcam << Nzeros, Nzeros;

    // Stack the constraint matrices
    // *****************************
        int nConstraintscam = Aeq_cam.rows() + AZmp_cam.rows();
        AConstraintcam.resize(nConstraintscam, 2*N);
        bConstraintMincam.resize(nConstraintscam);
        bConstraintMaxcam.resize(nConstraintscam);
        bConstraintMincam << beq_cam, bZmpMin_cam;
        bConstraintMaxcam << beq_cam, bZmpMax_cam;

    AConstraintcam    << Aeq_cam, AZmp_cam;
    bConstraintMincam << beq_cam, bZmpMin_cam;
    bConstraintMaxcam << beq_cam, bZmpMax_cam;

    // Solve QP and update state
    // *************************

    //auto finish = std::chrono::high_resolution_clock::now();

    //auto interval = std::chrono::time_point_cast<std::chrono::microseconds>(finish) - std::chrono::time_point_cast<std::chrono::microseconds>(start);
    //std::cout << "Elapsed time: " << interval.count() << std::endl;





    //auto start = std::chrono::high_resolution_clock::now();
    //Eigen::VectorXd decisionVariables = solveQP(costFunctionH, costFunctionF, AConstraint, bConstraintMin, bConstraintMax);
    Eigen::VectorXd decisionVariables;
    decisionVariables = solveQP_hpipm(costFunctionHcam, costFunctionFcam, AConstraintcam, bConstraintMincam, bConstraintMaxcam);

    // Eigen::VectorXd decisionVariables = solveQP_hpipm(costFunctionH, costFunctionF, AConstraint, bConstraintMin, bConstraintMax);
    //auto finish = std::chrono::high_resolution_clock::now();

    //auto interval = std::chrono::time_point_cast<std::chrono::milliseconds>(finish) - std::chrono::time_point_cast<std::chrono::milliseconds>(start);
    //std::cout << "Elapsed time: " << interval.count() << std::endl;


    // Split the QP solution in ZMP dot and footsteps
    Eigen::VectorXd zDotOptimalX_cam(N);
    Eigen::VectorXd zDotOptimalY_cam(N);

    zDotOptimalX_cam = (decisionVariables.head(N));
    zDotOptimalY_cam = (decisionVariables.tail(N));

    // Update the com-torso state based on the result of the QP
    double ch = cosh(omega*controlTimeStep);
    double sh = sinh(omega*controlTimeStep);

    Eigen::Matrix3d A_upd = Eigen::MatrixXd::Zero(3,3);
    Eigen::Vector3d B_upd = Eigen::VectorXd::Zero(3);
    A_upd<<ch,sh/omega,1-ch,omega*sh,ch,-omega*sh,0,0,1;
    B_upd<<controlTimeStep-sh/omega,1-ch,controlTimeStep;

    Eigen::Vector3d currentStateX_cam = Eigen::Vector3d(current_cam.comPos(0), current_cam.comVel(0), current_cam.zmpPos(0));
    Eigen::Vector3d currentStateY_cam = Eigen::Vector3d(current_cam.comPos(1), current_cam.comVel(1), current_cam.zmpPos(1));
    Eigen::Vector3d nextStateX_cam = A_upd*currentStateX_cam + B_upd*zDotOptimalX_cam(0);
    Eigen::Vector3d nextStateY_cam = A_upd*currentStateY_cam + B_upd*zDotOptimalY_cam(0);

    State next_cam = current_cam;
    next_cam.comPos = Eigen::Vector3d(nextStateX_cam(0), nextStateY_cam(0), comTargetHeight);
    next_cam.comVel = Eigen::Vector3d(nextStateX_cam(1), nextStateY_cam(1), 0.0);
    next_cam.zmpPos = Eigen::Vector3d(nextStateX_cam(2), nextStateY_cam(2), 0.0);
    next_cam.comAcc = omega*omega * (next_cam.comPos - next_cam.zmpPos);
    next_cam.torsoOrient = Eigen::Vector3d(0.0, 0.0, 0.0);

    xz_dot_cam = zDotOptimalX_cam(0);
    yz_dot_cam = zDotOptimalY_cam(0);

    // Eigen::Vector4d footstepPredicted;
    // footstepPredicted << footstepsOptimalX(0),footstepsOptimalY(0),0.0,predictedOrientations(1);

    //std::cout << "x "<< footstepsOptimalX(0) <<std::endl;
    //std::cout << "y "<< footstepsOptimalX(1) <<std::endl;

    // Update swing foot position

    // If it's the first step, or we are in double support, keep the foot on the ground
    double timeSinceFootstepStart = walkState.controlIter * controlTimeStep;
    double singleSupportEnd = (fp.at(walkState.footstepCounter + 1)(6) - fp.at(walkState.footstepCounter)(6) - ds_samples) * controlTimeStep;
    double swingFootHeight = -(4*stepHeight/pow(singleSupportEnd,2)) * timeSinceFootstepStart * (timeSinceFootstepStart - singleSupportEnd);
    int samplesTillNextFootstep = (int)fp.at(walkState.footstepCounter + 1)(6) - ((int)fp.at(walkState.footstepCounter)(6) + walkState.mpcIter);


return next_cam;
}