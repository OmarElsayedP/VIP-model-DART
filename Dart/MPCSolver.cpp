#include "MPCSolver.hpp"

static std::ofstream foutDebug(realpath("../data/debug.txt", NULL), std::ofstream::trunc);

MPCSolver::MPCSolver(FootstepPlan* _footstepPlan, int sim, bool CAM) : footstepPlan(_footstepPlan) {

    this-> sim = sim;
    VIP = CAM;
    ds_samples = doubleSupportSamples;

    // Matrices for cost function
    costFunctionH = Eigen::MatrixXd::Zero(2*(N+M),2*(N+M));
    costFunctionF = Eigen::VectorXd::Zero(2*(N+M));
if (VIP)
    {
    costFunctionHcam = Eigen::MatrixXd::Zero(2*N,2*N);
    costFunctionFcam = Eigen::VectorXd::Zero(2*N);
    AZmp_cam = Eigen::MatrixXd::Zero(2*N,2*N);
    bZmpMax_cam = Eigen::VectorXd::Zero(2*N);
    bZmpMin_cam = Eigen::VectorXd::Zero(2*N);
    Aeq_cam = Eigen::MatrixXd::Zero(2,N*2);
    beq_cam = Eigen::VectorXd::Zero(2);
    }    
    // Matrices for stability constraint
    Aeq = Eigen::MatrixXd::Zero(2,(N*2)+(M*2));
    beq = Eigen::VectorXd::Zero(2);

    // Matrices for ZMP constraint
    AZmp = Eigen::MatrixXd::Zero(2*N,2*(N+M));
    bZmpMax = Eigen::VectorXd::Zero(2*N);
    bZmpMin = Eigen::VectorXd::Zero(2*N);

    // Matrices for feasibility constraint
    AFootsteps = Eigen::MatrixXd::Zero(2*M,2*(M+N));
    bFootstepsMax = Eigen::VectorXd::Zero(2*M);
    bFootstepsMin = Eigen::VectorXd::Zero(2*M);

    // Matrices for fixed footstep constraint
    ASwingFoot = Eigen::MatrixXd::Zero(4,(N*2)+(M*2));
    bSwingFoot = Eigen::VectorXd::Zero(4);

    // Matrices for all constraints stacked        
        AConstraint = Eigen::MatrixXd::Zero(2*(N+M)+2,2*(N+M));
        bConstraintMin = Eigen::VectorXd::Zero(2*(N+M)+2);
        bConstraintMax = Eigen::VectorXd::Zero(2*(N+M)+2);
        if(VIP){
            AConstraintcam = Eigen::MatrixXd::Zero(2*N+2,2*N);
            bConstraintMincam = Eigen::VectorXd::Zero(2*N+2);
            bConstraintMaxcam = Eigen::VectorXd::Zero(2*N+2);
        }

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

MPCSolver::~MPCSolver() {}

State MPCSolver::solve(State current, State current_cam, WalkState walkState, double vRefX, double vRefY, double omegaRef, double fx_, double mass) {

    itr = walkState.mpcIter;
    fsCount = walkState.footstepCounter;

    if (fsCount != old_fsCount) {
        adaptation_memo = 0;
        ds_samples = doubleSupportSamples;
        ct = 0;
        std::cout << "FOOTSTEP HAS CHANGED" << '\n';
        std::cout << "footstepCounter = "<< walkState.footstepCounter << '\n';
    }
        std::cout << "footstepCounter = "<< walkState.footstepCounter << '\n';

    std::vector<Eigen::VectorXd> fp = footstepPlan->getPlan();

    // Reset constraint matrices
    AZmp.setZero();
    AFootsteps.setZero();
    if(VIP)
    {
        AZmp_cam.setZero();
    }

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

    //I dont know what this is:
    //============================================================================
    int controlHorizonEndTiming = footstepPlan->getFootstepStartTiming(walkState.footstepCounter) + walkState.mpcIter + N;
    int previewHorizonEndTiming = footstepPlan->getFootstepStartTiming(walkState.footstepCounter) + walkState.mpcIter + N + prev;
    
    // find the last footstep in the control horizon and preview horizon
    Eigen::Vector3d startingZmpPositionAnticipative;
    
    int lastFootstepNumber = footstepPlan->getFootstepIndexAtTime(controlHorizonEndTiming) - walkState.footstepCounter;
    int lastFootstepTiming = footstepPlan->getFootstepStartTiming(walkState.footstepCounter + lastFootstepNumber);
    int lastFootstepNumberPreview = footstepPlan->getFootstepIndexAtTime(previewHorizonEndTiming) - walkState.footstepCounter;
    int lastFootstepTimingPreview = footstepPlan->getFootstepStartTiming(walkState.footstepCounter + lastFootstepNumberPreview);
    
    // find the iteration within the footstep at the end of the control horizon
    int lastFootstepIter = footstepPlan->getFootstepStartTiming(walkState.footstepCounter) + walkState.mpcIter + N - lastFootstepTiming;
    int lastFootstepRemainingIterations = footstepPlan->getFootstepEndTiming(walkState.footstepCounter + lastFootstepNumber) - controlHorizonEndTiming;
    
    // if the last sample falls in single support, ZMP starts from the foot position, otherwise it's on the line connecting to the successive footstep
    if (lastFootstepRemainingIterations > ds_samples) {
        startingZmpPositionAnticipative = footstepPlan->getFootstepPosition(walkState.footstepCounter + lastFootstepNumber);
    } else {
        double doubleSupportAdvancementRatio = 1.0 - (float)lastFootstepRemainingIterations / (float)ds_samples;
        startingZmpPositionAnticipative = (1.0 - doubleSupportAdvancementRatio) * footstepPlan->getFootstepPosition(walkState.footstepCounter + lastFootstepNumber)
                                          + doubleSupportAdvancementRatio * footstepPlan->getFootstepPosition(walkState.footstepCounter + lastFootstepNumber + 1);
    }

    //===================================================================================================

    // add contribution to the tail: first contribution is the last footstep in the control horizon
    Eigen::Vector3d anticipativeTail = Eigen::Vector3d::Zero();

    // int firstFootstepOutsideControlHorizonTiming = footstepPlan->getFootstepEndTiming(walkState.footstepCounter + lastFootstepNumber);
    // Eigen::Vector3d firstFootstepOutsideControlHorizonPos = footstepPlan->getFootstepPosition(walkState.footstepCounter + lastFootstepNumber + 1);
    // if (controlHorizonEndTiming <= firstFootstepOutsideControlHorizonTiming - ds_samples) {
    //     double bigFraction = (exp(omega*ds_samples*mpcTimeStep) - 1.0) / (omega*ds_samples*mpcTimeStep);
    //     double firstCoefficent = exp(-omega*controlHorizonEndTiming*mpcTimeStep) - exp(-omega*firstFootstepOutsideControlHorizonTiming*mpcTimeStep) * (1.0 - bigFraction);
    //     double secondCoefficent = exp(-omega*firstFootstepOutsideControlHorizonTiming*mpcTimeStep) * bigFraction;
    //     anticipativeTail = startingZmpPositionAnticipative * firstCoefficent + firstFootstepOutsideControlHorizonPos * secondCoefficent;
    // } else {
    //     anticipativeTail = (startingZmpPositionAnticipative
    //                        + (firstFootstepOutsideControlHorizonPos - startingZmpPositionAnticipative) / (omega * (firstFootstepOutsideControlHorizonTiming - controlHorizonEndTiming)))
    //                        * (exp(-omega*controlHorizonEndTiming) - exp(-omega*firstFootstepOutsideControlHorizonTiming));
    // }

    // // add contributions for all footstep fully inside the preview horizon
    // for (int i = lastFootstepNumber + 1; i < lastFootstepNumberPreview; i++) {
    //     double firstTiming = footstepPlan->getFootstepStartTiming(walkState.footstepCounter + i) * mpcTimeStep;
    //     double secondTiming = footstepPlan->getFootstepStartTiming(walkState.footstepCounter + i + 1) * mpcTimeStep;
    //     double bigFraction = (exp(omega*ds_samples*mpcTimeStep) - 1.0) / (omega*ds_samples*mpcTimeStep);
    //     double firstCoefficent = exp(-omega*firstTiming) - exp(-omega*secondTiming) * (1.0 - bigFraction);
    //     double secondCoefficent = exp(-omega*secondTiming) * bigFraction;
    //     anticipativeTail += footstepPlan->getFootstepPosition(walkState.footstepCounter + i) * firstCoefficent + footstepPlan->getFootstepPosition(walkState.footstepCounter + i + 1) * secondCoefficent;
    // }

    // add contribution from the last footstep in the preview horizon (FIXME here we neglect the double support for simplicity)
    // anticipativeTail += footstepPlan->getFootstepPosition(walkState.footstepCounter + lastFootstepNumberPreview) * exp(-omega*lastFootstepTiming * mpcTimeStep);

    // construct stability constraint with anticipative tail
    for (int i = 0; i < prev; i++) {
        anticipativeTail(0) += exp(-omega*mpcTimeStep*(N + i)) * (1 - exp(-omega*mpcTimeStep)) * x_midpoint(walkState.mpcIter + N + i);
        anticipativeTail(1) += exp(-omega*mpcTimeStep*(N + i)) * (1 - exp(-omega*mpcTimeStep)) * y_midpoint(walkState.mpcIter + N + i);
    }

    anticipativeTail(0) += exp(-omega*mpcTimeStep*(N + prev)) * x_midpoint(walkState.mpcIter + N + prev);
    anticipativeTail(1) += exp(-omega*mpcTimeStep*(N + prev)) * y_midpoint(walkState.mpcIter + N + prev);


    double stabConstrMultiplier = (1-exp(-omega*mpcTimeStep)) / omega;

    for(int i = 0; i < N; ++i) {
        Aeq(0,i)     = stabConstrMultiplier * exp(-omega*mpcTimeStep*i) - mpcTimeStep * exp(-omega*mpcTimeStep*N);
        Aeq(1,N+M+i) = stabConstrMultiplier * exp(-omega*mpcTimeStep*i) - mpcTimeStep * exp(-omega*mpcTimeStep*N);
    }

    beq << current.comPos(0) + current.comVel(0)/omega - current.zmpPos(0) * (1.0 - exp(-omega*mpcTimeStep*N)) - anticipativeTail(0) ,
           current.comPos(1) + current.comVel(1)/omega - current.zmpPos(1) * (1.0 - exp(-omega*mpcTimeStep*N)) - anticipativeTail(1) ;
/**/
    //CAM
    if(VIP)
    {
    // anticipativeTail = Eigen::Vector3d::Zero();
    
    for(int i = 0; i < N; ++i) {
        Aeq_cam(0,i)     = stabConstrMultiplier * exp(-omega*mpcTimeStep*i) - mpcTimeStep * exp(-omega*mpcTimeStep*N);
        Aeq_cam(1,N+i) = stabConstrMultiplier * exp(-omega*mpcTimeStep*i) - mpcTimeStep * exp(-omega*mpcTimeStep*N);
    }

    beq_cam << current_cam.comPos(0) + current_cam.comVel(0)/omega - current_cam.zmpPos(0) * (1.0 - exp(-omega*mpcTimeStep*N)) - anticipativeTail(0),
           current_cam.comPos(1) + current_cam.comVel(1)/omega - current_cam.zmpPos(1) * (1.0 - exp(-omega*mpcTimeStep*N)) - anticipativeTail(1);

}
     // Construct the ZMP constraint
    // ****************************

    // Construct the ZMP rotation matrix
    Eigen::MatrixXd rCosZmp = Eigen::MatrixXd::Identity(N,N);
    Eigen::MatrixXd rSinZmp = Eigen::MatrixXd::Zero(N,N);

    fsAhead = 0;
    for (int i = 0; i < N; ++i) {
        // if with the prediction index i we have reach the next footstep increase fsAhead
        if ((int)fp.at(walkState.footstepCounter)(6) + walkState.mpcIter + i + 1 >= (int)fp.at(walkState.footstepCounter + fsAhead + 1)(6)) fsAhead++;

        // how many samples are left till the end of the current footstep?
        int samplesTillNextFootstep = (int)fp.at(walkState.footstepCounter + fsAhead + 1)(6) - ((int)fp.at(walkState.footstepCounter)(6) + walkState.mpcIter + i + 1);

        // If it is the current footstep, it does not go in Cc, but subsequent ones do
        if (samplesTillNextFootstep > ds_samples) {
            rCosZmp(i,i) = cos(predictedOrientations(fsAhead));
            rSinZmp(i,i) = sin(predictedOrientations(fsAhead));
        } else {
            double movingConstraintOrientation = predictedOrientations(fsAhead) * (double)samplesTillNextFootstep / (double)ds_samples
                                                 + predictedOrientations(fsAhead + 1) * (1.0 - (double)samplesTillNextFootstep / (double)ds_samples);
            rCosZmp(i,i) = cos(predictedOrientations(movingConstraintOrientation));
            rSinZmp(i,i) = sin(predictedOrientations(movingConstraintOrientation));
        }
    }

    Eigen::MatrixXd zmpRotationMatrix(2*N,2*N);
    zmpRotationMatrix <<  rCosZmp,rSinZmp,
                 -rSinZmp,rCosZmp;


    // Construct the A matrix of the ZMP constraint, by diagonalizing two of the same, then rotating
    Eigen::MatrixXd halfAZmp(N,N+M);
    halfAZmp << Ic*P, -Ic*Cc;
    AZmp.block(0,0,N,N+M) = halfAZmp;
    AZmp.block(N,N+M,N,N+M) = halfAZmp;

    AZmp = zmpRotationMatrix * AZmp;

    // Construct the b vector of the ZMP constraint
    Eigen::VectorXd bZmpSizeTerm = Eigen::VectorXd::Zero(2*N);
    Eigen::VectorXd bZmpStateTerm = Eigen::VectorXd::Zero(2*N);
    Eigen::VectorXd restriction = Eigen::VectorXd::Zero(2*N);
    Eigen::VectorXd res = Eigen::VectorXd::Zero(N);

    bZmpSizeTerm << Ic*p*footConstraintSquareWidth/2, Ic*p*footConstraintSquareWidth/2;
    for (int j = 0; j<N; j++) res(j) = j;
    restriction << Ic*res*controlTimeStep*footConstraintSquareWidth/2, Ic*res*controlTimeStep*footConstraintSquareWidth/2;

    bZmpStateTerm << Ic*(-p*current.zmpPos(0)+currentFootstepZmp*supportFootPose(3)), Ic*(-p*current.zmpPos(1)+currentFootstepZmp*supportFootPose(4));
    bZmpStateTerm = zmpRotationMatrix * bZmpStateTerm;

    bZmpMin = - bZmpSizeTerm + bZmpStateTerm;
    bZmpMax =   bZmpSizeTerm + bZmpStateTerm;
    // Construct the A matrix of the ZMP constraint, by diagonalizing two of the same, then rotating
    if(VIP){
    AZmp_cam.block(0,0,N,N) = P;
    AZmp_cam.block(N,N,N,N) = P;
    double xzcam_max = forceLimit/(mass*9.81);
    double yzcam_max = forceLimit/(mass*9.81);

        bZmpMin_cam << p*(xzcam_max-current.zmpPos(0)),p*(yzcam_max-current.zmpPos(1));
        bZmpMax_cam << -bZmpMin_cam;
}
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

    // Assemble the A matrix for the kinematic constraint, and rotate
    AFootsteps.block(0,N,M,M) = differenceMatrix;
    AFootsteps.block(M,2*N+M,M,M) = differenceMatrix;
    AFootsteps = footstepsRotationMatrix * AFootsteps;

    // Assemble the b vector for the kinematic constraint
    bFootstepsMin << -pF*deltaXMax + supportFootPose(3)*currentFootstepKinematic, -pFl*deltaYOut + pFr*deltaYIn  + supportFootPose(4)*currentFootstepKinematic;
    bFootstepsMax <<  pF*deltaXMax + supportFootPose(3)*currentFootstepKinematic, -pFl*deltaYIn  + pFr*deltaYOut + supportFootPose(4)*currentFootstepKinematic;

    // Construct the cost function
    // ***************************

    // Construct the H matrix, which is made of two of the same halfH block
    Eigen::MatrixXd halfH = Eigen::MatrixXd::Zero(N+M, N+M);
    halfH << qZd*Eigen::MatrixXd::Identity(N,N) + qZ*P.transpose()*P,
             -qZ*P.transpose()*Cc, -qZ*Cc.transpose()*P, qZ*Cc.transpose()*Cc + qF*Eigen::MatrixXd::Identity(M,M);
    costFunctionH.block(0,0,N+M,N+M) = halfH;
    costFunctionH.block(N+M,N+M,N+M,N+M) = halfH;

    if(VIP){
        costFunctionHcam.block(0,0,N,N) = qZd_cam*Eigen::MatrixXd::Identity(N,N);
        costFunctionH.block(N,N,N,N) = qZd_cam*Eigen::MatrixXd::Identity(N,N);
    }

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
        Eigen::Vector3d stateX = Eigen::Vector3d(current.comPos(0), current.comVel(0), current.zmpPos(0));
        Eigen::Vector3d stateY = Eigen::Vector3d(current.comPos(1), current.comVel(1), current.zmpPos(1));

    costFunctionF << qZ*P.transpose()*(p*current.zmpPos(0) - currentFootstepZmp*supportFootPose(3)),
             -qZ*Cc.transpose()*(p*current.zmpPos(0) - currentFootstepZmp*supportFootPose(3)) - qF*xCandidateFootsteps,
             qZ*P.transpose()*(p*current.zmpPos(1) - currentFootstepZmp*supportFootPose(4)),
             -qZ*Cc.transpose()*(p*current.zmpPos(1) - currentFootstepZmp*supportFootPose(4)) - qF*yCandidateFootsteps;

             if(VIP){
                Eigen::VectorXd Nzeros = Eigen::VectorXd::Zero(N);
                costFunctionFcam << Nzeros, Nzeros;
        }

    // Stack the constraint matrices
    // *****************************
    int nConstraints = Aeq.rows() + AFootsteps.rows() + AZmp.rows();
    AConstraint.resize(nConstraints, 2*(N+M));
    bConstraintMin.resize(nConstraints);
    bConstraintMax.resize(nConstraints);

    AConstraint    << Aeq, AFootsteps, AZmp;
    bConstraintMin << beq, bFootstepsMin, bZmpMin;
    bConstraintMax << beq, bFootstepsMax, bZmpMax;
    if(VIP){
        int nConstraintscam = Aeq_cam.rows() + AZmp_cam.rows();
        AConstraintcam.resize(nConstraintscam, 2*N);
        bConstraintMincam.resize(nConstraintscam);
        bConstraintMaxcam.resize(nConstraintscam);
        bConstraintMincam << beq_cam, bZmpMin_cam;
        bConstraintMaxcam << beq_cam, bZmpMax_cam;

    AConstraintcam    << Aeq_cam, AZmp_cam;
    bConstraintMincam << beq_cam, bZmpMin_cam;
    bConstraintMaxcam << beq_cam, bZmpMax_cam;

    }

    // Solve QP and update state
    // *************************

    //auto finish = std::chrono::high_resolution_clock::now();

    //auto interval = std::chrono::time_point_cast<std::chrono::microseconds>(finish) - std::chrono::time_point_cast<std::chrono::microseconds>(start);
    //std::cout << "Elapsed time: " << interval.count() << std::endl;





    //auto start = std::chrono::high_resolution_clock::now();
    //Eigen::VectorXd decisionVariables = solveQP(costFunctionH, costFunctionF, AConstraint, bConstraintMin, bConstraintMax);
    Eigen::VectorXd decisionVariables;
    decisionVariables = solveQP_hpipm(costFunctionH, costFunctionF, AConstraint, bConstraintMin, bConstraintMax);

    // Eigen::VectorXd decisionVariables = solveQP_hpipm(costFunctionH, costFunctionF, AConstraint, bConstraintMin, bConstraintMax);
    //auto finish = std::chrono::high_resolution_clock::now();

    //auto interval = std::chrono::time_point_cast<std::chrono::milliseconds>(finish) - std::chrono::time_point_cast<std::chrono::milliseconds>(start);
    //std::cout << "Elapsed time: " << interval.count() << std::endl;


    // Split the QP solution in ZMP dot and footsteps
    Eigen::VectorXd zDotOptimalX(N);
    Eigen::VectorXd zDotOptimalY(N);
    Eigen::VectorXd footstepsOptimalX(M);
    Eigen::VectorXd footstepsOptimalY(M);

    zDotOptimalX = (decisionVariables.head(N));
    zDotOptimalY = (decisionVariables.segment(N+M,N));
    footstepsOptimalX = decisionVariables.segment(N,M);
    footstepsOptimalY = decisionVariables.segment(2*N+M,M);

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

    State next = current;
    next.comPos = Eigen::Vector3d(nextStateX(0), nextStateY(0), comTargetHeight);
    next.comVel = Eigen::Vector3d(nextStateX(1), nextStateY(1), 0.0);
    next.zmpPos = Eigen::Vector3d(nextStateX(2), nextStateY(2), 0.0);
    next.comAcc = omega*omega * (next.comPos - next.zmpPos);
    next.torsoOrient = Eigen::Vector3d(0.0, 0.0, 0.0);

    xz_dot = zDotOptimalX(0);
    yz_dot = zDotOptimalY(0);

    Eigen::Vector4d footstepPredicted;
    footstepPredicted << footstepsOptimalX(0),footstepsOptimalY(0),0.0,predictedOrientations(1);

    //std::cout << "x "<< footstepsOptimalX(0) <<std::endl;
    //std::cout << "y "<< footstepsOptimalX(1) <<std::endl;

    // Update swing foot position

    // If it's the first step, or we are in double support, keep the foot on the ground
    double timeSinceFootstepStart = walkState.controlIter * controlTimeStep;
    double singleSupportEnd = (fp.at(walkState.footstepCounter + 1)(6) - fp.at(walkState.footstepCounter)(6) - ds_samples) * controlTimeStep;
    double swingFootHeight = -(4*stepHeight/pow(singleSupportEnd,2)) * timeSinceFootstepStart * (timeSinceFootstepStart - singleSupportEnd);
    int samplesTillNextFootstep = (int)fp.at(walkState.footstepCounter + 1)(6) - ((int)fp.at(walkState.footstepCounter)(6) + walkState.mpcIter);


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
        // next.rightFootOrient(1) = 0*0.0523599 ;


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



    //std::cout << "footstepPredicted.head(3)" <<std::endl;
    //std::cout << footstepPredicted.head(3) - current.rightFootPos <<std::endl;

    foutDebug << next.rightFootPos.transpose() << " " << next.leftFootPos.transpose() << " " << footstepPredicted.head(3).transpose() << std::endl;

    next.torsoOrient(2) = wrapToPi((next.leftFootOrient(2) + next.rightFootOrient(2)) / 2.0);

    old_fsCount = fsCount;
    ct = ct + 1;

    // Return next robot state
    return next;
}



int MPCSolver::timingQP(State current_s, WalkState walkState_s, double anticipativeTail_x, double anticipativeTail_y){


// See what can be passed to the function and what should be a pubblic class variable ...
    std::cout<< "yalahwyyy" << std::endl;
    int samplesTNF = (int)footstepPlan->getFootstepStartTiming(walkState_s.footstepCounter + 1)  - ((int)footstepPlan->getFootstepStartTiming(walkState_s.footstepCounter)  + walkState_s.mpcIter);

    // matrices
    Eigen::MatrixXd H_tqp;
    Eigen::VectorXd F_tqp;
    Eigen::MatrixXd C_tqp;
    Eigen::VectorXd d_min_tqp;
    Eigen::VectorXd d_max_tqp;

    H_tqp = Eigen::MatrixXd::Zero(3,3);
    F_tqp = Eigen::VectorXd::Zero(3);
    C_tqp = Eigen::MatrixXd::Zero(5,3);
    d_min_tqp = Eigen::VectorXd::Zero(5);
    d_max_tqp = Eigen::VectorXd::Zero(5);

    // cost function
    H_tqp(0,0) = 1;
    H_tqp(1,1) = 1000000000;
    H_tqp(2,2) = 1000000000;
    F_tqp(0) = -exp(-omega*samplesTNF*mpcTimeStep);

    // timing box constraints
    double t_max = 0.65;
    double t_min = 0.4;

    if (samplesTNF > 50 && samplesTNF <= t_max) t_min = 0.4;
    if (samplesTNF > 40 && samplesTNF <= 50) t_min = 0.35;
    if (samplesTNF > 30 && samplesTNF <= 40) t_min = 0.28;
    if (samplesTNF > 20 && samplesTNF <= 30) t_min = 0.18;
    if (samplesTNF > 10 && samplesTNF <= 20) t_min = 0.08;
    if (samplesTNF > 0 && samplesTNF <= 10) t_min = 0.01;

    C_tqp(0,0) = 1;
    d_min_tqp(0) = exp(-omega*t_max);
    d_max_tqp(0) = exp(-omega*t_min);

    // lambda_c and lambda_vector
    double lambda_c = exp(-omega*predictionTime);
    Eigen::VectorXd lambda_vector;
    lambda_vector = Eigen::VectorXd::Zero(M-1);
    for (int k = 0; k<M-1; k++) lambda_vector(k) = exp(-omega*mpcTimeStep*(  (int)footstepPlan->getFootstepStartTiming(walkState_s.footstepCounter + 2 + k)  - (int)footstepPlan->getFootstepStartTiming(walkState_s.footstepCounter + 1 + k) ));

 // check this, it may be wrong

    //build feasibility region constraint - x component
    double a_x_term = 0.0;
    double a_x_prod = 1.0;

    for (int k = 1; k<M-1; k++){
         a_x_prod = a_x_prod*lambda_vector(k-1);
         a_x_term = a_x_term + a_x_prod;
    }

    bool sf;
    if (walkState_s.footstepCounter%2 == 0) {
        sf = 0;
    }else {
        sf = 1;
    }

    double x_u = current_s.comPos(0) + current_s.comVel(0)/omega - current_s.getSupportFootPose(sf)(3);  // TODO: relative xu position. Check in debug

    C_tqp(1,0) = -(1+a_x_term)*deltaXMax;
    C_tqp(1,1) = 1;
    C_tqp(2,0) = -(1+a_x_term)*deltaXMax;
    C_tqp(2,1) = -1;
    d_max_tqp(1) = -(x_u) + footConstraintSquareWidth/2*(1-lambda_c) + anticipativeTail_x - (M-1)*lambda_c*deltaXMax;
    d_max_tqp(2) = (x_u) + footConstraintSquareWidth/2*(1-lambda_c) - anticipativeTail_x - (M-1)*lambda_c*deltaXMax;
    d_min_tqp(1) = - 100;
    d_min_tqp(2) = - 100;
/**/

    //build feasibility region constraint - y component
    double ell = 0.2 + 0*(deltaYOut+deltaYIn)/2;
    double wky = 0.08 + 0*(deltaYOut-deltaYIn)/2;
    double up_margin, down_margin;
    double y_upper_ZMP_boundary_p = 0;
    double y_lower_ZMP_boundary_p = 0;
    Eigen::VectorXd y_upper_ZMP_boundary;
    Eigen::VectorXd y_lower_ZMP_boundary;
    y_upper_ZMP_boundary = Eigen::VectorXd::Zero(M);
    y_lower_ZMP_boundary = Eigen::VectorXd::Zero(M);

    double a_y_term_upper = 0.0;
    double a_y_term_lower = 0.0;
    double a_y_prod = 1.0;

    double y_u = current_s.comPos(1) + current_s.comVel(1)/omega - current_s.getSupportFootPose(sf)(4);


    if (walkState_s.footstepCounter%2 == 0) {
        // left support foot
        up_margin = 0*0.031;
        down_margin = 0*0.008;

        for (int h = 1; h<= M; h++) {

        y_upper_ZMP_boundary(h-1) = (pow(-1.0,(double) h))*(ell+(pow(-1.0,(double)h))*wky)-y_upper_ZMP_boundary_p;
        y_upper_ZMP_boundary_p = y_upper_ZMP_boundary(h-1);

        y_lower_ZMP_boundary(h-1) = (pow(-1.0,(double)h))*(ell+(pow(-1.0,(double)h+1))*wky)-y_lower_ZMP_boundary_p;
        y_lower_ZMP_boundary_p = y_lower_ZMP_boundary(h-1);

        }


    } else {
        // right support foot
        up_margin = 0*0.008;
        down_margin = 0*0.031;

        for (int h = 1; h<= M; h++) {

        y_upper_ZMP_boundary(h-1) = (pow(-1.0,(double)h+1))*(ell+(pow(-1.0,(double)h+1))*wky)-y_upper_ZMP_boundary_p;
        y_upper_ZMP_boundary_p = y_upper_ZMP_boundary(h-1);

        y_lower_ZMP_boundary(h-1) = (pow(-1.0,(double)h+1))*(ell+(pow(-1.0,(double)h))*wky)-y_lower_ZMP_boundary_p;
        y_lower_ZMP_boundary_p = y_lower_ZMP_boundary(h-1);

        }

     }



     for (int k = 1; k<M-1; k++){
         a_y_prod = a_y_prod*lambda_vector(k-1);
         a_y_term_upper = a_y_term_upper + a_y_prod*y_upper_ZMP_boundary(k); // maybe (k-1)
         a_y_term_lower = a_y_term_lower + a_y_prod*y_lower_ZMP_boundary(k);

     }


      C_tqp(3,0) = -(y_upper_ZMP_boundary(0)+a_y_term_upper);
      C_tqp(3,2) = 1;
      C_tqp(4,0) = (y_lower_ZMP_boundary(0)+a_y_term_lower);
      C_tqp(4,2) = -1;
      Eigen::VectorXd sum; sum = Eigen::VectorXd::Ones(M); sum(M-1) = 0;
      Eigen::VectorXd summ; summ = Eigen::VectorXd::Ones(1);

      summ = (sum.transpose()*y_upper_ZMP_boundary);
      d_max_tqp(3) = -(y_u) + footConstraintSquareWidth/2*(1-lambda_c) + anticipativeTail_y - summ(0)*lambda_c - up_margin;
      summ = (sum.transpose()*y_lower_ZMP_boundary);
      d_max_tqp(4) = (y_u) + footConstraintSquareWidth/2*(1-lambda_c) - anticipativeTail_y + summ(0)*lambda_c - down_margin;
      d_min_tqp(3) = - 100;
      d_min_tqp(4) = - 100;

/**/

int n_variables = 3;
int n_constr = 5;

int nv = n_variables;
int ne = 0;  //number of equality constraints
int nb = 0;
int ng = n_constr;
int ns = 0;
int nsb = 0;
int nsg = 0;
int idxb[n_variables] = {};
double H[n_variables*n_variables] = {};
double g[n_variables] = {};
double d_lb[n_constr] = {};
double d_ub[n_constr] = {};
double C[n_constr*n_variables] = {};


for (int i = 0; i<n_variables; i++) {

    g[i] = F_tqp(i);

    for (int j = 0; j<n_variables; j++) {
         H[j*n_variables+i] = H_tqp(i,j);
    }

}

for (int k = 0; k<n_constr; k++) {
     d_lb[k] = d_min_tqp(k) ;
     d_ub[k] = d_max_tqp(k);

    for (int j = 0; j<n_variables; j++) {
         C[j*n_constr+k] = C_tqp(k,j);
    }
}

int dim_size = d_dense_qp_dim_memsize();
void *dim_mem = malloc(dim_size);
struct d_dense_qp_dim dim;
d_dense_qp_dim_create(&dim, dim_mem);
d_dense_qp_dim_set_all(nv, ne, nb, ng, nsb, nsg, &dim);
int qp_size = d_dense_qp_memsize(&dim);
void *qp_mem = malloc(qp_size);
struct d_dense_qp qp;
d_dense_qp_create(&dim, &qp, qp_mem);
d_dense_qp_set_H(H, &qp);
d_dense_qp_set_g(g, &qp);
d_dense_qp_set_C(C, &qp);
d_dense_qp_set_lg(d_lb, &qp);
d_dense_qp_set_ug(d_ub, &qp);
int qp_sol_size = d_dense_qp_sol_memsize(&dim);
void *qp_sol_mem = malloc(qp_sol_size);
struct d_dense_qp_sol qp_sol;
d_dense_qp_sol_create(&dim, &qp_sol, qp_sol_mem);
int ipm_arg_size = d_dense_qp_ipm_arg_memsize(&dim);
void *ipm_arg_mem = malloc(ipm_arg_size);
struct d_dense_qp_ipm_arg arg;
d_dense_qp_ipm_arg_create(&dim, &arg, ipm_arg_mem);
enum hpipm_mode mode = ROBUST;   // set mode ROBUST, SPEED, BALANCE, SPEED_ABS
d_dense_qp_ipm_arg_set_default(mode, &arg);
int ipm_size = d_dense_qp_ipm_ws_memsize(&dim, &arg);
void *ipm_mem = malloc(ipm_size);
struct d_dense_qp_ipm_ws workspace;
d_dense_qp_ipm_ws_create(&dim, &arg, &workspace, ipm_mem);
d_dense_qp_ipm_solve(&qp, &qp_sol, &arg, &workspace);
Eigen::VectorXd u_store = Eigen::VectorXd::Zero(n_variables);
double *u = (double *) malloc(n_variables*sizeof(double));
d_dense_qp_sol_get_v(&qp_sol, u);
for (int i=0; i<n_variables; i++) u_store(i) = u[i];


// show the solution

//std::cout<< "timing solution " << -(1/omega)*log(u_store(0)) << std::endl;

// free memory

free(u);
free(dim_mem);
free(qp_mem);
free(qp_sol_mem);
free(ipm_arg_mem);
free(ipm_mem);


if (isnan(-(1/omega)*log(u_store(0)))) { //std::cout<< "shit " << std::endl;
return samplesTNF;

} else {

return round(-(1/omega)*log(u_store(0))*100);

}





}
