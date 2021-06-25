#include "MPCSolvercam.hpp"

static std::ofstream foutDebug(realpath("../data/debug.txt", NULL), std::ofstream::trunc);

MPCSolvercam::MPCSolvercam(FootstepPlan* _footstepPlan, int sim, bool CAM, double torsomass, Eigen::Matrix3d MOI, Eigen::Vector3d theta_max) : footstepPlan(_footstepPlan) {

    this-> sim = sim;
    this->CAM = angleConstraint;
    ds_samples = doubleSupportSamples;

        currentTheta = Eigen::Vector3d::Zero(3);
        currentThetaD = Eigen::Vector3d::Zero(3);
        currentThetaDD = Eigen::Vector3d::Zero(3);
        this->MOI = MOI;
        desiredTorque = Eigen::MatrixXd::Zero(3,N);
        this->theta_max = theta_max;

        costFunctionHcam = Eigen::MatrixXd::Zero(2*N,2*N);
        costFunctionFcam = Eigen::VectorXd::Zero(2*N);
    AZmp_cam = Eigen::MatrixXd::Zero(2*N,2*N);
    bZmpMax_cam = Eigen::VectorXd::Zero(2*N);
    bZmpMin_cam = Eigen::VectorXd::Zero(2*N);
    Aeq_cam = Eigen::MatrixXd::Zero(2,N*2);
    beq_cam = Eigen::VectorXd::Zero(2);

    if(angleConstraint){
        std::cout<< "HERE!" <<std::endl;

        AConstraintcam = Eigen::MatrixXd::Zero(4*N+2,2*N);
        bConstraintMincam = Eigen::VectorXd::Zero(4*N+2);
        bConstraintMaxcam = Eigen::VectorXd::Zero(4*N+2);
        
        AAngleconstr = Eigen::MatrixXd::Zero(2*N,2*N);
        bAngleConstrMin = Eigen::VectorXd::Zero(2*N);
        bAngleConstrMax = Eigen::VectorXd::Zero(2*N);
    }
    else{
            AConstraintcam = Eigen::MatrixXd::Zero(2*N+2,2*N);
            bConstraintMincam = Eigen::VectorXd::Zero(2*N+2);
            bConstraintMaxcam = Eigen::VectorXd::Zero(2*N+2);
    }

    // Matrices for all constraints stacked        
    // Matrices for ZMP prediction
    p = Eigen::VectorXd::Ones(N);
    pmg = p*torsomass*9.81;

    // std::cout << pmg;

    P = Eigen::MatrixXd::Ones(N,N)*mpcTimeStep;
    Pmg = Eigen::MatrixXd::Ones(N,N)*mpcTimeStep*torsomass*9.81;
    Eigen::MatrixXd Pthdd1 = Eigen::MatrixXd::Ones(N,N)*mpcTimeStep;
    Eigen::MatrixXd Pthdd2 = Eigen::MatrixXd::Ones(N,N)*0.5*pow(mpcTimeStep,2);


    Eigen::MatrixXd Pthddnew1 = Eigen::MatrixXd::Ones(N,N)*mpcTimeStep;
    Eigen::MatrixXd Pthddnew2 = Eigen::MatrixXd::Ones(N,N)*0.5*pow(mpcTimeStep,2);


    for(int i = 0; i < N; ++i) {
    	for(int j = 0; j < N; ++j) {
            if (j > i) P(i, j)=0;
            if (j >= i) Pmg(i, j)=0;
            if (j <= i) Pthdd1(i, j) = 0;
            if(j < i) Pthdd2(i, j) = 0;

            if(j >= i) Pthddnew1(i, j) = 0;
            if(j > i) Pthddnew2(i, j) = 0;
        }
    }
    // Pthdd1 = Pthdd1.transpose();
    // Pthdd2 = Pthdd2.transpose();

    // std::cout << "P=" << P << std::endl;
    Pthdd = (Pthdd1*(P.transpose())) + Pthdd2;

    Pthddnew = P*Pthddnew1 + Pthddnew2;

    // std::cout << "Pmg.block(0,0,20,20) = " <<Pmg.block(0,0,20,20) << std::endl;
    // std::cout << "Pthdd1*(P.transpose()).block(0,0,20,20) = " << (Pthdd1*(P.transpose())).block(0,0,20,20) << std::endl;
    // std::cout << "Pthdd2.block(0,0,20,20) = " << Ptd2.block(0,0,20,20) << std::endl;
    // std::cout << "Pthdd = " << Pthdd.block(0,0,20,20) << std::endl;

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

        xz_dot_cam = 0.0;
        yz_dot_cam = 0.0;


}

MPCSolvercam::~MPCSolvercam() {}

State MPCSolvercam::solve(State no_current, State current_cam, WalkState walkState, double mass, const dart::dynamics::SkeletonPtr& mRobot, double x_tot, double y_tot, 
    Eigen::Vector3d AngularPosition, Eigen::Vector3d AngularVelocity) {

    itr = walkState.mpcIter;
    fsCount = walkState.footstepCounter;

    // if (fsCount != old_fsCount) {
    //     adaptation_memo = 0;
    //     ds_samples = doubleSupportSamples;
    //     ct = 0;
    //     std::cout << "FOOTSTEP HAS CHANGEDcam" << '\n';
    //     std::cout << "footstepCountercam = "<< walkState.footstepCounter << '\n';
    // }

    std::vector<Eigen::VectorXd> fp = footstepPlan->getPlan();

    // Reset constraint matrices
    AZmp_cam.setZero();
    AAngleconstr.setZero();
    Atermconstr.setZero();
    costFunctionHcam.setZero();
    costFunctionFcam.setZero();
    Aeq_cam.setZero();
    // Get the pose of the support foot in the world frame
    // Eigen::VectorXd supportFootPose = current.getSupportFootPose(walkState.supportFoot);

    // Construct some matrices that will be used later in the cost function and constraints
    // ************************************************************************************

    // Construct the Cc matrix, which maps iterations to predicted footsteps
    // Eigen::MatrixXd CcFull = Eigen::MatrixXd::Zero(N,M+1);

    // int fsAhead = 0;
    // for (int i = 0; i < N; ++i) {
    //     // if with the prediction index i we have reach the next footstep increase fsAhead
    //     if (footstepPlan->getFootstepStartTiming(walkState.footstepCounter) + walkState.mpcIter + i + 1 >= footstepPlan->getFootstepEndTiming(walkState.footstepCounter + fsAhead)) fsAhead++;

    //     // how many samples are left till the end of the current footstep?
    //     int samplesTillNextFootstep = footstepPlan->getFootstepEndTiming(walkState.footstepCounter + fsAhead) - (footstepPlan->getFootstepStartTiming(walkState.footstepCounter) + walkState.mpcIter + i + 1);

    //     // If it is the current footstep, it does not go in Cc, but subsequent ones do
    //     if (samplesTillNextFootstep > ds_samples) {
    //         CcFull(i, fsAhead) = 1;
    //     } else {
    //         CcFull(i, fsAhead) = (double)samplesTillNextFootstep / (double)ds_samples;
    //         CcFull(i, fsAhead + 1) = 1.0 - (double)samplesTillNextFootstep / (double)ds_samples;
    //     }
    // }

    // Eigen::VectorXd currentFootstepZmp = CcFull.col(0);
    // Eigen::MatrixXd Cc = CcFull.block(0,1,N,M);

    // // Construct the Ic matrix, which removes constraints from double support phases
    // Eigen::MatrixXd Ic = Eigen::MatrixXd::Identity(N,N);
    // for (int i = 0; i < N; ++i) {
    //     if (walkState.footstepCounter == 0 && walkState.mpcIter+i <= footstepPlan->getFootstepEndTiming(0)) Ic(i,i) = 0;
    // }

    // // Construct the difference matrix (x_j - x_{j-1})
    // Eigen::MatrixXd differenceMatrix = Eigen::MatrixXd::Identity(M,M);
    // for (int i = 0; i < M-1; ++i) {
    //     differenceMatrix(i+1,i) = -1;
    // }

    // // Retrieve footstep orientations over control horizon
    // Eigen::VectorXd predictedOrientations(M+1); // FIXME we would like to remove conditionals for the first step
    // for (int i = 0; i < M+1; i++) {
    //     if (walkState.footstepCounter - 2 + i >= 0) predictedOrientations(i) = footstepPlan->getFootstepOrientation(walkState.footstepCounter - 2 + i);
    //     else predictedOrientations(i) = footstepPlan->getFootstepOrientation(0);
    // }


    // Construct the stability constraint
    // **********************************
// // Periodic tail
    double stabConstrMultiplierP = (1-exp(-omega*mpcTimeStep)) / (1-pow(exp(-omega*mpcTimeStep),N));
    for(int i = 0; i < N; ++i) {
        Aeq_cam(0,i)     = stabConstrMultiplierP * exp(-omega*mpcTimeStep*i)/omega;
        Aeq_cam(1,N+i) = stabConstrMultiplierP * exp(-omega*mpcTimeStep*i)/omega;
    }
    
    beq_cam << current_cam.comPos(0) + current_cam.comVel(0)/omega - current_cam.zmpPos(0),
           current_cam.comPos(1) + current_cam.comVel(1)/omega - current_cam.zmpPos(1);


// //Truncated tail
    // double lambda_tail = exp(-omega*mpcTimeStep);
    // for(int i = 0; i < N; ++i) {
    //     Aeq_cam(0,i)  = (1/omega)*(1-lambda_tail)*exp(-omega*mpcTimeStep*i);
    //     Aeq_cam(1,N+i) = (1/omega)*(1-lambda_tail)*exp(-omega*mpcTimeStep*i);
    // }
    
    // beq_cam << current_cam.comPos(0) + current_cam.comVel(0)/omega - current_cam.zmpPos(0),
    //        current_cam.comPos(1) + current_cam.comVel(1)/omega - current_cam.zmpPos(1);


    // int prev = 200;

    // // add contribution to the tail: first contribution is the last footstep in the control horizon

    // Eigen::Vector3d anticipativeTail = Eigen::Vector3d::Zero();

    // // add contribution from the last footstep in the preview horizon (FIXME here we neglect the double support for simplicity)
    // // construct stability constraint with anticipative tail
    // for (int i = 0; i < prev; i++) {
    //     anticipativeTail(0) += exp(-omega*mpcTimeStep*(N + i)) * (1 - exp(-omega*mpcTimeStep)) * x_midpoint(walkState.mpcIter + N + i);
    //     anticipativeTail(1) += exp(-omega*mpcTimeStep*(N + i)) * (1 - exp(-omega*mpcTimeStep)) * y_midpoint(walkState.mpcIter + N + i);
    // }

    // anticipativeTail(0) += exp(-omega*mpcTimeStep*(N + prev)) * x_midpoint(walkState.mpcIter + N + prev);
    // anticipativeTail(1) += exp(-omega*mpcTimeStep*(N + prev)) * y_midpoint(walkState.mpcIter + N + prev);


    // double stabConstrMultiplier = (1-exp(-omega*mpcTimeStep)) / omega;

    // for(int i = 0; i < N; ++i) {
    //     Aeq_cam(0,i) = stabConstrMultiplier * exp(-omega*mpcTimeStep*i) - mpcTimeStep * exp(-omega*mpcTimeStep*N);
    //     Aeq_cam(1,N+i) = stabConstrMultiplier * exp(-omega*mpcTimeStep*i) - mpcTimeStep * exp(-omega*mpcTimeStep*N);
    // }

    // beq_cam << current_cam.comPos(0) + current_cam.comVel(0)/omega - current_cam.zmpPos(0) * (1.0 - exp(-omega*mpcTimeStep*N)) - anticipativeTail(0),
    //        current_cam.comPos(1) + current_cam.comVel(1)/omega - current_cam.zmpPos(1) * (1.0 - exp(-omega*mpcTimeStep*N)) - anticipativeTail(1);
     // Construct the ZMP constraint
    // ****************************


    // Construct the A matrix of the ZMP constraint, by diagonalizing two of the same, then rotating
    AZmp_cam.block(0,0,N,N) = P;
    AZmp_cam.block(N,N,N,N) = P;
    double xzcam_max = forceLimittorques/(mass*9.81);
    double yzcam_max = forceLimittorques/(mass*9.81);
    // xzcam_max = 10.0;
    // yzcam_max = 10.0;

        bZmpMax_cam << p*(xzcam_max-current_cam.zmpPos(0)),p*(yzcam_max-current_cam.zmpPos(1));
        bZmpMin_cam << p*(-xzcam_max-current_cam.zmpPos(0)),p*(-yzcam_max-current_cam.zmpPos(1));

        // std::cout << "xzcam_max = " << xzcam_max << std::endl;
        // std::cout << "yzcam_max = " << yzcam_max << std::endl;
        // std::cout << "-xzcam_max-current_cam.zmpPos(0) = " << -xzcam_max-current_cam.zmpPos(0) << std::endl;
        // std::cout << "xzcam_max-current_cam.zmpPos(0) = " << xzcam_max-current_cam.zmpPos(0) << std::endl;
    // Construct the Angle constraint
    // *******************************
        std::cout << angleConstraint << std::endl;
        std::cout << CAM << std::endl;
                    // std::cout << pmg << std::endl;
        currentThetaD = AngularVelocity;
        currentTheta = AngularPosition;
        if(angleConstraint){

            // std::cout<<"1"<<std::endl;
            Eigen::RowVectorXd desiredTorqueX, desiredTorqueY, desiredTorqueZ;
            Eigen::VectorXd desiredTorqueZV;
            // std::cout<<"2"<<std::endl;
            desiredTorqueX = -current_cam.zmpPos(1)*pmg.transpose();
            // std::cout<<"3"<<std::endl;
            desiredTorqueY = current_cam.zmpPos(0)*pmg.transpose();
            // std::cout<<"4"<<std::endl;

            // desiredtorqueZ = (((x_tot-no_current.zmpPos(0))*(-current_cam.zmpPos(1))*mass*9.81)
            //  + (y_tot-no_current.zmpPos(1))*(current_cam.zmpPos(0)*mass*9.81))/comTargetHeight;
            // std::cout<<"5"<<std::endl;
            desiredTorqueZV = ((x_tot-no_current.zmpPos(0))*(desiredTorqueX.transpose()) + (y_tot-no_current.zmpPos(1))*(desiredTorqueY.transpose()));
            // std::cout<<desiredTorqueZV<<std::endl;
            desiredTorqueZV = desiredTorqueZV/-comTargetHeight;
            // std::cout<<desiredTorqueZV<<std::endl;
            // std::cout << comTargetHeight << std::endl;
            // std::cout<<"6"<<std::endl;
            desiredTorqueZ = desiredTorqueZV.transpose();

            desiredTorque.block(0,0,1,N) = desiredTorqueX;
            // std::cout<<"7"<<std::endl;
            desiredTorque.block(1,0,1,N) = desiredTorqueY;
            // std::cout<<"8"<<std::endl;
            desiredTorque.block(2,0,1,N) = desiredTorqueZ;

            // std::cout<<"9"<<std::endl;
            // std::cout << "desiredTorque = " << desiredTorque << std::endl;
            // std::cout<<"9"<<std::endl;


        // desiredTorque.block(2,0,1,1) << ((x_tot-no_current.zmpPos(0))*desiredTorque(0)+(y_tot-no_current.zmpPos(1))*desiredTorque(1))/comTargetHeight;
        // std::cout<<"desiredTorque = " << desiredTorque << std::endl;
        // std::cout << "currentThetaDD = " << currentThetaDD << std::endl;
        // std::cout << "currentTheta = " << currentTheta << std::endl;
        // std::cout << "torsoOrient = " << current_cam.torsoOrient << ", " << current.torsoOrient << std::endl;

        // currentThetaD = mRobot->getCOMLinearVelocity();
        // currentTheta = mRobot->getCOM();
        
        // actual current angular values
        // std::cout << "Before angle const" << std::endl;

        // Eigen::Vector3d thetaMax << M_PI/6, M_PI/6, M_PI/6;
        // std::cout<<"theta_max = "<< theta_max << std::endl;
        // std::cout << theta_max*(p.transpose()) << std::endl;
        // std::cout << currentTheta*(p.transpose()) << std::endl;
        // std::cout << currentThetaD*p.transpose()*P.transpose() << std::endl;
        Eigen::MatrixXd rhs = MOI*(theta_max*(p.transpose())-currentTheta*(p.transpose()) - currentThetaD*p.transpose()*P.transpose())-desiredTorque*Pthdd;
        // rhs = MOI*rhs;
        // rhs = rhs - desiredTorque*Pthdd;
        Eigen::MatrixXd temprhs = Eigen::MatrixXd::Zero(2,N);
        temprhs = rhs.block(0,0,2,N);
        Eigen::MatrixXd _rhs = (temprhs.transpose());
        
        Eigen::MatrixXd lhs = -theta_max*(p.transpose())-currentTheta*(p.transpose()) - currentThetaD*p.transpose()*P.transpose();
        lhs = MOI*lhs;
        lhs = lhs - desiredTorque*Pthdd;

        Eigen::MatrixXd templhs = Eigen::MatrixXd::Zero(2,N);
        templhs = lhs.block(0,0,2,N);
        Eigen::MatrixXd _lhs = (templhs.transpose());
        // _lhs.setZero();
        // std::cout<< "_lhs.cols(), _lhs.rows() = " << _lhs.rows() << ", " << _lhs.cols() << std::endl;

        // std::cout << "after angle const" << std::endl;

    AAngleconstr.block(0,N,N,N) = -(Pthdd.transpose())*Pmg;
    AAngleconstr.block(N,0,N,N) = (Pthdd.transpose())*Pmg;

        // std::cout << "after AAngleconstr const" << std::endl;

    // std::cout << "AAngleconstr.block(0,0,N,N)" << AAngleconstr.block(0,0,N,N);
    // std::cout << "AAngleconstr.block(0,N,N,N) =" << AAngleconstr.block(0,N,N,N);
    // std::cout << "AAngleconstr.block(N,0,N,N)" << AAngleconstr.block(N,0,N,N);
    // std::cout << "AAngleconstr.block(N,N,N,N)" << AAngleconstr.block(N,N,N,N);


            // std::cout << "AYWAAA" << std::endl;
        bAngleConstrMin.block(0,0,N,1) = _lhs.block(0,0,N,1);
        bAngleConstrMin.block(N,0,N,1) = _lhs.block(0,1,N,1);
        // std::cout << "Eldapapaaaa" << std::endl;
        bAngleConstrMax.block(0,0,N,1) = _rhs.block(0,0,N,1);
        bAngleConstrMax.block(N,0,N,1) = _rhs.block(0,1,N,1);
        // bAngleConstrMax << _rhs.block(0,0,N,1), _rhs.block(0,1,N,1);

        // std::cout << "after bAngleConstr const" << std::endl;

    // Construct the terminal constraint
    // *******************************
        // std::cout<< "walkState.mpcIter" << walkState.mpcIter << std::endl;
        // Atermconstr.block(0,50+walkState.mpcIter,1,50-walkState.mpcIter) = Eigen::RowVectorXd::Ones(1,50-walkState.mpcIter)*mpcTimeStep;
        // Atermconstr.block(1,50+walkState.mpcIter,1,50-walkState.mpcIter) = Eigen::RowVectorXd::Ones(1,50-walkState.mpcIter)*mpcTimeStep;
        // // if((50-walkState.mpcIter) <= 1)
        // //     Atermconstr.block(0,97,1,3) = Eigen::RowVectorXd::Ones(1,3)*mpcTimeStep;
        // // std::cout << Atermconstr.block(0,0,1,100) << std::endl;
        // btermconstr << -current_cam.zmpPos(0), -current_cam.zmpPos(1);

        // std::cout << "after terminal const" << std::endl;

        int nConstraintscam = Aeq_cam.rows() + AZmp_cam.rows() + AAngleconstr.rows();
        // std::cout << "nConstraintscam = " << nConstraintscam << std::endl;
        AConstraintcam.resize(nConstraintscam, 2*N);
        bConstraintMincam.resize(nConstraintscam);
        bConstraintMaxcam.resize(nConstraintscam);

                // std::cout << "After resize"<< std::endl;

    AConstraintcam    << Aeq_cam, AZmp_cam, AAngleconstr;
    bConstraintMincam << beq_cam, bZmpMin_cam, bAngleConstrMin;
    bConstraintMaxcam << beq_cam, bZmpMax_cam, bAngleConstrMax;

            // std::cout << "after angle constraint" << std::endl;
}
else{
    // Stack the constraint matrices
    // *****************************
        int nConstraintscam = Aeq_cam.rows() + AZmp_cam.rows();
        AConstraintcam.resize(nConstraintscam, 2*N);
        bConstraintMincam.resize(nConstraintscam);
        bConstraintMaxcam.resize(nConstraintscam);

    AConstraintcam    << Aeq_cam, AZmp_cam;
    bConstraintMincam << beq_cam, bZmpMin_cam;
    bConstraintMaxcam << beq_cam, bZmpMax_cam;
}
    // Construct the terminal constraint
    // *******************************
    // delta_remaining = zeros(1,C);
    // n_remaining = j - fs_timing(fsCounter) + 1; % According to paper n must be >= 3
    // %     if(n_remaining > fs_timing(2)-2)
    // %         n_remaining = fs_timing(2)-2;
    // %     end
    // delta_remaining(1,n_remaining:fs_timing(2)) = mpcTimeStep;

    
    // Atermconstr = [delta_remaining, zeros(1,C); ...
    //     zeros(1,C), delta_remaining];
    // btermconstr = [-xzcam; -yzcam];
    


    // Construct the cost function
    // ***************************
        // std::cout << "before cost fcn" << std::endl;

Eigen::MatrixXd Ixx;
Eigen::MatrixXd Iyy;
Eigen::MatrixXd Ixy;
Eigen::MatrixXd Iyx;

Ixx = MOI.inverse()(0,0)*Eigen::MatrixXd::Identity(N,N);
Iyy = MOI.inverse()(1,1)*Eigen::MatrixXd::Identity(N,N);

Ixy = MOI.inverse()(0,1)*Eigen::MatrixXd::Identity(N,N);
Iyx = MOI.inverse()(1,0)*Eigen::MatrixXd::Identity(N,N);

Eigen::MatrixXd Qthx;
Eigen::MatrixXd Qthy;

Qthx = qThx*Eigen::MatrixXd::Identity(N,N);
Qthy = qThy*Eigen::MatrixXd::Identity(N,N);

// Ixy = 0.0*Eigen::MatrixXd::Identity(N,N);
// Iyx = 0.0*Eigen::MatrixXd::Identity(N,N);

// std::cout << "Ixx = " << MOI.inverse()(0,0)  << std::endl;
// std::cout << "Ixy = " << MOI.inverse()(0,1)  << std::endl;
// std::cout << "Ixz = " << MOI.inverse()(0,2)  << std::endl;
// std::cout << "Iyx = " << MOI.inverse()(1,0)   << std::endl;
// std::cout << "Iyy = " << MOI.inverse()(1,1)   << std::endl;
// std::cout << "Iyz = " << MOI.inverse()(1,2)  << std::endl;
// std::cout << "Izx = " << MOI.inverse()(2,0)   << std::endl;
// std::cout << "Izy = " << MOI.inverse()(2,1)   << std::endl;
// std::cout << "Izz = " << MOI.inverse()(2,2)  << std::endl;
// Ixx = (1.0/MOI(0,0))*Eigen::MatrixXd::Identity(N,N);
// Iyy = (1.0/MOI(1,1))*Eigen::MatrixXd::Identity(N,N);

Eigen::MatrixXd Pthddnew1mg = Eigen::MatrixXd::Ones(N,N)*mpcTimeStep*mass*9.81;

    for(int i = 0; i < N; ++i) {
        for(int j = 0; j < N; ++j) {
            if(j >= i) Pthddnew1mg(i, j) = 0;
        }
    }

    // Construct the H matrix, which is made of two of the same halfH block
        costFunctionHcam.block(0,0,N,N) = qZd_cam*Eigen::MatrixXd::Identity(N,N);
        costFunctionHcam.block(N,N,N,N) = qZd_cam*Eigen::MatrixXd::Identity(N,N);





        Eigen::MatrixXd A_ = Eigen::MatrixXd::Zero(3,3);
        Eigen::MatrixXd B_ = Eigen::MatrixXd::Zero(3,1);
        Eigen::MatrixXd C_ = Eigen::MatrixXd::Zero(1,3);
        A_ << 1, mpcTimeStep, pow(mpcTimeStep,2)/2, 0, 1, mpcTimeStep, 0, 0, 1;
        B_ << pow(mpcTimeStep,2)/6, pow(mpcTimeStep,2)/2, mpcTimeStep;
        C_ << 1, 0, 0;
        Eigen::MatrixXd State_trans = Eigen::MatrixXd::Zero(3*N,N);
        Eigen::MatrixXd State_trans_th = Eigen::MatrixXd::Zero(N,N);
        Eigen::MatrixXd Init_state_mat = Eigen::MatrixXd::Zero(3*N,3);
        Eigen::MatrixXd Init_state_mat_th = Eigen::MatrixXd::Zero(N,3);
        for(int i = 0; i < 3*N; i=i+3){
            for(int j = N-1; j >= 0; j--){
                if(i == j*3)
                    State_trans.block(i,j,3,1) = B_;
                if(i > j*3)
                    State_trans.block(i,j,3,1) = A_*State_trans.block(i,j+1,3,1);
            }
            if(i == 0)
                Init_state_mat.block(0,0,3,3) = A_;
            else
                Init_state_mat.block(i,0,3,3) = A_*Init_state_mat.block(i-3,0,3,3);
                        // std::cout << "i = " << i << std::endl;
        }
        int iter = 0;
        for(int i = 0; i < 3*N; i=i+3){
            for(int j = N-1; j >= 0; j--){
                    State_trans_th.block<1,1>(iter,j) = C_*State_trans.block<3, 1>(i,j);
            }
                Init_state_mat_th.block(iter,0,1,3) = C_*Init_state_mat.block(i,0,3,3);
                iter += 1;
            }

        Eigen::Vector3d Initial_state = Eigen::Vector3d::Zero(3);
        Initial_state << currentTheta(0), currentThetaD(0), currentThetaDD(0);
        Eigen::Vector3d Initial_state_y = Eigen::Vector3d::Zero(3);
        Initial_state_y << currentTheta(1), currentThetaD(1), currentThetaDD(1);

        costFunctionHcam.block(0,0,N,N) += 0.5*((State_trans_th*Ixy*Pthddnew1mg).transpose())*Qthx*(State_trans_th*Ixy*Pthddnew1mg);
        costFunctionHcam.block(0,N,N,N) += 0.5*((State_trans_th*Ixy*Pthddnew1mg).transpose())*Qthx*(-State_trans_th*Ixx*Pthddnew1mg);
        
        costFunctionHcam.block(N,0,N,N) += 0.5*((-State_trans_th*Ixx*Pthddnew1mg).transpose())*Qthx*(State_trans_th*Ixy*Pthddnew1mg);
        costFunctionHcam.block(N,N,N,N) += 0.5*((-State_trans_th*Ixx*Pthddnew1mg).transpose())*Qthx*(-State_trans_th*Ixx*Pthddnew1mg);


        costFunctionHcam.block(0,0,N,N) += 0.5*((State_trans_th*Iyy*Pthddnew1mg).transpose())*Qthy*(State_trans_th*Iyy*Pthddnew1mg);
        costFunctionHcam.block(0,N,N,N) += 0.5*((State_trans_th*Iyy*Pthddnew1mg).transpose())*Qthy*(-State_trans_th*Iyx*Pthddnew1mg);
        
        costFunctionHcam.block(N,0,N,N) += 0.5*((-State_trans_th*Iyx*Pthddnew1mg).transpose())*Qthy*(State_trans_th*Iyy*Pthddnew1mg);
        costFunctionHcam.block(N,N,N,N) += 0.5*((-State_trans_th*Iyx*Pthddnew1mg).transpose())*Qthy*(-State_trans_th*Iyx*Pthddnew1mg);




    // Thetax squared and thetay squared H matrix
        // costFunctionHcam.block(0,0,N,N) += 0.5*((Pthddnew*Ixy*Pthddnew1mg).transpose())*Qthx*(Pthddnew*Ixy*Pthddnew1mg);
        // costFunctionHcam.block(0,N,N,N) += 0.5*((Pthddnew*Ixy*Pthddnew1mg).transpose())*Qthx*(-Pthddnew*Ixx*Pthddnew1mg);
        
        // costFunctionHcam.block(N,0,N,N) += 0.5*((-Pthddnew*Ixx*Pthddnew1mg).transpose())*Qthx*(Pthddnew*Ixy*Pthddnew1mg);
        // costFunctionHcam.block(N,N,N,N) += 0.5*((-Pthddnew*Ixx*Pthddnew1mg).transpose())*Qthx*(-Pthddnew*Ixx*Pthddnew1mg);



        // costFunctionHcam.block(0,0,N,N) += 0.5*((Pthddnew*Iyy*Pthddnew1mg).transpose())*Qthy*(Pthddnew*Iyy*Pthddnew1mg);
        // costFunctionHcam.block(0,N,N,N) += 0.5*((Pthddnew*Iyy*Pthddnew1mg).transpose())*Qthy*(-Pthddnew*Iyx*Pthddnew1mg);
        
        // costFunctionHcam.block(N,0,N,N) += 0.5*((-Pthddnew*Iyx*Pthddnew1mg).transpose())*Qthy*(Pthddnew*Iyy*Pthddnew1mg);
        // costFunctionHcam.block(N,N,N,N) += 0.5*((-Pthddnew*Iyx*Pthddnew1mg).transpose())*Qthy*(-Pthddnew*Iyx*Pthddnew1mg);

    // Construct the F vector
    Eigen::VectorXd Nzeros = Eigen::VectorXd::Zero(N);
    costFunctionFcam << Nzeros, Nzeros;


        costFunctionFcam.block(0,0,N,1) += 
        ((Init_state_mat_th*Initial_state -State_trans_th*Ixx*pmg*current_cam.comPos(1) + State_trans_th*Ixy*pmg*current_cam.comPos(0)).transpose()*Qthx*(State_trans_th*Ixy*Pthddnew1mg)).transpose();

    costFunctionFcam.block(N,0,N,1) += ((
        Init_state_mat_th*Initial_state -State_trans_th*Ixx*pmg*current_cam.comPos(1) + State_trans_th*Ixy*pmg*current_cam.comPos(0)).transpose()*Qthx*(-State_trans_th*Ixx*Pthddnew1mg)).transpose();



    costFunctionFcam.block(0,0,N,1) += ((
        Init_state_mat_th*Initial_state_y + State_trans_th*Iyy*pmg*current_cam.comPos(0) - State_trans_th*Iyx*pmg*current_cam.comPos(1)).transpose()*Qthy*(State_trans_th*Iyy*Pthddnew1mg)).transpose();

    
    costFunctionFcam.block(N,0,N,1) += qThy*Eigen::MatrixXd::Identity(N,N)*((
         Init_state_mat_th*Initial_state_y + State_trans_th*Iyy*pmg*current_cam.comPos(0) - State_trans_th*Iyx*pmg*current_cam.comPos(1)).transpose()*Qthy*(-State_trans_th*Iyx*Pthddnew1mg)).transpose();


    // Thetax squared and thetay squared F matrix

// qThx-(p*AngularPosition(0) + P*p*AngularVelocity(0) - Pthddnew*Ixx*pmg*current_cam.comPos(1)).transpose()*Pthddnew*Ixx*pmg*Pthddnew1;
// std::cout << "before fcam" << std::endl;

    // costFunctionFcam.block(0,0,N,1) += ((
    //     p*AngularPosition(0) + P*p*AngularVelocity(0) - Pthddnew*Ixx*pmg*current_cam.comPos(1) + Pthddnew*Ixy*pmg*current_cam.comPos(0)).transpose()*Qthx*(Pthddnew*Ixy*Pthddnew1mg)).transpose();

    // costFunctionFcam.block(N,0,N,1) += ((
    //     p*AngularPosition(0) + P*p*AngularVelocity(0) - Pthddnew*Ixx*pmg*current_cam.comPos(1) + Pthddnew*Ixy*pmg*current_cam.comPos(0)).transpose()*Qthx*(-Pthddnew*Ixx*Pthddnew1mg)).transpose();
    

 


    // costFunctionFcam.block(0,0,N,1) += ((
    //      p*AngularPosition(1) + P*p*AngularVelocity(1) + Pthddnew*Iyy*pmg*current_cam.comPos(0) - Pthddnew*Iyx*pmg*current_cam.comPos(1)).transpose()*Qthy*(Pthddnew*Iyy*Pthddnew1mg)).transpose();

    
    // costFunctionFcam.block(N,0,N,1) += qThy*Eigen::MatrixXd::Identity(N,N)*((
    //      p*AngularPosition(1) + P*p*AngularVelocity(1) + Pthddnew*Iyy*pmg*current_cam.comPos(0) - Pthddnew*Iyx*pmg*current_cam.comPos(1)).transpose()*Qthy*(-Pthddnew*Iyx*Pthddnew1mg)).transpose();

    // Solve QP and update state
    // *************************


    // std::cout << "AConstraintcam=" << AConstraintcam << std::endl;
    // std::cout << "bConstraintMincam=" << bConstraintMincam << std::endl;
    // std::cout << "bConstraintMaxcam=" << bConstraintMaxcam << std::endl;
    // std::cout << "costFunctionHcam=" << costFunctionHcam << std::endl;

    Eigen::VectorXd decisionVariables;
    // std::cout << "before QP" << std::endl;
    decisionVariables = solveQP_hpipm(costFunctionHcam, costFunctionFcam, AConstraintcam, bConstraintMincam, bConstraintMaxcam);
    // std::cout << "after QP" << std::endl;

    // std::cout << "decisionVariables=" << decisionVariables << std::endl;

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

    // std::cout << "current_cam.zmpPos(0) = " << current_cam.zmpPos(0) << std::endl;
    // std::cout << "next_cam.zmpPos(0) = " << next_cam.zmpPos(0) << std::endl;

    // std::cout << "xzcam_max = " << xzcam_max << std::endl;

    xz_dot_cam = zDotOptimalX_cam(0);
    yz_dot_cam = zDotOptimalY_cam(0);

    // Eigen::Vector4d footstepPredicted;
    // footstepPredicted << footstepsOptimalX(0),footstepsOptimalY(0),0.0,predictedOrientations(1);

    //std::cout << "x "<< footstepsOptimalX(0) <<std::endl;
    //std::cout << "y "<< footstepsOptimalX(1) <<std::endl;

    // next_cam.torsoOrient(2) = wrapToPi((next_cam.leftFootOrient(2) + next_cam.rightFootOrient(2)) / 2.0);

    old_fsCount = fsCount;
    ct = ct + 1;

// X-matrix way
// // std::cout << "1" << std::endl;
//         Eigen::MatrixXd A_ = Eigen::MatrixXd::Zero(3,3);
//         Eigen::MatrixXd B_ = Eigen::MatrixXd::Zero(3,1);
//         Eigen::MatrixXd C_ = Eigen::MatrixXd::Zero(1,3);
//         A_ << 1, mpcTimeStep, pow(mpcTimeStep,2)/2, 0, 1, mpcTimeStep, 0, 0, 1;
//         B_ << pow(mpcTimeStep,2)/6, pow(mpcTimeStep,2)/2, mpcTimeStep;
//         C_ << 1, 0, 0;
// // std::cout << "2" << std::endl;
//         Eigen::MatrixXd State_trans = Eigen::MatrixXd::Zero(3*N,N);
//         Eigen::MatrixXd Init_state_mat = Eigen::MatrixXd::Zero(3*N,3);
//         Eigen::Vector3d Initial_state = Eigen::Vector3d::Zero(3);
//         Initial_state << currentTheta(0), currentThetaD(0), currentThetaDD(0);
// // std::cout << "3" << std::endl;
//         for(int i = 0; i < 3*N; i=i+3){
//             for(int j = N-1; j >= 0; j--){
//                 if(i == j*3)
//                     State_trans.block(i,j,3,1) = B_;
//                 if(i > j*3)
//                     State_trans.block(i,j,3,1) = A_*State_trans.block(i,j+1,3,1);
//             }
//             if(i == 0)
//                 Init_state_mat.block(0,0,3,3) = A_;
//             else
//                 Init_state_mat.block(i,0,3,3) = A_*Init_state_mat.block(i-3,0,3,3);
//                         // std::cout << "i = " << i << std::endl;
//         }
// // std::cout << "4" << std::endl;
//         Eigen::VectorXd Next_state = Eigen::VectorXd::Zero(3*N);
//         // -Ixx*pmg*current_cam.comPos(1)-Ixx*Pthddnew1mg*yz_dot_cam ;//Ixy*pmg*current_cam.comPos(0)+Ixy*Pthddnew1mg*yz_dot_cam;
//    //     std::cout << "5" << std::endl;
//         Next_state = Init_state_mat*Initial_state + State_trans*(-Ixx*pmg*current_cam.comPos(1)-Ixx*Pthddnew1mg*zDotOptimalY_cam +Ixy*pmg*current_cam.comPos(0)+Ixy*Pthddnew1mg*zDotOptimalX_cam);
//         std::cout << "theta,thetad,thetadd = " << Next_state(0) << Next_state(1) << Next_state(2)<< std::endl;



return next_cam;
}