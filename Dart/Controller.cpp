#include "Controller.hpp"

using namespace std;
using namespace dart::dynamics;
using namespace dart::simulation;

//we can edit txt files from here
static std::ofstream fout1(realpath("../data/currentFoot.txt", NULL), std::ofstream::trunc);
static std::ofstream fout2(realpath("../data/currentCom.txt", NULL), std::ofstream::trunc);
static std::ofstream fout3(realpath("../data/desiredFoot.txt", NULL), std::ofstream::trunc);
static std::ofstream fout4(realpath("../data/desiredCom.txt", NULL), std::ofstream::trunc);
static std::ofstream fout5(realpath("../data/currentComAcc.txt", NULL), std::ofstream::trunc);
static std::ofstream fout6(realpath("../data/desiredComAcc.txt", NULL), std::ofstream::trunc);
static std::ofstream fout7(realpath("../data/jacobianComAcc.txt", NULL), std::ofstream::trunc);
static std::ofstream fout8(realpath("../data/measuredZmp.txt", NULL), std::ofstream::trunc);
static std::ofstream fout9(realpath("../data/desiredZmp.txt", NULL), std::ofstream::trunc);


static std::ofstream foutDebug(realpath("../data/debug.txt", NULL), std::ofstream::trunc);

Controller::Controller(dart::dynamics::SkeletonPtr _robot, dart::simulation::WorldPtr _world)
	: mRobot(_robot), mWorld(_world)
{
    // some useful pointers to robot limbs
    mLeftFoot = mRobot->getBodyNode("l_sole");
    mRightFoot = mRobot->getBodyNode("r_sole");
    mBase = mRobot->getBodyNode("base_link");
    mTorso = mRobot->getBodyNode("body");
    VIP = true;

    // Initialize walk state
    walkState.mpcIter = 0;
    walkState.controlIter = 0;
    walkState.footstepCounter = 0;
    walkState.supportFoot = false;

    balancePoint = TORSO;

    Eigen::Vector3d comInitialPosition;
    if (balancePoint == TORSO) {
        comInitialPosition = mBase->getCOM();
    } else {
        comInitialPosition = mRobot->getCOM();
    }

    sim_ = 1;
    std::vector<Vref> vrefSequence;

    // Plan footsteps

    for (int i = 0; i < 5000; i++) {
        if (i < 100) vrefSequence.push_back(Vref(0.0, 0.0, 0.00));
        else vrefSequence.push_back(Vref(0.00, 0.0, 0.00));
    }


    bool firstSupportFootIsLeft = false;
    Eigen::VectorXd leftFootPose(6), rightFootPose(6);
    leftFootPose << getRPY(mLeftFoot), mLeftFoot->getCOM();
    rightFootPose << getRPY(mRightFoot), mRightFoot->getCOM();

    footstepPlan = new FootstepPlan;
    footstepPlan->plan(vrefSequence, leftFootPose, rightFootPose, firstSupportFootIsLeft);

    // Instantiate MPC solver
        solver_cam = new MPCSolvercam(footstepPlan,1,true);
        solver = new MPCSolver(footstepPlan,1,false);
    
    if(!VIPsturn || !VIP){
    current.comPos << 0.0,0.0,0.0;
    current_cam.comPos << 0.0,0.0,0.0;
    current.torsoOrient << 0.0,0.0,0.0;
    current_cam.torsoOrient << 0.0,0.0,0.0;
}
    //Desired values
if(!VIPsturn || !VIP){
    desired.comPos = Eigen::Vector3d(current.comPos(0), current.comPos(1), comTargetHeight);
    desired.comVel = Eigen::Vector3d::Zero();
    desired.zmpPos = Eigen::Vector3d(current.comPos(0), current.comPos(1),0.0);
    desired.leftFootPos = Eigen::Vector3d(0.0, 0.08, 0.0);
    desired.rightFootPos = Eigen::Vector3d(0.0, -0.08, 0.0);
    desired.torsoOrient = Eigen::Vector3d(0.0, 0.0, 0.0);
    desired.leftFootOrient = Eigen::Vector3d(0.0, 0.0, 0.0);
    desired.rightFootOrient = Eigen::Vector3d(0.0, 0.0, 0.0);
    desired.comAcc = omega*omega * (desired.comPos - desired.zmpPos);
}
    //Desired cam values
if(VIPsturn && VIP){
    desired_cam.comPos = Eigen::Vector3d(current_cam.comPos(0), current_cam.comPos(1), comTargetHeight);
    desired_cam.comVel = Eigen::Vector3d::Zero();
    desired_cam.zmpPos = Eigen::Vector3d(current_cam.comPos(0), current_cam.comPos(1),0.0);
    desired_cam.leftFootPos = Eigen::Vector3d(0.0, 0.08, 0.0);
    desired_cam.rightFootPos = Eigen::Vector3d(0.0, -0.08, 0.0);
    desired_cam.torsoOrient = Eigen::Vector3d(0.0, 0.0, 0.0);
    desired_cam.leftFootOrient = Eigen::Vector3d(0.0, 0.0, 0.0);
    desired_cam.rightFootOrient = Eigen::Vector3d(0.0, 0.0, 0.0);
    desired_cam.comAcc = omega*omega * (desired_cam.comPos - desired_cam.zmpPos);
}

    px = 0.0;
    py = 0.0;

    // Instantiate new Filter (new is for memory allocation and referencing!)
    Eigen::Matrix2f input_noise_cov; input_noise_cov << 1000.0f, 0.0f, 0.0f, 1000.0f;
    Eigen::Matrix3f meas_noise_cov; meas_noise_cov << 0.01f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f;
    Eigen::Vector3f in_st_x; in_st_x << (float) current.comPos(0), 0.0f, 0.0f;
    Eigen::Vector3f in_st_y; in_st_y << (float) current.comPos(1), 0.0f, 0.0f;
    Eigen::Vector3f in_st_z; in_st_z << (float) comTargetHeight + 0.03f, 0.0f, 0.0f;
    float h = (float) comTargetHeight + 0.03f; //needed correction
    float mass = 50.055f;
    float delta_t = 0.01f;

    Eigen::Matrix2f input_noise_cov_z = 1.0f*input_noise_cov;
    Eigen::Matrix3f meas_noise_cov_z; meas_noise_cov_z << 0.01f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.01f;

    // Filter = new StateFiltering(in_st_x, input_noise_cov, meas_noise_cov,
    //                             in_st_y, input_noise_cov, meas_noise_cov,
    //                             in_st_z, input_noise_cov_z, meas_noise_cov_z,
    //                             h, mass, delta_t);

    // std::cout << "mass " << mRobot->getMass() << std::endl;
    mRobotMass = mRobot->getMass();
    mAngularVelocity = Eigen::Vector3d::Zero(3);
    mAngularPosition = current_cam.torsoOrient;
}

Controller::~Controller() {}



void Controller::update() {
    // STOP EXECUTION AFTER 6 SECONDS
    if (mWorld->getSimFrames() == 2000) exit(1);

    // if(mWorld->getSimFrames() == 20){
    //     for(int i = 0; i<mRobot->getNumDofs(); i++)
    //     {
    //         std::cout << "i = "<< i << ", Bodynode is = " << mRobot->getDof(i)->getName() << std::endl;
    //     }
    // }
// std::cout << "mRobot->getCommands = " << mRobot->getCommands() << std::endl;
    auto visualShapeNodes = mTorso->getShapeNodesWith<VisualAspect>();


    // if(visualShapeNodes.size() == 3u )
    // {
    // //assert(visualShapeNodes[2]->getShape() == mArrow);
    //     visualShapeNodes[2]->remove();
    // }
		//
    // if(visualShapeNodes.size() == 4u )
    // {
    // //assert(visualShapeNodes[2]->getShape() == mArrow);
    //     visualShapeNodes[2]->remove();
    //     visualShapeNodes[3]->remove();
    // }



    //if (mWorld->getSimFrames()==500) system("gnuplot ../plotters/plot");
    walkState.simulationTime = mWorld->getSimFrames();

    // Retrieve current position and orientation of com and feet in the world frame
    if(!VIPsturn || !VIP){
    current.comPos = balancePoint == TORSO ? mBase->getCOM() : mRobot->getCOM();
    current.comVel = balancePoint == TORSO ? mBase->getCOMLinearVelocity() : mRobot->getCOMLinearVelocity();
    current.comAcc = balancePoint == TORSO ? mBase->getCOMLinearAcceleration() : mRobot->getCOMLinearAcceleration();
}

/*
    current.comPos = mTorso->getCOM();
    current.comVel = mTorso->getCOMLinearVelocity();
    current.comAcc = mTorso->getCOMLinearAcceleration();
/**/
    if(!VIPsturn || !VIP){
    current.zmpPos = current.comPos - current.comAcc / (omega*omega);
    current.leftFootPos = mLeftFoot->getCOM();
    current.rightFootPos = mRightFoot->getCOM();
    current.torsoOrient = getRPY(mBase);
    // std::cout << "current.torsoOrient = " << current.torsoOrient << std::endl;
    current.leftFootOrient = getRPY(mLeftFoot);
    current.rightFootOrient = getRPY(mRightFoot);
}


// exit(1);
    //desired.leftFootPos =  current.leftFootPos;
    //desired.rightFootPos =  current.rightFootPos;

    // Compute the new desired state using MPC
    if(!VIPsturn || !VIP) desired = solver->solve(desired, desired_cam, walkState, 0.0, 0.0, 0.0, 0.0, 0.0); // we start from desired because mpc is open loop right now

        // desired = solver->solve(desired, walkState, 0.0, 0.0, 0.0, 0.0, 0.0); // we start from desired because mpc is open loop right now
    //std::cout << "error " << desired.comPos - current.comPos << std::endl;
    //std::cout << "desired left " << desired.leftFootPos << " desired right " << desired.rightFootPos <<std::endl;
    //std::cout << "getFootstep " << footstepPlan->getFootstep(walkState.footstepCounter) << std::endl;
    //std::cout << "current left " << current.leftFootPos << " current right " << current.rightFootPos <<std::endl;
    // if(!VIPsturn)     std::cout << "desired.torsoOrient = " << desired.torsoOrient << std::endl;
    //mTorso->addExtForce(39.0*(solver->push));


    //Timing adaptation
    // play here! -- for now, just an example

    // if (walkState.footstepCounter > 1 ){ //(walkState.footstepCounter > 1  && walkState.footstepCounter <= 4 )
    //    walkState.controlIter = walkState.controlIter + solver->prova;
    //  }

    // Compute inverse kinematics
    Eigen::VectorXd qDot = Eigen::VectorXd::Zero(50);
        if(!VIPsturn || !VIP){
        qDot =  getJointVelocitiesStacked(desired.getRelComPose(walkState.supportFoot),  current.getRelComPose(walkState.supportFoot),
         desired.getRelSwingFootPose(walkState.supportFoot),  current.getRelSwingFootPose(walkState.supportFoot), desired.comVel);
 /**/

    // Set the velocity of the floating base to zero
    for (int i = 0; i < 6; ++i){
        mRobot->setCommand(i, 0);
    }

    // Set the velocity of each joint as per inverse kinematics
    for (int i = 0; i < 50; ++i){
        mRobot->setCommand(i+6,qDot(i)); //velocity joint control
    }
}
        if(VIPsturn && VIP){
    current_cam.zmpPos = current_cam.comPos - current_cam.comAcc / (omega*omega);
    current_cam.leftFootPos = mLeftFoot->getCOM();
    current_cam.rightFootPos = mRightFoot->getCOM();
    current_cam.torsoOrient = getRPY(mBase);
    current_cam.leftFootOrient = getRPY(mLeftFoot);
    current_cam.rightFootOrient = getRPY(mRightFoot);
}
    //Eigen::VectorXd qDot =  getJointVelocitiesQp(current, desired);
        Eigen::Vector3d desiredTorques = Eigen::Vector3d::Zero();
        Eigen::Matrix3d MOI = getMomentOfInertia();
        Eigen::Vector3d angularAcceleration = Eigen::Vector3d::Zero();
        if(VIP && VIPsturn){
        desired_cam.comPos = mRobot->getCOM()-desired.comPos;
        desired_cam = solver_cam->solve(desired, desired_cam, walkState, 0.0, 0.0, 0.0, 0.0, 0.0); // we start from desired because mpc is open loop right now;
        
        desiredTorques = computeTorques(desired_cam, desired);
        angularAcceleration = MOI.inverse()*desiredTorques;
        mAngularPosition+= mAngularVelocity*mpcTimeStep + angularAcceleration*0.5*pow(mpcTimeStep,2);
        mAngularVelocity+= angularAcceleration*mpcTimeStep;
        desired_cam.torsoOrient = mAngularPosition;
        desired_cam.comVel = mAngularVelocity;
        // std::cout<< "qdot = " << qDot << std::endl;
        qDot =  getJointVelocitiesStackedcam(desired_cam.getRelComPose(walkState.supportFoot), current_cam.getRelComPose(walkState.supportFoot), 
            desired_cam.comVel);
    // Set the velocity of the floating base to zero
    for (int i = 0; i < 6; ++i){
        mRobot->setCommand(i, 0);
    }

    // Set the velocity of each joint as per inverse kinematics
    for (int i = 0; i < 50; ++i){

        // if(i == 46) std::cout << " before 2nd command = " << mRobot->getCommand(i+6);
        mRobot->setCommand(i+6,qDot(i)); //velocity joint control
        // if(i == 46) std::cout << " after 2nd command = " << mRobot->getCommand(i+6);
    }
}

    // Eigen::VectorXd qDot =  getJointVelocitiesStacked(desired.getRelComPose(walkState.supportFoot),  current.getRelComPose(walkState.supportFoot),
    // desired.getRelSwingFootPose(walkState.supportFoot),  current.getRelSwingFootPose(walkState.supportFoot), desired.comVel, Eigen::Vector3d::Zero(3));

    // // Set the velocity of the floating base to zero
    // for (int i = 0; i < 6; ++i){
    //     mRobot->setCommand(i, 0);
    // }

    // // Set the velocity of each joint as per inverse kinematics
    // for (int i = 0; i < 50; ++i){
    //     mRobot->setCommand(i+6,qDot(i)); //velocity joint control
    // }

    // Store the results in files (for plotting)
    //storeData();

    // Arm swing
    // ArmSwing();
    if(!VIP || VIPsturn){
    AnkleRegulation();
}
    // std::cout << angularAcceleration << std::endl;

    // Eigen::Vector3d AngularMomentum = ComputeAngularMomentum();

    // Update the iteration counters, if a step is finished reset and change support foot

    // std::cout<< "VIPsturn is " << VIPsturn << ", and controliter is " << walkState.controlIter << std::endl;
    // std::cout<< "mpcIter = " << walkState.mpcIter << std::endl;

    if(!VIP || VIPsturn){
    ++walkState.controlIter;
}

if(!VIP || VIPsturn){
    walkState.mpcIter = floor(walkState.controlIter*controlTimeStep/mpcTimeStep);

    if (footstepPlan->getFootstepStartTiming(walkState.footstepCounter) + walkState.mpcIter >= footstepPlan->getFootstepEndTiming(walkState.footstepCounter)) {
    	walkState.controlIter = 0;
    	walkState.mpcIter = 0;
        walkState.footstepCounter++;
	walkState.supportFoot = !walkState.supportFoot;
        // std::cout << "Iteration " << walkState.controlIter << " Footstep " << walkState.footstepCounter << std::endl;
        // std::cout <<  mWorld->getSimFrames() << std::endl;
    }
}
// std::cout << "controlIter = " << walkState.controlIter << '\n';
// std::cout << "mpcIter = " << walkState.mpcIter << '\n';

VIPsturn = !VIPsturn;
}


//Compute the desired torques from the desired states
Eigen::Vector3d Controller::computeTorques(const State& desired_cam,const State& desired)
{
    Eigen::Vector3d mDesiredTorque = Eigen::Vector3d::Zero(3);
    double totalCOMx, totalCOMy;
    totalCOMx = mRobot->getCOM()(0);
    totalCOMy = mRobot->getCOM()(1);
    mDesiredTorque(0) = -desired_cam.comPos(1)*mRobotMass*9.81;
    mDesiredTorque(1) = desired_cam.comPos(0)*mRobotMass*9.81;
    // mDesiredTorque(2) = ((totalCOMx-desired.zmpPos(0))* mDesiredTorque(0) 
    //     + (totalCOMy-desired.zmpPos(1))*mDesiredTorque(1))/comTargetHeight;

    // std::cout << "COM acceleration" << desired.comAcc(2) << std::endl;
    // std::cout << "desired.comPos(0) + desired_cam.comPos(0)" << desired.comPos(0) + desired_cam.comPos(0) << std::endl;
    // std::cout << "totalCOMx" << totalCOMx << std::endl;
    return mDesiredTorque;
}

Eigen::Vector3d Controller::computeAngularMomentum(){ //FIXME: Does this get the angular momentum with respect to the COM or wrt the body itself 
    Eigen::Vector3d AngularMomentum = Eigen::Vector3d::Zero(3);
    // std::cout << "mRobot->getBodyNode(i)->getName :"<< mRobot->getBodyNode(1)->getName() << '\n';
    for(int i = 0; i < mRobot->getNumBodyNodes(); ++i){
        AngularMomentum += mRobot->getBodyNode(i)->getAngularMomentum(mTorso->getCOM());  //FIXME
        if(mWorld->getSimFrames() == 100)
        std::cout << mRobot->getBodyNode(i)->getName() << " has ang momentum of : " << mRobot->getBodyNode(i)->getAngularMomentum() << std::endl;
    }
    return AngularMomentum;
}

Eigen::Matrix3d Controller::getMomentOfInertia(){
    Eigen::Matrix3d MOI = Eigen::Matrix3d::Zero();
    Inertia inertia;
    inertia.setMass(mTorso->getMass());
    // std::cout << "mass = " << mTorso->getMass() << std::endl;
    MOI << 0.344345670122806, -5.676994253777424e-04, 0.045048560212699, 
    -5.676994253777424e-04, 0.338324801980916, 0.003172978374495,
     0.045048560212699, 0.003172978374495, 0.048160214086886;
    // std::cout << "inertia = " << MOI << std::endl;
    return MOI;
}

void Controller::AnkleRegulation() {

Eigen::Vector3d GYRO = mTorso->getAngularVelocity();

double K_pitch = -0.05*(3.14/180.0);  //0.3*(3.14/180.0)
double K_roll= -0.05*(3.14/180.0);    //0.1*(3.14/180.0)

// right ankle pitch
mRobot->setPosition(mRobot->getDof("R_ANKLE_P")->getIndexInSkeleton(), mRobot->getPosition(mRobot->getDof("R_ANKLE_P")->getIndexInSkeleton())+K_pitch*(GYRO(1)-mRobot->getPosition(mRobot->getDof("R_ANKLE_P")->getIndexInSkeleton())));
// right ankle roll
mRobot->setPosition(mRobot->getDof("R_ANKLE_R")->getIndexInSkeleton(), mRobot->getPosition(mRobot->getDof("R_ANKLE_R")->getIndexInSkeleton())+K_roll*(GYRO(0)-mRobot->getPosition(mRobot->getDof("R_ANKLE_R")->getIndexInSkeleton())));
// left ankle pitch
mRobot->setPosition(mRobot->getDof("L_ANKLE_P")->getIndexInSkeleton(), mRobot->getPosition(mRobot->getDof("L_ANKLE_P")->getIndexInSkeleton())+K_pitch*(GYRO(1)-mRobot->getPosition  (mRobot->getDof("L_ANKLE_P")->getIndexInSkeleton())));
// left ankle roll
mRobot->setPosition(mRobot->getDof("L_ANKLE_R")->getIndexInSkeleton(), mRobot->getPosition(mRobot->getDof("L_ANKLE_R")->getIndexInSkeleton())+K_roll*(GYRO(0)-mRobot->getPosition(mRobot->getDof("L_ANKLE_R")->getIndexInSkeleton())));
}

Eigen::MatrixXd Controller::getTorsocamJacobian() { //FIXME this is still expressed in relative frame
    if (walkState.supportFoot == true) {
        mSupportFoot = mRobot->getBodyNode("r_sole");
    } else {
        mSupportFoot = mRobot->getBodyNode("l_sole");
    }

    Eigen::MatrixXd Jacobian_supportToBase;

    // Jacobian_supportToBase =  mRobot->getAngularJacobian(mTorso,mSupportFoot) - mRobot->getAngularJacobian(mSupportFoot,mSupportFoot); //(mBase,mSupportFoot)

    Jacobian_supportToBase = mRobot->getJacobian(mTorso,mSupportFoot) - mRobot->getJacobian(mSupportFoot,mSupportFoot);


    // std::cout << "Jacobian_supportToBase size: (" << Jacobian_supportToBase.rows() << ", " << Jacobian_supportToBase.cols() << ")" << std::endl;

    // std::cout << "mRobot->getJacobian(mSupportFoot,mSupportFoot)-mRobot->getJacobian(mSupportFoot) = " 
    // << mRobot->getJacobian(mSupportFoot,mSupportFoot)-mRobot->getJacobian(mSupportFoot);

    // for (unsigned int i=0; i< Jacobian_supportToBase.cols(); ++i){ 
    // if(i!=44 && i!=45 && i!=46 && i!=50 && i!=51 && i!=52)
    // Jacobian_supportToBase.col(i).setZero();
    // }
    // std::cout << "(" << Jacobian_SupportToSwing.rows() << ", " << Jacobian_SupportToSwing.cols() << ")" << std::endl;

    Eigen::MatrixXd Jacobian_tot_(6, 56);

    Jacobian_tot_ << Jacobian_supportToBase;

    Eigen::MatrixXd Jacobian_tot(6, 50);

    // Remove the floating base columns
    Jacobian_tot = Jacobian_tot_.block<6,50>(0, 6);

    // std::cout << "OKAY until here" << std::endl;    

    return Jacobian_tot;
}


Eigen::MatrixXd Controller::getTorsoAndSwfJacobian() { //FIXME this is still expressed in relative frame
    if (walkState.supportFoot == true) {
        mSupportFoot = mRobot->getBodyNode("r_sole");
        mSwingFoot = mRobot->getBodyNode("l_sole");
    } else {
        mSupportFoot = mRobot->getBodyNode("l_sole");
        mSwingFoot = mRobot->getBodyNode("r_sole");
    }

    Eigen::MatrixXd Jacobian_supportToBase;

    if (balancePoint == TORSO) {
    Jacobian_supportToBase =  mRobot->getJacobian(mTorso,mSupportFoot) - mRobot->getJacobian(mSupportFoot,mSupportFoot); //(mBase,mSupportFoot)
    } else {
    Jacobian_supportToBase =  (mRobot->getCOMJacobian(mSupportFoot) - mRobot->getJacobian(mSupportFoot,mSupportFoot));
    }

    // for (unsigned int i=0; i<44; i++){ 
    // if (i!=5 || i!=6)  Jacobian_supportToBase.col(i).setZero();
    // }

    Eigen::MatrixXd Jacobian_SupportToSwing =  mRobot->getJacobian(mSwingFoot,mSupportFoot) - mRobot->getJacobian(mSupportFoot,mSupportFoot);

    // std::cout << "(" << Jacobian_SupportToSwing.rows() << ", " << Jacobian_SupportToSwing.cols() << ")" << std::endl;

    Eigen::MatrixXd Jacobian_tot_(12, 56);

    Jacobian_tot_ << Jacobian_supportToBase, Jacobian_SupportToSwing;

    Eigen::MatrixXd Jacobian_tot(12, 50);

    // Remove the floating base columns
    Jacobian_tot = Jacobian_tot_.block<12,50>(0, 6);

    return Jacobian_tot;
}

Eigen::MatrixXd Controller::getTorsoAndSwfJacobianDeriv() {
    if (walkState.supportFoot == true) {
        mSupportFoot = mRobot->getBodyNode("r_sole");
        mSwingFoot = mRobot->getBodyNode("l_sole");
    } else {
        mSupportFoot = mRobot->getBodyNode("l_sole");
        mSwingFoot = mRobot->getBodyNode("r_sole");
    }

    Eigen::MatrixXd Jacobian_supportToBaseDeriv;

    if (balancePoint == TORSO) {
    Jacobian_supportToBaseDeriv =  mRobot->getJacobianSpatialDeriv(mTorso,mSupportFoot) - mRobot->getJacobianSpatialDeriv(mSupportFoot,mSupportFoot); //(mBase,mSupportFoot)
    } else {
    Jacobian_supportToBaseDeriv =  (mRobot->getJacobianSpatialDeriv(mSupportFoot) - mRobot->getJacobianSpatialDeriv(mSupportFoot,mSupportFoot));
    }

    // for (unsigned int i=0; i<44; i++){ 
    // if (i!=5 || i!=6)  Jacobian_supportToBaseDeriv.col(i).setZero();
    // }

    Eigen::MatrixXd Jacobian_SupportToSwingDeriv =  mRobot->getJacobianClassicDeriv(mSwingFoot,mSupportFoot) - mRobot->getJacobianClassicDeriv(mSupportFoot,mSupportFoot);

    Eigen::MatrixXd Jacobian_tot_(12, 56);

    Jacobian_tot_ << Jacobian_supportToBaseDeriv, Jacobian_SupportToSwingDeriv;

    Eigen::MatrixXd Jacobian_tot(12, 50);

    // Remove the floating base columns
    Jacobian_tot = Jacobian_tot_.block<12,50>(0, 6);

    return Jacobian_tot;


}

Eigen::VectorXd Controller::getJointVelocitiesQp(State current, State desired) {
    int nVariables = 50;

    double jointVelocitiesGain = 0.00001;//0.00001;
    Eigen::MatrixXd taskGain = Eigen::MatrixXd::Identity(12,12);

    // Torso Orientation
    taskGain(0,0) = 1;
    taskGain(1,1) = 1;
    taskGain(2,2) = 1;

    // CoM Position
    taskGain(3,3) = 5;
    taskGain(4,4) = 5;
    taskGain(5,5) = 1;


    // Swing Foot Orientation
    taskGain(6,6) = 5;
    taskGain(7,7) = 5;
    taskGain(8,8) = 1;

    // Swing Foot Position
    taskGain(9,9) = 5;
    taskGain(10,10) = 5;
    taskGain(11,11) = 5;

    // Construct stack of error, wrap to Pi angular errors
    Eigen::VectorXd desired_pos(12);
    desired_pos << desired.getRelComPose(walkState.supportFoot), desired.getRelSwingFootPose(walkState.supportFoot);
    Eigen::VectorXd actual_pos(12);
    actual_pos << current.getRelComPose(walkState.supportFoot), current.getRelSwingFootPose(walkState.supportFoot);


    Eigen::VectorXd errorStack = actual_pos - desired_pos;
    errorStack(0) = wrapToPi(errorStack(0));
    errorStack(1) = wrapToPi(errorStack(1));
    errorStack(2) = wrapToPi(errorStack(2));
    errorStack(6) = wrapToPi(errorStack(6));
    errorStack(7) = wrapToPi(errorStack(7));
    errorStack(8) = wrapToPi(errorStack(8));

    // Cost Function
    Eigen::MatrixXd Jacobian_tot = getTorsoAndSwfJacobian();
    Eigen::MatrixXd costFunctionH = mWorld->getTimeStep()*mWorld->getTimeStep()*Jacobian_tot.transpose()*taskGain*Jacobian_tot +
            jointVelocitiesGain*Eigen::MatrixXd::Identity(nVariables,nVariables);

    Eigen::VectorXd costFunctionF = mWorld->getTimeStep()*Jacobian_tot.transpose() * taskGain * IKerrorGain * errorStack;

    // Constraint RHipYawPitch and LHipYawPitch to be at the same angle
    // not working on HRP4
    Eigen::MatrixXd AHip = Eigen::MatrixXd::Zero(1,nVariables);
    AHip(0,52-6) = -mWorld->getTimeStep();
    AHip(0,46-6) = mWorld->getTimeStep();
    Eigen::VectorXd bHip(1);
    bHip(0) = mRobot->getPosition(54-6) - mRobot->getPosition(48-6);

    // Solve the QP
    Eigen::VectorXd solution = solveQP(costFunctionH, costFunctionF, 0*AHip, 0*bHip, 0*bHip);

    return solution;
}
Eigen::VectorXd Controller::getJointVelocitiesStackedcam(Eigen::VectorXd desidercam_pos_base, 
    Eigen::VectorXd actcamPosBase, Eigen::VectorXd desidercam_com_vel){


    Eigen::VectorXd ComVref = Eigen::VectorXd::Zero(6);
    ComVref << desidercam_com_vel(0), desidercam_com_vel(1), desidercam_com_vel(2), 0.0, 0.0, 0.0;

    Eigen::VectorXd desired_pos(6);
    desired_pos << desidercam_pos_base;

    // Assemble actual positions and orientations
    Eigen::VectorXd actual_pos(6);
    actual_pos << actcamPosBase;


    // Get the proper jacobian and pseudoinvert it
    Eigen::MatrixXd Jacobian_tot = getTorsocamJacobian();
    Eigen::MatrixXd PseudoJacobian_tot = (Jacobian_tot.transpose())*(Jacobian_tot*Jacobian_tot.transpose()).inverse();

    Eigen::MatrixXd _taskGain = Eigen::MatrixXd::Identity(6,6);
        // Torso Orientation
    _taskGain(0,0) = 1;
    _taskGain(1,1) = 1;//0.001;
    _taskGain(2,2) = 0.1;//0;

        // CoM Position
    _taskGain(3,3) = 0;  //0.1  10
    _taskGain(4,4) = 0;
    _taskGain(5,5) = 0;


    double ikGainvel = 0;
    double ikGain = 0;

    // std::cout << "PseudoJacobian_tot = " << PseudoJacobian_tot << std::endl;

    Eigen::VectorXd qDot(50);
    qDot = PseudoJacobian_tot*(ikGainvel*ComVref+ikGain*_taskGain*(desired_pos - actual_pos));

    // std::cout << "qdotcam = " << qDot << std::endl;
    return qDot;
}

Eigen::VectorXd Controller::getJointVelocitiesStacked(Eigen::VectorXd desider_pos_base, Eigen::VectorXd actPosBase,
        Eigen::VectorXd desider_pos_SwingFoot, Eigen::VectorXd actPosSwingFoot, Eigen::VectorXd desider_com_vel){

        Eigen::VectorXd ComVref = Eigen::VectorXd::Zero(12);
     
        ComVref<<0.0, 0.0, 0.0, desider_com_vel(0), desider_com_vel(1), desider_com_vel(2),0.0,0.0,0.0,0.0,0.0,0.0;

    Eigen::VectorXd desired_pos(12);
    desired_pos << desider_pos_base, desider_pos_SwingFoot;

    // Assemble actual positions and orientations
    Eigen::VectorXd actual_pos(12);
    actual_pos << actPosBase, actPosSwingFoot;

    // Get the proper jacobian and pseudoinvert it
    Eigen::MatrixXd Jacobian_tot = getTorsoAndSwfJacobian();
    Eigen::MatrixXd PseudoJacobian_tot = (Jacobian_tot.transpose())*(Jacobian_tot*Jacobian_tot.transpose()).inverse();

    Eigen::MatrixXd _taskGain = Eigen::MatrixXd::Identity(12,12);


        //// VERY GOOD
    // Torso Orientation
    _taskGain(0,0) = 0;
    _taskGain(1,1) = 0;//0.001;
    _taskGain(2,2) = 0;//0;

    // CoM Position
    _taskGain(3,3) = 5;  //0.1  10
    _taskGain(4,4) = 5;
    _taskGain(5,5) = 1;

    // Swing Foot Orientation
    _taskGain(6,6) = 1;
    _taskGain(7,7) = 1;
    _taskGain(8,8) = 1;

    // Swing Foot Position
    _taskGain(9,9) = 5;
    _taskGain(10,10) = 5;
    _taskGain(11,11) = 5;


        double ikGain = 10;

    Eigen::VectorXd qDot(50);
    qDot = PseudoJacobian_tot*(ComVref+ikGain*_taskGain*(desired_pos - actual_pos));

    return qDot;
}

Eigen::VectorXd Controller::getJointVelocitiesQpAcceleration(State current, State desired) {
    int nVariables = 24;

    // Fetch Jacobian and Jacobian derivative
    Eigen::MatrixXd jac = getTorsoAndSwfJacobian();
    Eigen::MatrixXd jacDot = getTorsoAndSwfJacobianDeriv();

    Eigen::VectorXd q(24);
    Eigen::VectorXd qDot(24);
    Eigen::VectorXd qDoubleDot(24);

    for (int i = 0; i < 24; ++i){
        q(i) = mRobot->getPosition(i+6);
        qDot(i) = mRobot->getVelocity(i+6);
        qDoubleDot(i) = mRobot->getAcceleration(i+6);
    }

    // Construct stack of error, wrap to Pi angular errors
    Eigen::VectorXd desiredPos(12);
    desiredPos << desired.getRelComPose(walkState.supportFoot), desired.getRelSwingFootPose(walkState.supportFoot);
    Eigen::VectorXd actual_pos(12);
    actual_pos << current.getRelComPose(walkState.supportFoot), current.getRelSwingFootPose(walkState.supportFoot);

    Eigen::VectorXd errorStack = actual_pos - desiredPos;
    errorStack(0) = wrapToPi(errorStack(0));
    errorStack(1) = wrapToPi(errorStack(1));
    errorStack(2) = wrapToPi(errorStack(2));
    errorStack(6) = wrapToPi(errorStack(6));
    errorStack(7) = wrapToPi(errorStack(7));
    errorStack(8) = wrapToPi(errorStack(8));

    // Get desired velocity and acceleration
    Eigen::VectorXd desiredVel(12);
    desiredVel << Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6);
    Eigen::VectorXd desiredAcc(12);
    desiredAcc << Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6);

    // Cost Function
    double delta = mWorld->getTimeStep();
    double alpha = 0;
    double beta = 0;
    double gamma = 100000;
    
    Eigen::MatrixXd costFunctionH = alpha * (jac + jacDot * delta).transpose() * (jac + jacDot * delta) +
                                    beta  * (jac * delta).transpose() * (jac * delta) +
                                    gamma * (jac * delta * delta).transpose() * (jac * delta * delta) +
                                    Eigen::MatrixXd::Identity(nVariables, nVariables);

    Eigen::VectorXd costFunctionF = alpha * (jac + jacDot * delta).transpose() * (jacDot * qDot - desiredAcc) +
                                    beta  * (jac * delta).transpose() * (jac * qDot - desiredVel + delta * jac * qDoubleDot) +
                                    gamma * (jac * delta * delta).transpose() * (errorStack + jac * qDot);

    // Dummy constraint
    Eigen::MatrixXd AHip = Eigen::MatrixXd::Zero(1,nVariables);
    Eigen::VectorXd bHip(1);
    bHip(0) = 0;

    // Solve the QP
    Eigen::VectorXd solution = solveQP(costFunctionH, costFunctionF, AHip, bHip, bHip);
    return solution;
}

Eigen::Vector3d Controller::getRPY(dart::dynamics::BodyNode* body, dart::dynamics::BodyNode* referenceFrame) {
    Eigen::MatrixXd rotMatrix = body->getTransform(referenceFrame).rotation();

    Eigen::Vector3d RPY;
    RPY << atan2(rotMatrix(2,1),rotMatrix(2,2)),
        atan2(-rotMatrix(2,0),sqrt(rotMatrix(2,1)*rotMatrix(2,1)+rotMatrix(2,2)*rotMatrix(2,2))),
        atan2(rotMatrix(1,0),rotMatrix(0,0));

    return RPY;
}

Eigen::Vector3d Controller::getRPY(dart::dynamics::BodyNode* body) {
    Eigen::MatrixXd rotMatrix = body->getTransform().rotation();

    Eigen::Vector3d RPY;
    RPY << atan2(rotMatrix(2,1),rotMatrix(2,2)),
        atan2(-rotMatrix(2,0),sqrt(rotMatrix(2,1)*rotMatrix(2,1)+rotMatrix(2,2)*rotMatrix(2,2))),
        atan2(rotMatrix(1,0),rotMatrix(0,0));

    return RPY;
}

Eigen::Vector3d Controller::getZmpFromExternalForces()
{
    Eigen::Vector3d zmp_v;
    bool left_contact = false;
    bool right_contact = false;

    Eigen::Vector3d left_cop;
    if(abs(mLeftFoot->getConstraintImpulse()[5]) > 0.01){
        left_cop << -mLeftFoot->getConstraintImpulse()(1)/mLeftFoot->getConstraintImpulse()(5), mLeftFoot->getConstraintImpulse()(0)/mLeftFoot->getConstraintImpulse()(5), 0.0;
        Eigen::Matrix3d iRotation = mLeftFoot->getWorldTransform().rotation();
        Eigen::Vector3d iTransl   = mLeftFoot->getWorldTransform().translation();
        left_cop = iTransl + iRotation*left_cop;
        left_contact = true;
    }

    Eigen::Vector3d right_cop;
    if(abs(mRightFoot->getConstraintImpulse()[5]) > 0.01){
        right_cop << -mRightFoot->getConstraintImpulse()(1)/mRightFoot->getConstraintImpulse()(5), mRightFoot->getConstraintImpulse()(0)/mRightFoot->getConstraintImpulse()(5), 0.0;
        Eigen::Matrix3d iRotation = mRightFoot->getWorldTransform().rotation();
        Eigen::Vector3d iTransl   = mRightFoot->getWorldTransform().translation();
        right_cop = iTransl + iRotation*right_cop;
        right_contact = true;
    }

    if(left_contact && right_contact){
        zmp_v << (left_cop(0)*mLeftFoot->getConstraintImpulse()[5] + right_cop(0)*mRightFoot->getConstraintImpulse()[5])/(mLeftFoot->getConstraintImpulse()[5] + mRightFoot->getConstraintImpulse()[5]),
                 (left_cop(1)*mLeftFoot->getConstraintImpulse()[5] + right_cop(1)*mRightFoot->getConstraintImpulse()[5])/(mLeftFoot->getConstraintImpulse()[5] + mRightFoot->getConstraintImpulse()[5]),
         0.0;
    }else if(left_contact){
        zmp_v << left_cop(0), left_cop(1), 0.0;
    }else if(right_contact){
        zmp_v << right_cop(0), right_cop(1), 0.0;
    }else{
        // No contact detected
        zmp_v << 0.0, 0.0, 0.0;
    }

    return zmp_v;
}

void Controller::keyboard(unsigned char /*_key*/, int /*_x*/, int /*_y*/) {}

void Controller::setInitialConfiguration() {
  Eigen::VectorXd q = mRobot->getPositions();


    // Floating Base
    q[0] = 0.0;
    q[1] = 4*M_PI/180;
    q[2] = 0.0;
    q[3] = 0.00;
    q[4] = 0.00;
    q[5] = 0.753;

    // Right Leg
    q[44] = 0.0;            // hip yaw
    q[45] = 3*M_PI/180;    // hip roll
    q[46] = -25*M_PI/180;   // hip pitch
    q[47] = 50*M_PI/180;   // knee pitch
    q[48] = -30*M_PI/180; // ankle pitch
    q[49] = -4*M_PI/180;   // ankle roll         
    // Left Leg
    q[50] = 0.0;            // hip yaw
    q[51] = -3*M_PI/180;    // hip roll
    q[52] = -25*M_PI/180;   // hip pitch
    q[53] = 50*M_PI/180;   // knee pitch
    q[54] = -30*M_PI/180; // ankle pitch
    q[55] = 4*M_PI/180;   // ankle roll       

    mRobot->setPositions(q);

    // Additional arm position setting
    mRobot->setPosition(mRobot->getDof("R_SHOULDER_P")->getIndexInSkeleton(), (4)*M_PI/180 );
    mRobot->setPosition(mRobot->getDof("R_SHOULDER_R")->getIndexInSkeleton(), -8*M_PI/180  );
    mRobot->setPosition(mRobot->getDof("R_SHOULDER_Y")->getIndexInSkeleton(), 0 );

    mRobot->setPosition(mRobot->getDof("R_ELBOW_P")->getIndexInSkeleton(), -25*M_PI/180 );
    mRobot->setPosition(mRobot->getDof("R_ELBOW_P")->getIndexInSkeleton(), -25*M_PI/180 );

    mRobot->setPosition(mRobot->getDof("L_SHOULDER_P")->getIndexInSkeleton(), (4)*M_PI/180  );
    mRobot->setPosition(mRobot->getDof("L_SHOULDER_R")->getIndexInSkeleton(), 8*M_PI/180  );
    mRobot->setPosition(mRobot->getDof("L_SHOULDER_Y")->getIndexInSkeleton(), 0 );

    mRobot->setPosition(mRobot->getDof("L_ELBOW_P")->getIndexInSkeleton(), -25*M_PI/180 ); 
    mRobot->setPosition(mRobot->getDof("L_ELBOW_P")->getIndexInSkeleton(), -25*M_PI/180 ); 
}


void Controller::ArmSwing() {
    // std::cout << "mRobot->getDof(\"R_SHOULDER_P\")->getIndexInSkeleton() = " << mRobot->getDof("R_SHOULDER_P")->getIndexInSkeleton();
    // std::cout << "mRobot->getDof(\"L_SHOULDER_P\")->getIndexInSkeleton() = " << mRobot->getDof("L_SHOULDER_P")->getIndexInSkeleton();

// TO DO: add variable period to the swing trajecotry
mRobot->setPosition(mRobot->getDof("R_SHOULDER_P")->getIndexInSkeleton(), (4+5*sin(2*M_PI*0.01*(mWorld->getSimFrames())))*M_PI/180 );
mRobot->setPosition(mRobot->getDof("L_SHOULDER_P")->getIndexInSkeleton(), (4-5*sin(2*M_PI*0.01*(mWorld->getSimFrames())))*M_PI/180  );

}

void Controller::storeData() {
/*
    fout1 << current.getSwingFootPose(walkState.supportFoot).transpose() << std::endl;
    fout2 << current.getComPose().transpose() << std::endl;
    fout3 << desired.getSwingFootPose(walkState.supportFoot).transpose() << std::endl;
    fout4 << desired.getComPose().transpose() << std::endl;
    fout5 << current.comAcc.transpose() << std::endl;
    fout6 << desired.comAcc.transpose() << std::endl;

    // CoM acceleration reconstructed from jacobian derivative
    Eigen::MatrixXd jac = getTorsoAndSwfJacobian();
    Eigen::MatrixXd jacDeriv = getTorsoAndSwfJacobianDeriv();

    Eigen::VectorXd qDot(50);
    Eigen::VectorXd qDoubleDot(50);
    for (int i = 0; i < 50; ++i){
        qDot(i) = mRobot->getVelocity(i+6);
        qDoubleDot(i) = mRobot->getAcceleration(i+6);
    }

    Eigen::VectorXd state = jac * qDoubleDot + jacDeriv * qDot;
    fout7 << state.transpose() << std::endl;

    fout8 << getZmpFromExternalForces().transpose() << std::endl;
    fout9 << desired.zmpPos.transpose() << std::endl;
/**/



        Eigen::VectorXd COMPOS_meas  = Eigen::VectorXd::Zero(3);
        COMPOS_meas = mRobot->getCOM();
        Eigen::VectorXd COMVEL_meas  = Eigen::VectorXd::Zero(3);
        COMVEL_meas = mTorso->getCOMLinearVelocity();
        Eigen::VectorXd FOOT_meas  = Eigen::VectorXd::Zero(3);
        FOOT_meas = mSupportFoot->getCOM();

        Eigen::VectorXd ZMPPOS_meas_cop =  Eigen::VectorXd::Zero(3);
        ZMPPOS_meas_cop = COMPOS_meas - mRobot->getCOMLinearAcceleration()/(omega*omega);
        ZMPPOS_meas_cop = getZmpFromExternalForces();
        //std::cout << " qui " << ZMPPOS_meas_cop << std::endl;

        ofstream myfile;

        myfile.open ("./Data/x_RF.txt",ios::app);
        myfile << mSupportFoot->getCOM()(0) <<endl; 
        myfile.close();
        myfile.open ("./Data/y_RF.txt",ios::app);
        myfile << mSupportFoot->getCOM()(1) <<endl; 
        myfile.close();

        myfile.open ("./Data/x_m.txt",ios::app);
        myfile << COMPOS_meas(0) <<endl; 
        myfile.close();
        myfile.open ("./Data/y_m.txt",ios::app);
        myfile << COMPOS_meas(1) <<endl; 
        myfile.close();
        myfile.open ("./Data/xz_m_cop.txt",ios::app);
        myfile << ZMPPOS_meas_cop(0) <<endl; //current.zmpPos
        myfile.close();
        myfile.open ("./Data/yz_m_cop.txt",ios::app);
        myfile << ZMPPOS_meas_cop(1) <<endl; 
        myfile.close();

        myfile.open ("./Data/x.txt",ios::app);
        myfile << desired.comPos(0) <<endl; 
        myfile.close();
        myfile.open ("./Data/y.txt",ios::app);
        myfile << desired.comPos(1) <<endl; 
        myfile.close();
        myfile.open ("./Data/xz.txt",ios::app);
        myfile << desired.zmpPos(0) <<endl; 
        myfile.close();
        myfile.open ("./Data/yz.txt",ios::app);
        myfile << desired.zmpPos(1) <<endl; 
        myfile.close();
/**/
}
