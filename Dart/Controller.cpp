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
    // isDoublePendulum = true;
    isDoublePendulum = VIP_global;

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
    float mass = 50.055f;
    mRobotMass = 50.055;
    mTorsoMass =  mTorso->getMass();

    for (int i = 0; i < 5000; i++) {
        if (i < 100) vrefSequence.push_back(Vref(0.0, 0.0, 0.00));
        else vrefSequence.push_back(Vref(0.00, 0.0, 0.00));
    }


    bool firstSupportFootIsLeft = false;
    walkState.supportFoot = firstSupportFootIsLeft;
    Eigen::VectorXd leftFootPose(6), rightFootPose(6);
    leftFootPose << getRPY(mLeftFoot), mLeftFoot->getCOM();
    rightFootPose << getRPY(mRightFoot), mRightFoot->getCOM();

    footstepPlan = new FootstepPlan;
    footstepPlan->plan(vrefSequence, leftFootPose, rightFootPose, firstSupportFootIsLeft);

    Eigen::Vector3d maxTheta = Eigen::Vector3d::Zero(3);
    maxTheta << M_PI, M_PI, M_PI;
    // maxTheta << M_PI/18.0, M_PI/18.0, 0.0;
// std::cout << "M_PI =" << M_PI << std::endl;
    // maxTheta << 0.2, 0.2, 0.2;
    
    Eigen::Matrix3d momentOfInertia;
    momentOfInertia << 0.344345670122806, -5.676994253777424e-04, 0.045048560212699, 
    -5.676994253777424e-04, 0.338324801980916, 0.003172978374495,
     0.045048560212699, 0.003172978374495, 0.048160214086886;

    //       momentOfInertia << 0.344345670122806, 0.0, 0.0, 
    // 0.0, 0.338324801980916, 0.0,
    //  0.0, 0.0, 0.048160214086886;


    // Instantiate MPC solver
    if(isDoublePendulum)
        solver_cam = new MPCSolvercam(footstepPlan, 1, angleConstraint, mTorsoMass, momentOfInertia, maxTheta);
    solver = new MPCSolver(footstepPlan,1,false);
// }
    current.comPos << 0.0,0.0,0.0;
    current_cam.comPos << 0.0,0.0,0.0;
    current.torsoOrient << 0.0,0.0,0.0;
    current_cam.torsoOrient << 0.0,0.0,0.0;

    //Desired values
    desired.comPos = Eigen::Vector3d(current.comPos(0), current.comPos(1), comTargetHeight);
    desired.comVel = Eigen::Vector3d::Zero();
    desired.zmpPos = Eigen::Vector3d(current.comPos(0), current.comPos(1),0.0);
    desired.leftFootPos = Eigen::Vector3d(0.0, FootStepY, 0.0);
    desired.rightFootPos = Eigen::Vector3d(0.0, -FootStepY, 0.0);
    desired.torsoOrient = Eigen::Vector3d(0.0, 0.0, 0.0);
    desired.leftFootOrient = Eigen::Vector3d(0.0, 0.0, 0.0);
    desired.rightFootOrient = Eigen::Vector3d(0.0, 0.0, 0.0);
    desired.comAcc = omega*omega * (desired.comPos - desired.zmpPos);

    desiredWithNoise = desired;
    previous_desired = desired;

    //Desired cam values
    desired_cam.comPos = Eigen::Vector3d(current_cam.comPos(0), current_cam.comPos(1), comTargetHeight);
    desired_cam.comVel = Eigen::Vector3d::Zero();
    desired_cam.zmpPos = Eigen::Vector3d(0.0, 0.0,0.0);
    desired_cam.leftFootPos = Eigen::Vector3d(0.0, FootStepY, 0.0);
    desired_cam.rightFootPos = Eigen::Vector3d(0.0, -FootStepY, 0.0);
    desired_cam.torsoOrient = Eigen::Vector3d(0.0, 0.0, 0.0);
    desired_cam.leftFootOrient = Eigen::Vector3d(0.0, 0.0, 0.0);
    desired_cam.rightFootOrient = Eigen::Vector3d(0.0, 0.0, 0.0);
    desired_cam.comAcc = omega*omega * (desired_cam.comPos - desired_cam.zmpPos);

    // std::cout << desired.comPos << " " << desired.comVel << " " << desired.zmpPos << " " << desired.comAcc << std::endl;
    // std::cout << desired_cam.comPos << " " << desired_cam.comVel << " " << desired_cam.zmpPos << " " << desired_cam.comAcc << std::endl;

    // Instantiate new Filter (new is for memory allocation and referencing!)
    Eigen::Matrix2f input_noise_cov; input_noise_cov << 1000.0f, 0.0f, 0.0f, 1000.0f;
    Eigen::Matrix3f meas_noise_cov; meas_noise_cov << 0.01f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f;
    Eigen::Vector3f in_st_x; in_st_x << (float) current.comPos(0), 0.0f, 0.0f;
    Eigen::Vector3f in_st_y; in_st_y << (float) current.comPos(1), 0.0f, 0.0f;
    Eigen::Vector3f in_st_z; in_st_z << (float) comTargetHeight + 0.03f, 0.0f, 0.0f;
    float h = (float) comTargetHeight + 0.03f; //needed correction
    float delta_t = 0.01f;

    Eigen::Matrix2f input_noise_cov_z = 1.0f*input_noise_cov;
    Eigen::Matrix3f meas_noise_cov_z; meas_noise_cov_z << 0.01f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.01f;

    // Filter = new StateFiltering(in_st_x, input_noise_cov, meas_noise_cov,
    //                             in_st_y, input_noise_cov, meas_noise_cov,
    //                             in_st_z, input_noise_cov_z, meas_noise_cov_z,
    //                             h, mass, delta_t);

    // std::cout << "mass " << mRobot->getMass() << std::endl;
    mAngularVelocity = Eigen::Vector3d::Zero(3);
    mAngularPosition = current_cam.torsoOrient;
    prevQdot = Eigen::VectorXd::Zero(50);

    xcom_tot = 0.0;
    ycom_tot = 0.0;
    xdcom_tot = 0.0;
    ydcom_tot = 0.0;
    xzcom_tot = 0.0;
    yzcom_tot = 0.0;

    xzd = 0.0;
    yzd = 0.0;

    virt_torq = Eigen::Vector2d::Zero();
    virt_torq_zeros_allthetime = Eigen::Vector2d::Zero();
    // desiredTorques = Eigen::Vector3d::Zero();
    // TorquesDesired = Eigen::Vector3d::Zero(3);

    error_torsoAngle_foot = Eigen::Vector3d::Zero();
    error_torsoPos_foot = Eigen::Vector3d::Zero();
    error_footAngle_foot = Eigen::Vector3d::Zero();
    error_footPos_foot = Eigen::Vector3d::Zero();

    ofstream myfile;

        myfile.open("../txts/x_m_supp.txt", ios::trunc);
        myfile.close();
        myfile.open ("../txts/y_m_supp.txt", ios::trunc);
        myfile.close();
        myfile.open("../txts/x_m_sw.txt", ios::trunc);
        myfile.close();
        myfile.open ("../txts/y_m_sw.txt", ios::trunc);
        myfile.close();

        myfile.open("../txts/x_supp.txt", ios::trunc);
        myfile.close();
        myfile.open ("../txts/y_supp.txt", ios::trunc);
        myfile.close();
        myfile.open("../txts/x_sw.txt", ios::trunc);
        myfile.close();
        myfile.open ("../txts/y_sw.txt", ios::trunc);
        myfile.close();

        myfile.open ("../txts/x_m.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/y_m.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/xz_m_cop.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/yz_m_cop.txt",ios::trunc);
        myfile.close();

        myfile.open ("../txts/x.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/y.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/xz.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/yz.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/xd.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/yd.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/xd_m.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/yd_m.txt",ios::trunc);
        myfile.close();

    
        myfile.open ("../txts/xcam.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/ycam.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/xzcam.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/yzcam.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/xdcam.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/ydcam.txt",ios::trunc);
        myfile.close();

        myfile.open ("../txts/virt_torqx.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/virt_torqy.txt",ios::trunc);
        myfile.close();

        myfile.open ("../txts/desiredTorsoVelocity_x.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/desiredTorsoVelocity_y.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/desiredTorsoVelocity_z.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/desiredTorsoPosition_x.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/desiredTorsoPosition_y.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/desiredTorsoPosition_z.txt",ios::trunc);
        myfile.close();

        myfile.open ("../txts/xwithnoise.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/ywithnoise.txt",ios::trunc);
        myfile.close();

        myfile.open ("../txts/xzwithnoise.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/yzwithnoise.txt",ios::trunc);
        myfile.close();

        myfile.open ("../txts/xzd.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/yzd.txt",ios::trunc);
        myfile.close();

        myfile.open ("../txts/error_x.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/error_y.txt",ios::trunc);
        myfile.close();

        myfile.open ("../txts/errorfoot_x.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/errorfoot_y.txt",ios::trunc);
        myfile.close();

        myfile.open ("../txts/errorfoot_thetax.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/errorfoot_thetay.txt",ios::trunc);
        myfile.close();


        myfile.open ("../txts/actualOrientation_x.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/actualOrientation_y.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/actualOrientation_z.txt",ios::trunc);
        myfile.close();

        myfile.open ("../txts/actualAngVel_x.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/actualAngVel_y.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/actualAngVel_z.txt",ios::trunc);
        myfile.close();

        myfile.open ("../txts/desiredTorsoPosition_foot_x.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/desiredTorsoPosition_foot_y.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/desiredTorsoPosition_foot_z.txt",ios::trunc);
        myfile.close();

        myfile.open ("../txts/actualOrientation_foot_x.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/actualOrientation_foot_y.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/actualOrientation_foot_z.txt",ios::trunc);
        myfile.close();

        myfile.open ("../txts/torquex_desired.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/torquey_desired.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/torquez_desired.txt",ios::trunc);
        myfile.close();


        myfile.open ("../txts/error_torsoAngle_foot_x.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/error_torsoAngle_foot_y.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/error_torsoAngle_foot_z.txt",ios::trunc);
        myfile.close();

        myfile.open ("../txts/error_torsoPos_foot_x.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/error_torsoPos_foot_y.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/error_torsoPos_foot_z.txt",ios::trunc);
        myfile.close();

        myfile.open ("../txts/error_footAngle_foot_x.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/error_footAngle_foot_y.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/error_footAngle_foot_z.txt",ios::trunc);
        myfile.close();

        myfile.open ("../txts/error_footPos_foot_x.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/error_footPos_foot_y.txt",ios::trunc);
        myfile.close();
        myfile.open ("../txts/error_footPos_foot_z.txt",ios::trunc);
        myfile.close();


}

Controller::~Controller() {}



void Controller::update() {
        // std::cout << "walkState.mpcIter = "<< walkState.mpcIter << '\n';        
        // std::cout << "walkState.controlIter = "<< walkState.controlIter << '\n';
        // std::cout << "VIPsturn = " << VIPsturn << std::endl;
    // STOP EXECUTION AFTER 6 SECONDS
    // if (mWorld->getSimFrames() == 1000) exit(1);
    if (mWorld->getSimFrames() == 4000) exit(1);

    // if(mWorld->getSimFrames() == 20){
    //     for(int i = 0; i<mRobot->getNumDofs(); i++)
    //     {
    //         std::cout << "i = "<< i << ", Bodynode is = " << mRobot->getDof(i)->getName() << std::endl;
    //     }
    // }

    // if (mWorld->getSimFrames()==100)
        // auto visualShapeNodes = mTorso->getShapeNodesWith<VisualAspect>();
    // if(visualShapeNodes.size() == 3u )
    // {
    // //assert(visualShapeNodes[2]->getShape() == mArrow);
    //     visualShapeNodes[2]->remove();
    // }
		
    // if(visualShapeNodes.size() == 4u )
    // {
    // //assert(visualShapeNodes[2]->getShape() == mArrow);
    //     visualShapeNodes[2]->remove();
    //     visualShapeNodes[3]->remove();
    // }

        // std::cout << " mTorso->getCOM()->getPosition()/2 = " <<  mTorso->getCOM()->getPosition()/2 << std::endl;
    // if (mWorld->getSimFrames()==100)
    //     std::shared_ptr<ArrowShape> mArrow;
    // if (mWorld->getSimFrames()==200)
    //  {   
    //         mTorso->addExtForce(Eigen::Vector3d(0,200,0));        
    // ArrowShape::Properties arrow_properties;
    // arrow_properties.mRadius = 0.05;
    // Eigen::Vector3d tail_pos = Eigen::Vector3d(0.0, -0.2, comTargetHeight/2) ;
    // Eigen::Vector3d tail_offset = Eigen::Vector3d(0.0, 0.65, 0.0);
    // Eigen::Vector3d head_pos = tail_pos - tail_offset;
    // mArrow = std::shared_ptr<ArrowShape>(new ArrowShape(
    //          Eigen::Vector3d(0.0, 0.0, 0.0),
    //          Eigen::Vector3d(0.2, 0.05, 0.05),
    //          arrow_properties, dart::Color::Red(1.0)));
    // mArrow->setPositions(
    //         head_pos,
    //         tail_pos);
    // mTorso->createShapeNodeWith<VisualAspect>(mArrow);
    // }
    // if (mWorld->getSimFrames()==205)
    //     {
    //         // std::cout << "mTorso->getNumVisualizationShapes() = " << mTorso->getNumVisualizationShapes() << std::endl;
    //         visualShapeNodes[1]->remove();
    //         // mTorso->removeVisualizationShape(mArrow);
    //     }


    //if (mWorld->getSimFrames()==500) system("gnuplot ../plotters/plot");
    walkState.simulationTime = mWorld->getSimFrames();


    // std::cout << "walkState.simulationTime = " << walkState.simulationTime << std::endl;

    // Retrieve current position and orientation of com and feet in the world frame
    // if(!VIPsturn || !isDoublePendulum){
    // current.comPos = balancePoint == TORSO ? mBase->getCOM() : mRobot->getCOM();
    // current.comVel = balancePoint == TORSO ? mBase->getCOMLinearVelocity() : mRobot->getCOMLinearVelocity();
    // current.comAcc = balancePoint == TORSO ? mBase->getCOMLinearAcceleration() : mRobot->getCOMLinearAcceleration();

    current.comPos = balancePoint == TORSO ? mBase->getCOM() : mRobot->getCOM();
    current.comVel = balancePoint == TORSO ? mBase->getCOMLinearVelocity() : mRobot->getCOMLinearVelocity();
    current.comAcc = balancePoint == TORSO ? mBase->getCOMLinearAcceleration() : mRobot->getCOMLinearAcceleration();
// }

    // current.comPos = mRobot->getCOM();
    // current.comVel = mRobot->getCOMLinearVelocity();
    // current.comAcc = mRobot->getCOMLinearAcceleration();
/**/
    // if(!VIPsturn || !isDoublePendulum){
    current.zmpPos = current.comPos - current.comAcc / (omega*omega);
    current.leftFootPos = mLeftFoot->getCOM();
    current.rightFootPos = mRightFoot->getCOM();
    current.torsoOrient = getRPY(mBase);
    current.leftFootOrient = getRPY(mLeftFoot);
    current.rightFootOrient = getRPY(mRightFoot);

// Eigen::Isometry3d centerTf(Eigen::Isometry3d::Identity());
// centerTf.translation() = mTorso->getCOM();
// centerTf.linear() = mTorso->getTransform().linear();
// SimpleFrame center(Frame::World(), "center", centerTf);

    // std::cout<<"angulatvel is = " <<mTorso->getCOMAngularVelocity() << std::endl;
    // std:cout << mTorso->getAngularVelocity(mWorld);

// }


// exit(1);
    //desired.leftFootPos =  current.leftFootPos;
    //desired.rightFootPos =  current.rightFootPos;

    // Compute the new desired state using MPC
    // if(!VIPsturn || !isDoublePendulum){
        // if(walkState.simulationTime < 500)
    // desired.comPos = desired.comPos - desired_cam.comPos;

            // desired.comPos(0) = xcom_tot-desired_cam.comPos(0);
            // desired.comPos(1) = ycom_tot-desired_cam.comPos(1);
            
            // desired.comVel(0) = xdcom_tot - desired_cam.comVel(0);
            // desired.comVel(1) = ydcom_tot - desired_cam.comVel(1);

            // desired.comAcc(0) = xacom_tot - desired_cam.comAcc(0);
            // desired.comAcc(1) = yacom_tot - desired_cam.comAcc(1);

            // desired.zmpPos = desired.comPos - desired.comAcc / (omega*omega);

            // if ((solver->ct) % 10 == 0 && walkState.footstepCounter > 1 && (solver->ct) >= 0 ) {
            // desired.comPos(0) += 0.1* (mRobot->getCOM()(0) - desired.comPos(0) - desired_cam.comPos(0));
            // desired.comPos(1) += 0.1* (mRobot->getCOM()(1) - desired.comPos(1) - desired_cam.comPos(1));

            // desired.comVel(0) += 0.1* (mRobot->getCOMLinearVelocity()(0) - desired.comVel(0) - desired_cam.comVel(0));
            // desired.comVel(1) += 0.1* (mRobot->getCOMLinearVelocity()(1) - desired.comVel(1) - desired_cam.comVel(1));

            // std::cout << "desired.comPos(0) =" << (mRobot->getCOM()(0) - desired.comPos(0) - desired_cam.comPos(0)) << std::endl;
            // std::cout << "desired.comPos(1) =" << (mRobot->getCOM()(1) - desired.comPos(1) - desired_cam.comPos(1)) << std::endl;

            // std::cout << "desired.comVel(0) =" << (mRobot->getCOMLinearVelocity()(0) - desired.comVel(0) - desired_cam.comVel(0)) << std::endl;
            // std::cout << "desired.comVel(1) =" << (mRobot->getCOMLinearVelocity()(1) - desired.comVel(1) - desired_cam.comVel(1)) << std::endl;

            // std::cout << "solver->ct = " << solver->ct;
// }
            // desired_cam.comPos(0) = (mRobot->getCOM()(0) - desired.comPos(0));
            // desired_cam.comPos(1) = (mRobot->getCOM()(1) - desired.comPos(1));

            // desired.comPos(0) += (mRobot->getCOM()(0) - desired_cam.comPos(0));
            // desired.comPos(1) += (mRobot->getCOM()(1) - desired_cam.comPos(1));
            
            // desired.comVel(0) += (mRobot->getCOMLinearVelocity()(0) - desired_cam.comVel(0));
            // desired.comVel(1) += (mRobot->getCOMLinearVelocity()(1) - desired_cam.comVel(1));

            // desired.comAcc(0) += (mRobot->getCOMLinearAcceleration()(0) - desired_cam.comAcc(0));
            // desired.comAcc(1) += (mRobot->getCOMLinearAcceleration()(1) - desired_cam.comAcc(1));

            // desired.zmpPos = desired.comPos - desired.comAcc / (omega*omega);

    // if(walkState.simulationTime+1 <= 49)
    //     virt_torq << 0.0, -0.0005*sin((walkState.simulationTime+1)*0.01*2*M_PI-4*M_PI/5);
    // else   
    //     virt_torq << -0.0005*sin((walkState.simulationTime+1)*0.01*4*M_PI-M_PI), -0.0005*sin((walkState.simulationTime+1)*0.01*2*M_PI-4*M_PI/5);
    // desiredWithNoise = solver->solve(desired, desired_cam, walkState, 0.0, 0.0, 0.0, virt_torq, 0.0);
            // std::cout << "before MPCSolver" << std::endl;
    previous_desired = desired;
    desired = solver->solve(desired, desired_cam, walkState, 0.0, 0.0, 0.0, virt_torq, 0.0);

        error_x = (mBase->getCOM()(0) - previous_desired.comPos(0));
        error_y = (mBase->getCOM()(1) - previous_desired.comPos(1));

        error_xd = (mBase->getCOMLinearVelocity()(0) - previous_desired.comVel(0));
        error_yd = (mBase->getCOMLinearVelocity()(1) - previous_desired.comVel(1));

        error_xa = (mBase->getCOMLinearAcceleration()(0) - previous_desired.comAcc(0));
        error_ya = (mBase->getCOMLinearAcceleration()(1) - previous_desired.comAcc(1));

        errorfoot_x = (current.getRelComPose(walkState.supportFoot)(3) - previous_desired.getRelComPose(walkState.supportFoot)(3));
        errorfoot_y = (current.getRelComPose(walkState.supportFoot)(4) - previous_desired.getRelComPose(walkState.supportFoot)(4));

        errorfoot_thetax = (current.getRelComPose(walkState.supportFoot)(0) - previous_desired.getRelComPose(walkState.supportFoot)(0));
        errorfoot_thetay = (current.getRelComPose(walkState.supportFoot)(1) - previous_desired.getRelComPose(walkState.supportFoot)(1));



        // errorfoot_x = (current.getRelComPose(walkState.supportFoot)(0) - desired.getRelComPose(walkState.supportFoot)(0));
        // errorfoot_y = (current.getRelComPose(walkState.supportFoot)(1) - desired.getRelComPose(walkState.supportFoot)(1));

        // error_x = (mBase->getCOM()(0) - desired.comPos(0));
        // error_y = (mBase->getCOM()(1) - desired.comPos(1));

        // std::cout << "CHANGE ME: ERROR IN Update()" << endl;


    // xzd = solver->xz_dot;
    // yzd = solver->yz_dot;
    // if(isDoublePendulum && isVirtualNoise) desiredWithNoise = solver->nextwithdisturbance;
    // else desiredWithNoise = desired;

// else     desired = solver->solve(current, desired_cam, walkState, 0.0, 0.0, 0.0, 0.0, 0.0);

         // we start from desired because mpc is open loop right now
// }
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
        // if(!VIPsturn || !isDoublePendulum){
        // qDot =  getJointVelocitiesStacked(desired.getRelComPose(walkState.supportFoot),  current.getRelComPose(walkState.supportFoot),
        //  desired.getRelSwingFootPose(walkState.supportFoot),  current.getRelSwingFootPose(walkState.supportFoot), desired.comVel);
 // /**/
 //    // Set the velocity of the floating base to zero
 //    for (int i = 0; i < 6; ++i){
 //        mRobot->setCommand(i, 0);
 //    }

 //    // Set the velocity of each joint as per inverse kinematics
 //    for (int i = 0; i < 50; ++i){
 //        // mRobot->setCommand(i+6,mRobot->getCommand(i+6)+qDot(i)); //velocity joint control
 //        mRobot->setCommand(i+6,qDot(i)); //velocity joint control
 //    }

// }
    // prevQdot = qDot;

    // if(VIPsturn && isDoublePendulum){
    // current_cam.comPos = mRobot->getCOM()-desired.comPos;
    // current_cam.comVel = mRobot->getCOMLinearVelocity()-desired.comVel;
    // current_cam.comAcc = mRobot->getCOMLinearAcceleration()-desired.comAcc;

    // current_cam.comPos = mRobot->getCOM();
    // current_cam.comVel = mRobot->getCOMLinearVelocity();
    // current_cam.comAcc = mRobot->getCOMLinearAcceleration();


    // current_cam.zmpPos = current_cam.comPos - current_cam.comAcc / (omega*omega);
    // current_cam.leftFootPos = mLeftFoot->getCOM();
    // current_cam.rightFootPos = mRightFoot->getCOM();
    // current_cam.torsoOrient = getRPY(mBase);
    // current_cam.leftFootOrient = getRPY(mLeftFoot);
    // current_cam.rightFootOrient = getRPY(mRightFoot);
// }
        // std::cout << "1 "<<std::endl;
    //Eigen::VectorXd qDot =  getJointVelocitiesQp(current, desired);
    Eigen::Vector3d desiredTorques = Eigen::Vector3d::Zero();
    // desiredTorques = Eigen::Vector3d::Zero();
        Eigen::Matrix3d MOI = getMomentOfInertia();
        Eigen::Vector3d angularAcceleration = Eigen::Vector3d::Zero();
        // std::cout << "2"<<std::endl;
        // if(isDoublePendulum && VIPsturn){
        // if(isDoublePendulum && walkState.footstepCounter > 0){
        if(isDoublePendulum){
// std::cout << "torso mass = " << mTorsoMass << endl;
            // if ((solver_cam->ct) % 10 == 0 && walkState.footstepCounter > 1 && (solver_cam->ct) >= 0 ) {

        //         desired_cam.comPos(0) += 0.3* (mRobot->getCOM()(0) - desired.comPos(0) - desired_cam.comPos(0));
        //         desired_cam.comPos(1) += 0.3* (mRobot->getCOM()(1) - desired.comPos(1) - desired_cam.comPos(1));

        //         desired_cam.comVel(0) += 0.3* (mRobot->getCOMLinearVelocity()(0) - desired.comVel(0) - desired_cam.comVel(0));
        //         desired_cam.comVel(1) += 0.3* (mRobot->getCOMLinearVelocity()(1) - desired.comVel(1) - desired_cam.comVel(1));

        //         // std::cout << "solver_cam->ct = " << solver_cam->ct;
        // }
        // std::cout << "mRobot->getCOM()(0) - desired.comPos(0) = " << mRobot->getCOM()(0) - desired.comPos(0) << std::endl;
            // desired_cam.comAcc(0) += 0.1* (mRobot->getCOMLinearAcceleration()(0) - desired.comPos(0));
            // desired_cam.comAcc(1) += 0.1* (mRobot->getCOMLinearAcceleration()(1) - desired.comPos(1));

            // desired_cam.zmpPos = desired_cam.comPos - desired_cam.comAcc / (omega*omega);
            
            // desired_cam.comPos = xcom_tot-desired.comPos;

            // desired_cam.comPos(0) = xcom_tot-desired.comPos(0);
            // desired_cam.comPos(1) = ycom_tot-desired.comPos(1);
            
            // desired_cam.comVel(0) = xdcom_tot - desired.comVel(0);
            // desired_cam.comVel(1) = ydcom_tot - desired.comVel(1);

            // desired_cam.comAcc(0) = xacom_tot - desired.comAcc(0);
            // desired_cam.comAcc(1) = yacom_tot - desired.comAcc(1);



        // desired_cam.comPos = desired.comPos-desired_cam.comPos;
        // desired_cam.comVel = mRobot->getCOMLinearVelocity()-desired.comVel;
        // desired_cam.comAcc = mRobot->getCOMLinearAcceleration()-desired.comAcc;
        // current_cam.comVel = desired_cam.comVel;
        // current_cam.comAcc = desired_cam.comAcc;
        // std::cout << "hold for cam" << std::endl;
        // std::cout << "desired_cam.zmpPos=" << desired_cam.zmpPos << std::endl;

    //     if ((solver->ct) % 10 == 0 && walkState.footstepCounter > 1 && (solver->ct) >= 0 ) {

    //     Eigen::Vector3d pos = mTorso->getCOM();
    //     Eigen::Vector3d vel = mTorso->getCOMLinearVelocity();

        // desired_cam.comPos(0) += 0.125 * (current.comPos(0) - desired_cam.comPos(0));
        // desired_cam.comVel(0) += 0.125 * (current.comVel(0) - desired_cam.comVel(0));
        // desired_cam.comPos(1) += 0.125 * (current.comPos(1) - desired_cam.comPos(1));
        // desired_cam.comVel(1) += 0.125 * (current.comVel(1) - desired_cam.comVel(1));

    //     // desired.leftFootPos += 0*0.02 * (current.leftFootPos - desired.leftFootPos);
    //     // desired.rightFootPos += 0*0.02 * (current.rightFootPos - desired.rightFootPos);
    // // }
            // desired_cam.comPos(0) = mRobot->getCOM()(0) - desired.comPos(0);
            // desired_cam.comPos(1) = mRobot->getCOM()(1) - desired.comPos(1);

            // desired_cam.comPos(0) = 0.1* (mRobot->getCOM()(0) - desired.comPos(0));
            // desired_cam.comPos(1) = 0.1* (mRobot->getCOM()(1) - desired.comPos(1));
            
            // desired_cam.comVel(0) = 0.1* (mRobot->getCOMLinearVelocity()(0) - desired.comVel(0));
            // desired_cam.comVel(1) = 0.1* (mRobot->getCOMLinearVelocity()(1) - desired.comVel(1));

            // desired_cam.comAcc(0) = 0.1* (mRobot->getCOMLinearAcceleration()(0) - desired.comAcc(0));
            // desired_cam.comAcc(1) = 0.1* (mRobot->getCOMLinearAcceleration()(1) - desired.comAcc(1));

            // desired_cam.zmpPos = desired_cam.comPos - desired_cam.comAcc / (omega*omega);


        // desired_cam.comPos(0) += 0.01*error_x;
        // desired_cam.comPos(1) += 0.01*error_y;

        // desired_cam.comVel(0) += 0.01*error_xd;
        // desired_cam.comVel(1) += 0.01*error_yd;

        // desired_cam.comAcc(0) += 0.01*error_xa;
        // desired_cam.comAcc(1) += 0.01*error_ya;


        // desired_cam.zmpPos = desired_cam.comPos - desired_cam.comAcc / (omega*omega);

            // desired_cam.zmpPos = desired_cam.comPos - desired_cam.comAcc / (omega*omega);

            //Matlab

        // desired_cam.comPos(0) += desiredWithNoise.comPos(0) - desired.comPos(0);
        // desired_cam.comPos(1) += desiredWithNoise.comPos(1) - desired.comPos(1);

        // desired_cam.comVel(0) += desiredWithNoise.comVel(0) - desired.comVel(0);
        // desired_cam.comVel(1) += desiredWithNoise.comVel(1) - desired.comVel(1);

        // desired_cam.zmpPos(0) += desiredWithNoise.zmpPos(0) - desired.zmpPos(0);
        // desired_cam.zmpPos(1) += desiredWithNoise.zmpPos(1) - desired.zmpPos(1);
// std::cout << "desired_cam = " << desired_cam << std::endl;


        // desired_cam.comPos(0) = 0.2*(mBase->getCOM()(0) - previous_desired.comPos(0));
        // desired_cam.comPos(1) = 0.2*(mBase->getCOM()(1) - previous_desired.comPos(1));

// std::cout << "desired_cam.comPos = " << desired_cam.comPos << std::endl;
// std::cout << "desired_cam.comPos = " << desired_cam.comPos << std::endl;

        //Relevant Gain
        desired_cam.comPos(0) = 0.1*errorfoot_x;
        desired_cam.comPos(1) = 0.1*errorfoot_y;

        // desired_cam.comPos(0) = 0.1*errorfoot_x - 0.01*errorfoot_thetay;
        // desired_cam.comPos(1) = 0.1*errorfoot_y - 0.01*errorfoot_thetax;

        // desired_cam.comPos(0) = 1.0*(current.getRelComPose(walkState.supportFoot)(0) - previous_desired.getRelComPose(walkState.supportFoot)(0));
        // desired_cam.comPos(1) = 1.0*(current.getRelComPose(walkState.supportFoot)(1) - previous_desired.getRelComPose(walkState.supportFoot)(1));

        // std::cout << "before MPCSolvercam" << std::endl;
        desired_cam = solver_cam->solve(desired, desired_cam, walkState, mTorsoMass, mRobot, mBase->getCOM()(0), mBase->getCOM()(1), mAngularPosition, mAngularVelocity); // we start from desired because mpc is open loop right now;
                // std::cout << "AFTER MPCSolvercam" << std::endl;
        // xcom_tot = desiredWithNoise.comPos(0) + desired_cam.comPos(0);
        // ycom_tot = desiredWithNoise.comPos(1) + desired_cam.comPos(1);        

        // xdcom_tot = desiredWithNoise.comVel(0) + desired_cam.comVel(0);
        // ydcom_tot = desiredWithNoise.comVel(1) + desired_cam.comVel(1);        

        // xzcom_tot = desiredWithNoise.zmpPos(0) + desired_cam.zmpPos(0);
        // yzcom_tot = desiredWithNoise.zmpPos(1) + desired_cam.zmpPos(1);        


        // current_cam.comPos = desired_cam.comPos;
        desiredTorques = computeTorques();
        angularAcceleration = MOI.inverse()*desiredTorques;
        mAngularPosition += mAngularVelocity*mpcTimeStep + angularAcceleration*0.5*pow(mpcTimeStep,2);
        mAngularVelocity += angularAcceleration*mpcTimeStep;

        // std::cout << "theta,thetad,thetadd = " << mAngularPosition(1) << mAngularVelocity(1) << angularAcceleration(1) << std::endl;

        // mAngularVelocity(2) = 0.0;
        // mAngularPosition(2) = 0.0;
        // mAngularVelocity = walkState.supportFoot == 0 ?  Eigen::Vector3d(0.4, 0.0, 0.0) : Eigen::Vector3d(-0.4, 0.0, 0.0);
        // mAngularPosition += mAngularVelocity*mpcTimeStep;

        desired_cam.torsoOrient = mAngularPosition;
        desired.torsoOrient = desired_cam.torsoOrient;
        // desired.torsoOrient(2) = 0.0;
        // mAngularVelocity(2) = 0.0;
        // std::cout << "desired_cam.comPos=" << desired_cam.comPos << std::endl;
        std::cout << "desiredTorques=" << desiredTorques << std::endl;
        // std::cout << "TorquesDesired=" << TorquesDesired << std::endl;
        std::cout << "desired_cam.torsoOrient=" << desired_cam.torsoOrient << std::endl;

       TorquesDesired = desiredTorques;

        // desired_cam.comVel = mAngularVelocity;
    //     qDot =  getJointVelocitiesStackedcam(desired_cam.getRelComPose(walkState.supportFoot), current_cam.getRelComPose(walkState.supportFoot),
    //      desired.getRelSwingFootPose(walkState.supportFoot), current_cam.getRelSwingFootPose(walkState.supportFoot), mAngularVelocity);
    //     // std::cout<< "qdot = " << qDot << std::endl;

    // // Set the velocity of the floating base to zero
    // for (int i = 0; i < 6; ++i){
    //     mRobot->setCommand(i, 0);
    //     // std::cout << mRobot->getCommand(i) << std::endl;
    // }
    //         // std::cout << mRobot->getCommands();

    // // Set the velocity of each joint as per inverse kinematics
    // for (int i = 0; i < 50; ++i){
    //     // std::cout << mRobot->getCommand(i+6) << std::endl;
    //     mRobot->setCommand(i+6,mRobot->getCommand(i+6)+qDot(i)); //velocity joint control
    //     // std::cout << mRobot->getCommand(i+6) << std::endl;
    // }

                // std::cout << mRobot->getCommands();

}
        // xzcom_tot = desired.zmpPos(0) + desired_cam.zmpPos(0);
        // yzcom_tot = desired.zmpPos(1) + desired_cam.zmpPos(1);
// std::cout << "desired.torsoOrient = " << desired.torsoOrient << std::endl;
        // std::cout << "mAngularPosition" << mAngularPosition << std::endl; 
        // std::cout << "mAngularVelocity" << mAngularVelocity << std::endl;

// std::cout << "ANg vel = " << mAngularVelocity;
// std::cout << "Torso orient = " << desired.torsoOrient;
        // std::cout << "before getJointVelocitiesDoublePendulum" << std::endl;
// std::cout << "REMOVE THE DESIRED ORIENTATION PART PLEASE" << std::endl;
// desired.torsoOrient << 0.0, -M_PI/180, 0.0;

        qDot =  getJointVelocitiesDoublePendulum(desired.getRelComPose(walkState.supportFoot), current.getRelComPose(walkState.supportFoot),
         desired.getRelSwingFootPose(walkState.supportFoot), current.getRelSwingFootPose(walkState.supportFoot), desired.comVel, mAngularVelocity, desired.torsoOrient, current.torsoOrient);


    // Set the velocity of the floating base to zero
    for (int i = 0; i < 6; ++i){
        mRobot->setCommand(i, 0);
    }

    // Set the velocity of each joint as per inverse kinematics
    // std::cout << mWorld->getSimFrames()<<std::endl;
    for (int i = 0; i < 50; ++i){
        mRobot->setCommand(i+6,qDot(i)); //velocity joint control
    // std::cout << "qdot of "<< mRobot->getDof(i+6)->getName() << " = " << qDot(i) << std::endl;
    }

    // Store the results in files (for plotting)
    //storeData();

    // Arm swing
    // ArmSwing();
    // if(!isDoublePendulum || !VIPsturn){
// }
    // std::cout << angularAcceleration << std::endl;

    // Eigen::Vector3d AngularMomentum = ComputeAngularMomentum();

    // Update the iteration counters, if a step is finished reset and change support foot

    // std::cout<< "VIPsturn is " << VIPsturn << ", and controliter is " << walkState.controlIter << std::endl;

    // if(!isDoublePendulum || VIPsturn){
    // if(VIPsturn){
    ++walkState.controlIter;
// }
// if(!isDoublePendulum || !VIPsturn){
    walkState.mpcIter = floor(walkState.controlIter*controlTimeStep/mpcTimeStep);
    if (footstepPlan->getFootstepStartTiming(walkState.footstepCounter) + walkState.mpcIter >= footstepPlan->getFootstepEndTiming(walkState.footstepCounter)) {
    	walkState.controlIter = 0;
    	walkState.mpcIter = 0;
        walkState.footstepCounter++;
	walkState.supportFoot = !walkState.supportFoot;
        // std::cout << "Iteration " << walkState.controlIter << " Footstep " << walkState.footstepCounter << std::endl;
        // std::cout <<  mWorld->getSimFrames() << std::endl;
    }
// }
// }
// std::cout << "controlIter = " << walkState.controlIter << '\n';
// std::cout << "mpcIter = " << walkState.mpcIter << '\n';

// VIPsturn = !VIPsturn;
    // std::cout << "EH YABA"<< endl;
        // xcom_tot = desiredWithNoise.comPos(0) + desired_cam.comPos(0);
        // ycom_tot = desiredWithNoise.comPos(1) + desired_cam.comPos(1);        

        // xdcom_tot = desiredWithNoise.comVel(0) + desired_cam.comVel(0);
        // ydcom_tot = desiredWithNoise.comVel(1) + desired_cam.comVel(1);        

        // xzcom_tot = desiredWithNoise.zmpPos(0) + desired_cam.zmpPos(0);
        // yzcom_tot = desiredWithNoise.zmpPos(1) + desired_cam.zmpPos(1);        
        storeData();

}


//Compute the desired torques from the desired states
Eigen::Vector3d Controller::computeTorques()
{
    Eigen::Vector3d mDesiredTorque = Eigen::Vector3d::Zero(3);
    double totalCOMx, totalCOMy;
    totalCOMx = mBase->getCOM()(0);
    totalCOMy = mBase->getCOM()(1);
    // totalCOMx = desired.comPos(0) + desired_cam.comPos(0);
    // totalCOMy = desired.comPos(1) + desired_cam.comPos(1);

    mDesiredTorque(0) = -desired_cam.zmpPos(1)*mTorsoMass*9.81;
    mDesiredTorque(1) = desired_cam.zmpPos(0)*mTorsoMass*9.81;

    mDesiredTorque(2) = ((totalCOMx-previous_desired.zmpPos(0))* mDesiredTorque(0) 
        + (totalCOMy-previous_desired.zmpPos(1))*mDesiredTorque(1))/-comTargetHeight;
    // mDesiredTorque(2) = 0.0;

    // std::cout <<  ((totalCOMx-desired.zmpPos(0))* mDesiredTorque(0) 
    //     + (totalCOMy-desired.zmpPos(1))*mDesiredTorque(1))/comTargetHeight << std::endl;

    //     double tempp  = ((totalCOMx-desired.zmpPos(0))* mDesiredTorque(0) 
    //     + (totalCOMy-desired.zmpPos(1))*mDesiredTorque(1));
    //     tempp = tempp/comTargetHeight;
    //     std::cout << tempp << std::endl;

    // mDesiredTorque(2) = 0.0;

    // std::cout << "COM acceleration" << desired.comAcc(2) << std::endl;
    // std::cout << "desired.comPos(0) + desired_cam.comPos(0)" << desired.comPos(0) + desired_cam.comPos(0) << std::endl;
    // std::cout << "totalCOMx" << totalCOMx << std::endl;

    // std::cout << "Desired Torque = " << mDesiredTorque << std::endl;
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

     mTorso->getInertia().getMoment();
    // std::cout << "torque inertia = " << mTorso->getInertia().getMoment(); << std::endl;
        MOI << 0.344345670122806, -5.676994253777424e-04, 0.045048560212699, 
        -5.676994253777424e-04, 0.338324801980916, 0.003172978374495,
         0.045048560212699, 0.003172978374495, 0.048160214086886;

    //  MOI << 0.344345670122806, 0.0, 0.0, 
    // 0.0, 0.338324801980916, 0.0,
    //  0.0, 0.0, 0.048160214086886;
    // std::cout << "inertia = " << MOI << std::endl;
     // MOI = (Eigen::MatrixXd::Identity(3,3)) * MOI;
    return MOI;
}

// void Controller::AnkleRegulation() {

// Eigen::Vector3d GYRO = mTorso->getAngularVelocity();
// // GYRO = Eigen::VectorXd::Zero(3);

// double K_pitch = -0.05*(M_PI/180.0);  //0.3*(3.14/180.0)
// double K_roll= -0.05*(M_PI/180.0);    //0.1*(3.14/180.0)

// // right ankle pitch
// mRobot->setPosition(mRobot->getDof("R_ANKLE_P")->getIndexInSkeleton(), mRobot->getPosition(mRobot->getDof("R_ANKLE_P")->getIndexInSkeleton())+K_pitch*(GYRO(1)-mRobot->getPosition(mRobot->getDof("R_ANKLE_P")->getIndexInSkeleton())));
// // right ankle roll
// mRobot->setPosition(mRobot->getDof("R_ANKLE_R")->getIndexInSkeleton(), mRobot->getPosition(mRobot->getDof("R_ANKLE_R")->getIndexInSkeleton())+K_roll*(GYRO(0)-mRobot->getPosition(mRobot->getDof("R_ANKLE_R")->getIndexInSkeleton())));
// // left ankle pitch
// mRobot->setPosition(mRobot->getDof("L_ANKLE_P")->getIndexInSkeleton(), mRobot->getPosition(mRobot->getDof("L_ANKLE_P")->getIndexInSkeleton())+K_pitch*(GYRO(1)-mRobot->getPosition(mRobot->getDof("L_ANKLE_P")->getIndexInSkeleton())));
// // left ankle roll
// mRobot->setPosition(mRobot->getDof("L_ANKLE_R")->getIndexInSkeleton(), mRobot->getPosition(mRobot->getDof("L_ANKLE_R")->getIndexInSkeleton())+K_roll*(GYRO(0)-mRobot->getPosition(mRobot->getDof("L_ANKLE_R")->getIndexInSkeleton())));
// }

void Controller::AnkleRegulation() {

    //Eigen::Vector3d GYRO = mTorso->getAngularVelocity() ;

    Eigen::Vector3d GYRO = getRPY(mSupportFoot) ;  //mSupportFoot

    double K_pitch = -0.01;  //0.3*(3.14/180.0)
    double K_roll= -0.01;    //0.1*(3.14/180.0)

    desired.rightFootOrient(0) = K_roll*(GYRO(0)-mRobot->getPosition(mRobot->getDof("R_ANKLE_R")->getIndexInSkeleton()));
    desired.rightFootOrient(1) =  K_pitch*(GYRO(1)-mRobot->getPosition(mRobot->getDof("R_ANKLE_P")->getIndexInSkeleton()));
    desired.leftFootOrient(0) = K_roll*(GYRO(0)-mRobot->getPosition(mRobot->getDof("L_ANKLE_R")->getIndexInSkeleton()));
    desired.leftFootOrient(1) = K_pitch*(GYRO(1)-mRobot->getPosition(mRobot->getDof("L_ANKLE_P")->getIndexInSkeleton()));

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

    // Eigen::MatrixXd Jacobian_SupportToSwing = mRobot->getJacobian(mSwingFoot,mSupportFoot) - mRobot->getJacobian(mSupportFoot,mSupportFoot);

    Eigen::MatrixXd Jacobian_tot_(6, 56);

    Jacobian_tot_ << Jacobian_supportToBase;

    Eigen::MatrixXd Jacobian_tot(6, 50);

    // Remove the floating base columns
    Jacobian_tot = Jacobian_tot_.block<6,50>(0, 6);

    // std::cout << Jacobian_tot << std::endl;    

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
Eigen::VectorXd Controller::getJointVelocitiesStackedcam(Eigen::VectorXd desidercam_pos_base, Eigen::VectorXd actcamPosBase,
 Eigen::VectorXd desider_pos_SwingFoot, Eigen::VectorXd actPosSwingFoot, Eigen::VectorXd desidercam_com_vel){

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

    //     // Swing Foot Orientation
    // _taskGain(6,6) = 0;
    // _taskGain(7,7) = 0;
    // _taskGain(8,8) = 0;

    // // Swing Foot Position
    // _taskGain(9,9) = 0;
    // _taskGain(10,10) = 0;
    // _taskGain(11,11) = 0;

    double ikGainvel = 1;
    double ikGain = 1;

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
    _taskGain(0,0) = 1;
    _taskGain(1,1) = 1;//0.001;
    _taskGain(2,2) = 1;//0;

    // CoM Position
    _taskGain(3,3) = 10;  //0.1  10
    _taskGain(4,4) = 5;
    _taskGain(5,5) = 1;

    // Swing Foot Orientation
    _taskGain(6,6) = 5;
    _taskGain(7,7) = 5;
    _taskGain(8,8) = 5;

    // Swing Foot Position
    _taskGain(9,9) = 5;
    _taskGain(10,10) = 5;
    _taskGain(11,11) = 1;

    
    double ikGainvel = 1;

    double ikGain = 10;

    Eigen::VectorXd qDot(50);
    qDot = PseudoJacobian_tot*(ComVref+ikGain*_taskGain*(desired_pos - actual_pos));

    return qDot;
}
Eigen::VectorXd Controller::getJointVelocitiesDoublePendulum(Eigen::VectorXd desider_pos_base, Eigen::VectorXd actPosBase,
        Eigen::VectorXd desider_pos_SwingFoot, Eigen::VectorXd actPosSwingFoot,Eigen::VectorXd desider_com_vel, Eigen::VectorXd desidercam_com_vel,
        Eigen::VectorXd desired_world, Eigen::VectorXd current_world){

        Eigen::VectorXd ComVref = Eigen::VectorXd::Zero(12);
        // desidercam_com_vel = Eigen::Vector3d::Zero(3);
     
        ComVref<<desidercam_com_vel(0), desidercam_com_vel(1), desidercam_com_vel(2), desider_com_vel(0), desider_com_vel(1), desider_com_vel(2),0.0,0.0,0.0,0.0,0.0,0.0;


    Eigen::VectorXd desired_pos(12);
    desired_pos << desider_pos_base, desider_pos_SwingFoot;
    // desired_pos << desired_world(0), desired_world(1), desired_world(2), desider_pos_base(3), desider_pos_base(4), desider_pos_base(5), desider_pos_SwingFoot;

        // desired_pos(1) = desired_pos(1) + 0.01;
        // desired_pos(3) = desired_pos(3) ; //0.05  0.02

    // Assemble actual positions and orientations
    Eigen::VectorXd actual_pos(12);
    actual_pos << actPosBase, actPosSwingFoot;
    // actual_pos << current_world(0), current_world(1), current_world(2), actPosBase(3), actPosBase(4), actPosBase(5), actPosSwingFoot;

    // Get the proper jacobian and pseudoinvert it
    Eigen::MatrixXd Jacobian_tot = getTorsoAndSwfJacobian();
    Eigen::MatrixXd PseudoJacobian_tot = (Jacobian_tot.transpose())*(Jacobian_tot*Jacobian_tot.transpose()).inverse();

    Eigen::MatrixXd _taskGain = Eigen::MatrixXd::Identity(12,12);


    //     // VERY GOOD
    // // Torso Orientation
    // _taskGain(0,0) = 1;
    // _taskGain(1,1) = 1;//0.001;
    // _taskGain(2,2) = 0.1;//0;

    // // CoM Position
    // _taskGain(3,3) = 5;  //0.1  10
    // _taskGain(4,4) = 5;
    // _taskGain(5,5) = 1;

    // // Swing Foot Orientation
    // _taskGain(6,6) = 1;
    // _taskGain(7,7) = 1;
    // _taskGain(8,8) = 1;

    // // Swing Foot Position
    // _taskGain(9,9) = 5;
    // _taskGain(10,10) = 5;
    // _taskGain(11,11) = 5;

    // double ikGain = 10;


        // VERY GOOD (LAst working)
    // Torso Orientation
    // _taskGain(0,0) = 0.1;//0.1
    // _taskGain(1,1) = 0.1;//0.1;
    // _taskGain(2,2) = 0.1;//0.1;

    // // CoM Position
    // _taskGain(3,3) = 5;  //0.1  10
    // _taskGain(4,4) = 5;
    // _taskGain(5,5) = 2; //1 IT WAS ONE BEFORE !!!!

    // // Swing Foot Orientation
    // _taskGain(6,6) = 6;
    // _taskGain(7,7) = 6;
    // _taskGain(8,8) = 1;

    // // Swing Foot Position
    // _taskGain(9,9) = 6;
    // _taskGain(10,10) = 6;
    // _taskGain(11,11) = 4;


    //     double ikGain = 10;
    //     ikGain = 9;
    // _taskGain(0,0) = 1;
    // _taskGain(1,1) = 1;//0.001;
    // _taskGain(2,2) = 0.1;//0;

    // // CoM Position
    // _taskGain(3,3) = 5;  //0.1  10
    // _taskGain(4,4) = 5;
    // _taskGain(5,5) = 1;

    // // Swing Foot Orientation
    // _taskGain(6,6) = 1;
    // _taskGain(7,7) = 1;
    // _taskGain(8,8) = 1;

    // // Swing Foot Position
    // _taskGain(9,9) = 5;
    // _taskGain(10,10) = 5;
    // _taskGain(11,11) = 5;


        // double ikGain = 10;

    _taskGain(0,0) = 0.1;//0.1
    _taskGain(1,1) = 0.8;//0.8;
    _taskGain(2,2) = 0.1;//0.1;

    // CoM Position
    _taskGain(3,3) = 5;  //5
    _taskGain(4,4) = 5; //5
    _taskGain(5,5) = 0.5; //0.5 IT WAS ONE BEFORE !!!!

    // Swing Foot Orientation
    _taskGain(6,6) = 2;
    _taskGain(7,7) = 6; //6
    _taskGain(8,8) = 1;

    // Swing Foot Position
    _taskGain(9,9) = 5; //5
    _taskGain(10,10) = 5; //5
    _taskGain(11,11) = 5; //5
    
        double ikGain = 10;
        ikGain = 9;
// std::cout << "ComVref = " << ComVref << std::endl;
// std::cout << "error = " << desired_pos - actual_pos << std::endl;

        Eigen::VectorXd error_foot(12);
        error_foot = desired_pos - actual_pos;
        error_torsoAngle_foot << error_foot(0), error_foot(1), error_foot(2);
        error_torsoPos_foot << error_foot(3), error_foot(4), error_foot(5);
        
        error_footAngle_foot << error_foot(6), error_foot(7), error_foot(8);
        error_footPos_foot << error_foot(9), error_foot(10), error_foot(11);

    Eigen::VectorXd qDot(50);
    qDot = PseudoJacobian_tot*(ComVref+ikGain*_taskGain*(desired_pos - actual_pos));



    return qDot;
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

Eigen::VectorXd Controller::getJointVelocitiesStacked_worldframe(Eigen::VectorXd desider_pos_base, Eigen::VectorXd actPosBase,
        Eigen::VectorXd desider_pos_SwingFoot, Eigen::VectorXd actPosSwingFoot, Eigen::VectorXd desider_com_vel, Eigen::VectorXd desidercam_com_vel){

        Eigen::VectorXd ComVref = Eigen::VectorXd::Zero(12);

        ComVref<<desidercam_com_vel(0), desidercam_com_vel(1), desidercam_com_vel(2), desider_com_vel(0), desider_com_vel(1), desider_com_vel(2),0.0,0.0,0.0,0.0,0.0,0.0;


    Eigen::VectorXd desired_pos(12);
    desired_pos << desider_pos_base, desider_pos_SwingFoot;

        // desired_pos(3) = desired_pos(3) ;
    // Assemble actual positions and orientations
    Eigen::VectorXd actual_pos(12);
    actual_pos << actPosBase, actPosSwingFoot;

    // Get the proper jacobian and pseudoinvert it
    Eigen::MatrixXd Jacobian_tot = getTorsoAndSwfJacobian_worldframe();
    Eigen::MatrixXd PseudoJacobian_tot = (Jacobian_tot.transpose())*(Jacobian_tot*Jacobian_tot.transpose()).inverse();

    Eigen::MatrixXd _taskGain = Eigen::MatrixXd::Identity(12,12);




        //// VERY GOOD
    // Torso Orientation
    _taskGain(0,0) = 0.1;
    _taskGain(1,1) = 0.1;//0.001;
    _taskGain(2,2) = 0.1;//0;

    // CoM Position
    _taskGain(3,3) = 5;  //0.1  10
    _taskGain(4,4) = 5;
    _taskGain(5,5) = 0.5; //1 IT WAS ONE BEFORE !!!!

    // Swing Foot Orientation
    _taskGain(6,6) = 2.0;
    _taskGain(7,7) = 3.5;
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
Eigen::MatrixXd Controller::getTorsoAndSwfJacobian_worldframe() { //FIXME this is still expressed in relative frame
    if (walkState.supportFoot == true) {
        mSupportFoot = mRobot->getBodyNode("r_sole");
        mSwingFoot = mRobot->getBodyNode("l_sole");
    } else {
        mSupportFoot = mRobot->getBodyNode("l_sole");
        mSwingFoot = mRobot->getBodyNode("r_sole");
    }

    Eigen::MatrixXd Jacobian_worldToTorso;


    Jacobian_worldToTorso =  mTorso->getJacobian(); //,mWorld) - mRobot->getJacobian(mWorld,mWorld); //(mBase,mSupportFoot)
 

    // for (unsigned int i=0; i<44; i++){ 
    // if (i!=5 || i!=6)  Jacobian_worldToTorso.col(i).setZero();
    // }
    // std::cout << mTorso->getJacobian() << std::endl;

    std::cout << mSwingFoot->getJacobian() << std::endl;

    std::cout << mSwingFoot->getJacobian().rows() << " ," << mSwingFoot->getJacobian().cols() << std::endl;

    std::cout << mTorso->getJacobian().rows() << " ," << mTorso->getJacobian().cols() << std::endl;

    Eigen::MatrixXd Jacobian_worldToSwing =  mSwingFoot->getJacobian()+mTorso->getJacobian(); //,mWorld) - mRobot->getJacobian(mWorld,mWorld);


    std::cout << Jacobian_worldToSwing << std::endl;

    Eigen::MatrixXd Jacobian_tot_(12, 56);

    Jacobian_tot_ << Jacobian_worldToTorso, Jacobian_worldToSwing;

    Eigen::MatrixXd Jacobian_tot(12, 50);

    // Remove the floating base columns
    Jacobian_tot = Jacobian_tot_.block<12,50>(0, 6);


    return Jacobian_tot;
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

    // q[6] = 25*M_PI/180;

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
    double TimePeriod = (singleSupportDuration + doubleSupportDuration) * 2 * 100;
// if(    walkState.footstepCounter > 0){
mRobot->setPosition(mRobot->getDof("R_SHOULDER_P")->getIndexInSkeleton(), (20*sin(2*M_PI*(1/TimePeriod)*(mWorld->getSimFrames())))*M_PI/180 );
mRobot->setPosition(mRobot->getDof("L_SHOULDER_P")->getIndexInSkeleton(), (-20*sin(2*M_PI*(1/TimePeriod)*(mWorld->getSimFrames())))*M_PI/180  );
// }
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
        COMPOS_meas = mBase->getCOM();
        Eigen::VectorXd COMVEL_meas  = Eigen::VectorXd::Zero(3);
        COMVEL_meas = mBase->getCOMLinearVelocity();
        Eigen::VectorXd FOOT_meas  = Eigen::VectorXd::Zero(3);
        FOOT_meas = mSupportFoot->getCOM();

        Eigen::VectorXd ZMPPOS_meas_cop =  Eigen::VectorXd::Zero(3);
        ZMPPOS_meas_cop = COMPOS_meas - mBase->getCOMLinearAcceleration()/(omega*omega);
        // ZMPPOS_meas_cop = getZmpFromExternalForces();
        //std::cout << " qui " << ZMPPOS_meas_cop << std::endl;

        ofstream myfile;

        myfile.open("../txts/x_m_supp.txt", ios::app);
        myfile << mSupportFoot->getCOM()(0) <<endl; 
        myfile.close();
        myfile.open ("../txts/y_m_supp.txt", ios::app);
        myfile << mSupportFoot->getCOM()(1) <<endl; 
        myfile.close();

        myfile.open("../txts/x_m_sw.txt", ios::app);
        myfile << mSwingFoot->getCOM()(0) <<endl; 
        myfile.close();
        myfile.open ("../txts/y_m_sw.txt", ios::app);
        myfile << mSwingFoot->getCOM()(1) <<endl; 
        myfile.close();

    if(walkState.supportFoot == 0){
        myfile.open("../txts/x_supp.txt", ios::app);
        myfile << desired.rightFootPos(0) <<endl; 
        myfile.close();
        myfile.open ("../txts/y_supp.txt", ios::app);
        myfile << desired.rightFootPos(1) <<endl; 
        myfile.close();
        myfile.open("../txts/x_sw.txt", ios::app);
        myfile << desired.leftFootPos(0) <<endl; 
        myfile.close();
        myfile.open ("../txts/y_sw.txt", ios::app);
        myfile << desired.leftFootPos(1) <<endl; 
        myfile.close();
    }
    else{
        myfile.open("../txts/x_supp.txt", ios::app);
        myfile << desired.leftFootPos(0) <<endl; 
        myfile.close();
        myfile.open ("../txts/y_supp.txt", ios::app);
        myfile << desired.leftFootPos(1) <<endl; 
        myfile.close();
        myfile.open("../txts/x_sw.txt", ios::app);
        myfile << desired.rightFootPos(0) <<endl; 
        myfile.close();
        myfile.open ("../txts/y_sw.txt", ios::app);
        myfile << desired.rightFootPos(1) <<endl; 
        myfile.close();

    }

        myfile.open ("../txts/x_m.txt",ios::app);
        myfile << COMPOS_meas(0) <<endl; 
        myfile.close();
        myfile.open ("../txts/y_m.txt",ios::app);
        myfile << COMPOS_meas(1) <<endl; 
        myfile.close();
        myfile.open ("../txts/xz_m_cop.txt",ios::app);
        myfile << ZMPPOS_meas_cop(0) <<endl; //current.zmpPos
        myfile.close();
        myfile.open ("../txts/yz_m_cop.txt",ios::app);
        myfile << ZMPPOS_meas_cop(1) <<endl; 
        myfile.close();

        myfile.open ("../txts/x.txt",ios::app);
        myfile << desired.comPos(0) <<endl; 
        myfile.close();
        myfile.open ("../txts/y.txt",ios::app);
        myfile << desired.comPos(1) <<endl; 
        myfile.close();

        myfile.open ("../txts/xwithnoise.txt",ios::app);
        myfile << desiredWithNoise.comPos(0) <<endl; 
        myfile.close();
        myfile.open ("../txts/ywithnoise.txt",ios::app);
        myfile << desiredWithNoise.comPos(1) <<endl; 
        myfile.close();
        // myfile.open ("../txts/xwithnoise.txt",ios::app);
        // myfile << mRobot->getCOM()(0) <<endl; 
        // myfile.close();
        // myfile.open ("../txts/ywithnoise.txt",ios::app);
        // myfile << mRobot->getCOM()(1) <<endl; 
        // myfile.close();
        // std::cout << "BE CAREFUL XZ AND YZ AND xzwithnoise AND yzwithnoise saved not correct please change me in storeData()" << std::endl;
        // myfile.open ("../txts/xz.txt",ios::app);
        // myfile << mTorso->getCOM()(0) <<endl; 
        // myfile.close();
        // myfile.open ("../txts/yz.txt",ios::app);
        // myfile << mTorso->getCOM()(1) <<endl; 
        // myfile.close();


        myfile.open ("../txts/xz.txt",ios::app);
        myfile << desired.zmpPos(0) <<endl; 
        myfile.close();
        myfile.open ("../txts/yz.txt",ios::app);
        myfile << desired.zmpPos(1) <<endl; 
        myfile.close();

        myfile.open ("../txts/xzwithnoise.txt",ios::app);
        myfile << desiredWithNoise.zmpPos(0) <<endl; 
        myfile.close();
        myfile.open ("../txts/yzwithnoise.txt",ios::app);
        myfile << desiredWithNoise.zmpPos(1) <<endl; 
        myfile.close();

        myfile.open ("../txts/xd.txt",ios::app);
        myfile << desired.comVel(0) <<endl; 
        myfile.close();
        myfile.open ("../txts/yd.txt",ios::app);
        myfile << desired.comVel(1) <<endl; 
        myfile.close();

        myfile.open ("../txts/xd_m.txt",ios::app);
        myfile << mRobot->getCOMLinearVelocity()(0) <<endl; 
        myfile.close();
        myfile.open ("../txts/yd_m.txt",ios::app);
        myfile << mRobot->getCOMLinearVelocity()(1) <<endl; 
        myfile.close();


        myfile.open ("../txts/xzd.txt",ios::app);
        myfile << xzd <<endl; 
        myfile.close();
        myfile.open ("../txts/yzd.txt",ios::app);
        myfile << yzd <<endl; 
        myfile.close();

        myfile.open ("../txts/error_x.txt",ios::app);
        myfile << error_x <<endl; 
        myfile.close();
        myfile.open ("../txts/error_y.txt",ios::app);
        myfile << error_y <<endl; 
        myfile.close();

        myfile.open ("../txts/errorfoot_x.txt",ios::app);
        myfile << errorfoot_x <<endl; 
        myfile.close();
        myfile.open ("../txts/errorfoot_y.txt",ios::app);
        myfile << errorfoot_y <<endl; 
        myfile.close();

        myfile.open ("../txts/errorfoot_thetax.txt",ios::app);
        myfile << errorfoot_thetax <<endl;
        myfile.close();
        myfile.open ("../txts/errorfoot_thetay.txt",ios::app);
        myfile << errorfoot_thetay <<endl;
        myfile.close();

        myfile.open ("../txts/desiredTorsoPosition_x.txt",ios::app);
        myfile << desired.torsoOrient(0) <<endl; 
        myfile.close();
        myfile.open ("../txts/desiredTorsoPosition_y.txt",ios::app);
        myfile << desired.torsoOrient(1) <<endl; 
        myfile.close();
        myfile.open ("../txts/desiredTorsoPosition_z.txt",ios::app);
        myfile << desired.torsoOrient(2) <<endl; 
        myfile.close();


        myfile.open ("../txts/actualOrientation_x.txt",ios::app);
        myfile << current.torsoOrient(0) <<endl; 
        myfile.close();
        myfile.open ("../txts/actualOrientation_y.txt",ios::app);
        myfile << current.torsoOrient(1) <<endl; 
        myfile.close();
        myfile.open ("../txts/actualOrientation_z.txt",ios::app);
        myfile << current.torsoOrient(2) <<endl; 
        myfile.close();

        myfile.open ("../txts/desiredTorsoPosition_foot_x.txt",ios::app);
        myfile << desired.getRelComPose(walkState.supportFoot)(0) <<endl; 
        myfile.close();
        myfile.open ("../txts/desiredTorsoPosition_foot_y.txt",ios::app);
        myfile << desired.getRelComPose(walkState.supportFoot)(1) <<endl; 
        myfile.close();
        myfile.open ("../txts/desiredTorsoPosition_foot_z.txt",ios::app);
        myfile << desired.getRelComPose(walkState.supportFoot)(2) <<endl; 
        myfile.close();


        myfile.open ("../txts/actualOrientation_foot_x.txt",ios::app);
        myfile << current.getRelComPose(walkState.supportFoot)(0) <<endl; 
        myfile.close();
        myfile.open ("../txts/actualOrientation_foot_y.txt",ios::app);
        myfile << current.getRelComPose(walkState.supportFoot)(1) <<endl; 
        myfile.close();
        myfile.open ("../txts/actualOrientation_foot_z.txt",ios::app);
        myfile << current.getRelComPose(walkState.supportFoot)(2) <<endl; 
        myfile.close();


        if(isDoublePendulum)
        {
        myfile.open ("../txts/xcam.txt",ios::app);
        myfile << desired_cam.comPos(0) <<endl; 
        myfile.close();
        myfile.open ("../txts/ycam.txt",ios::app);
        myfile << desired_cam.comPos(1) <<endl; 
        myfile.close();
        myfile.open ("../txts/xzcam.txt",ios::app);
        myfile << desired_cam.zmpPos(0) <<endl; 
        myfile.close();
        myfile.open ("../txts/yzcam.txt",ios::app);
        myfile << desired_cam.zmpPos(1) <<endl; 
        myfile.close();
        myfile.open ("../txts/xdcam.txt",ios::app);
        myfile << desired_cam.comVel(0) <<endl; 
        myfile.close();
        myfile.open ("../txts/ydcam.txt",ios::app);
        myfile << desired_cam.comVel(1) <<endl; 
        myfile.close();

        myfile.open ("../txts/torquex_desired.txt",ios::app);
        myfile << TorquesDesired(0) <<endl; 
        myfile.close();
        myfile.open ("../txts/torquey_desired.txt",ios::app);
        myfile << TorquesDesired(1) <<endl; 
        myfile.close();
        myfile.open ("../txts/torquez_desired.txt",ios::app);
        myfile << TorquesDesired(2) <<endl; 
        myfile.close();

        myfile.open ("../txts/desiredTorsoVelocity_x.txt",ios::app);
        myfile << mAngularVelocity(0) <<endl; 
        myfile.close();
        myfile.open ("../txts/desiredTorsoVelocity_y.txt",ios::app);
        myfile << mAngularVelocity(1) <<endl; 
        myfile.close();
        myfile.open ("../txts/desiredTorsoVelocity_z.txt",ios::app);
        myfile << mAngularVelocity(2) <<endl; 
        myfile.close();

        // myfile.open ("../txts/virt_torqx.txt",ios::app);
        // myfile << virt_torq(0) <<endl; 
        // myfile.close();
        // myfile.open ("../txts/virt_torqy.txt",ios::app);
        // myfile << virt_torq(1) <<endl; 
        // myfile.close();

        myfile.open ("../txts/actualAngVel_x.txt",ios::app);
        myfile << mTorso->getAngularVelocity()(0) <<endl; 
        myfile.close();
        myfile.open ("../txts/actualAngVel_y.txt",ios::app);
        myfile << mTorso->getAngularVelocity()(1) <<endl; 
        myfile.close();
        myfile.open ("../txts/actualAngVel_z.txt",ios::app);
        myfile << mTorso->getAngularVelocity()(2) <<endl; 
        myfile.close();


        }

        myfile.open ("../txts/error_torsoAngle_foot_x.txt",ios::app);
        myfile << error_torsoAngle_foot(0) <<endl; 
        myfile.close();
        myfile.open ("../txts/error_torsoAngle_foot_y.txt",ios::app);
        myfile << error_torsoAngle_foot(1) <<endl; 
        myfile.close();
        myfile.open ("../txts/error_torsoAngle_foot_z.txt",ios::app);
        myfile << error_torsoAngle_foot(2) <<endl;
        myfile.close();

        myfile.open ("../txts/error_torsoPos_foot_x.txt",ios::app);
        myfile << error_torsoPos_foot(0) <<endl; 
        myfile.close();
        myfile.open ("../txts/error_torsoPos_foot_y.txt",ios::app);
        myfile << error_torsoPos_foot(1) <<endl; 
        myfile.close();
        myfile.open ("../txts/error_torsoPos_foot_z.txt",ios::app);
        myfile << error_torsoPos_foot(2) <<endl;
        myfile.close();

        myfile.open ("../txts/error_footAngle_foot_x.txt",ios::app);
        myfile << error_footAngle_foot(0) <<endl; 
        myfile.close();
        myfile.open ("../txts/error_footAngle_foot_y.txt",ios::app);
        myfile << error_footAngle_foot(1) <<endl; 
        myfile.close();
        myfile.open ("../txts/error_footAngle_foot_z.txt",ios::app);
        myfile << error_footAngle_foot(2) <<endl; 
        myfile.close();

        myfile.open ("../txts/error_footPos_foot_x.txt",ios::app);
        myfile << error_footPos_foot(0) <<endl; 
        myfile.close();
        myfile.open ("../txts/error_footPos_foot_y.txt",ios::app);
        myfile << error_footPos_foot(1) <<endl; 
        myfile.close();
        myfile.open ("../txts/error_footPos_foot_z.txt",ios::app);
        myfile << error_footPos_foot(2) <<endl;
        myfile.close();

}