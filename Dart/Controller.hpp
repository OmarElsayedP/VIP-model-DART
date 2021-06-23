#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include "dart/dart.hpp"
#include "MPCSolver.hpp"
#include "MPCSolvercam.hpp"
#include "Utility.hpp"
#include  <iostream>
#include  <fstream>
#include "utils.cpp"
#include "types.hpp"
#include "FootstepPlan.hpp"
#include "StateFiltering.hpp"
#include <memory>
#include <random>
// #include "madplotlib/Madplotlib.h"

class Controller
{
public:
  Controller(dart::dynamics::SkeletonPtr _robot, dart::simulation::WorldPtr _world);
  virtual ~Controller();

  Eigen::Vector3d getZmpFromExternalForces();

  void update();

  Eigen::MatrixXd getTorsocamJacobian();
  Eigen::MatrixXd getTorsoAndSwfJacobian();
  Eigen::MatrixXd getTorsoAndSwfJacobianDeriv();

  Eigen::VectorXd getJointVelocitiesQp(State current, State desired);
  Eigen::VectorXd getJointVelocitiesQpAcceleration(State current, State desired);

  void setInitialConfiguration();
  void ArmSwing();
  void AnkleRegulation();

  Eigen::VectorXd getJointVelocitiesStacked(Eigen::VectorXd, Eigen::VectorXd,
      Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd);

  Eigen::VectorXd getJointVelocitiesDoublePendulum(Eigen::VectorXd, Eigen::VectorXd,
      Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd);
  
    Eigen::VectorXd getJointVelocitiesStacked_worldframe(Eigen::VectorXd, Eigen::VectorXd,
      Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd);

  Eigen::VectorXd getJointVelocitiesStackedcam(Eigen::VectorXd, Eigen::VectorXd,
      Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd);

  Eigen::Vector3d getRPY(dart::dynamics::BodyNode*, dart::dynamics::BodyNode*);
  Eigen::Vector3d getRPY(dart::dynamics::BodyNode*);

  void storeData();
  Eigen::Vector3d computeTorques();
  Eigen::Vector3d computeAngularMomentum();
  Eigen::Matrix3d getMomentOfInertia();
  Eigen::MatrixXd getTorsoAndSwfJacobian_worldframe();
  
  virtual void keyboard(unsigned char _key, int _x, int _y);

private:
  dart::dynamics::SkeletonPtr mRobot;

  dart::dynamics::BodyNode* mTorso;

  dart::simulation::WorldPtr mWorld;

  bool balancePoint;
  bool TORSO = false;
  bool COM = true;
  int sim_;
  bool isDoublePendulum = false;
  bool VIPsturn = false;
  double mRobotMass;
  double mTorsoMass;
  Eigen::VectorXd prevQdot;
  Eigen::Vector3d mAngularVelocity;
  Eigen::Vector3d mAngularPosition;

  MPCSolver* solver;
  MPCSolvercam* solver_cam;

  dart::dynamics::BodyNode* mLeftFoot;
  dart::dynamics::BodyNode* mRightFoot;
  dart::dynamics::BodyNode* mSupportFoot;
  dart::dynamics::BodyNode* mSwingFoot;
  dart::dynamics::BodyNode* mBase;

  State desired;
  State previous_desired;
  State current;
  State desiredWithNoise;
  State desired_cam;
  State current_cam;

  WalkState walkState;

  FootstepPlan* footstepPlan;
  //std::vector<Eigen::VectorXd> footstepPlan;
  double tail_counter = -100;
  Eigen::Vector3d memopush;
  double px, py;
  double fx = 0.0;
  double fy = 0.0;

  double xcom_tot;
  double ycom_tot;

  double xdcom_tot;
  double ydcom_tot;

  double xzcom_tot;
  double yzcom_tot;

  double xzd;
  double yzd;

Eigen::Vector2d virt_torq_zeros_allthetime;
Eigen::Vector2d virt_torq;

Eigen::VectorXd TorquesDesired = Eigen::VectorXd::Zero(3);
  
StateFiltering* Filter;

double errorfoot_x;
double errorfoot_y;
double errorfoot_thetax;
double errorfoot_thetay;
double error_x;
double error_y;

double error_xd;
double error_yd;
double error_xa;
double error_ya;

Eigen::Vector3d error_torsoAngle_foot;
Eigen::Vector3d error_torsoPos_foot;
Eigen::Vector3d error_footAngle_foot;
Eigen::Vector3d error_footPos_foot;

//   /*
  // std::random_device rd{};
  // std::mt19937 gen{rd()};
  // std::normal_distribution<> noise{0,10};  /**/

};
