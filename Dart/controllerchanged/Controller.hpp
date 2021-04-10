#pragma once

#include <Eigen/Core>
#include "dart/dart.hpp"
#include "MPCSolver.hpp"
#include "Utility.hpp"
#include  <fstream>
#include "utils.cpp"
#include "types.hpp"
#include "FootstepPlan.hpp"
#include "StateFiltering.hpp"
#include <memory>
#include <random>

class Controller
{
public:
  Controller(dart::dynamics::SkeletonPtr _robot, dart::simulation::WorldPtr _world);
  virtual ~Controller();

  Eigen::Vector3d getZmpFromExternalForces();

  void update();

  Eigen::MatrixXd getTorsoAndSwfJacobian();
  Eigen::MatrixXd getTorsoAndSwfJacobianDeriv();

  Eigen::VectorXd getJointVelocitiesQp(State current, State desired);
  Eigen::VectorXd getJointVelocitiesQpAcceleration(State current, State desired);

  void setInitialConfiguration();
  void ArmSwing();
  void AnkleRegulation();

 Eigen::VectorXd getJointVelocitiesStacked(Eigen::VectorXd, Eigen::VectorXd,
		  Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd);

  Eigen::Vector3d getRPY(dart::dynamics::BodyNode*, dart::dynamics::BodyNode*);
  Eigen::Vector3d getRPY(dart::dynamics::BodyNode*);

  void storeData();

  virtual void keyboard(unsigned char _key, int _x, int _y);

private:
  dart::dynamics::SkeletonPtr mRobot;

  dart::dynamics::BodyNode* mTorso;

  dart::simulation::WorldPtr mWorld;

  bool balancePoint;
  bool TORSO = false;
  bool COM = true;
  int sim_;

  MPCSolver* solver;

  dart::dynamics::BodyNode* mLeftFoot;
  dart::dynamics::BodyNode* mRightFoot;
  dart::dynamics::BodyNode* mSupportFoot;
  dart::dynamics::BodyNode* mSwingFoot;
  dart::dynamics::BodyNode* mBase;

  State desired;
  State current;
  WalkState walkState;

  FootstepPlan* footstepPlan;
  //std::vector<Eigen::VectorXd> footstepPlan;
  double tail_counter = -100;
  Eigen::Vector3d memopush;
  double px, py;
  StateFiltering* Filter;

  /*
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<> noise{0,10};  /**/

};
