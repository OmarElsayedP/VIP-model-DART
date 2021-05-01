#pragma once

#include <Eigen/Core>
#include "qpOASES/qpOASES.hpp"
#include "types.hpp"
#include "parameters.cpp"
#include "utils.cpp"
#include <vector>
#include <iostream>
#include <fstream>
#include "FootstepPlan.hpp"
#include <chrono>
#include "dart/dart.hpp"

class MPCSolvercam{
    public:
    MPCSolvercam(FootstepPlan* _footstepPlann, int sim, bool CAM, double torsomass, Eigen::Matrix3d MOI, Eigen::Vector3d theta_max);
    ~MPCSolvercam();

    // Compute the next desired state starting from the current state
    State solve(State current, State current_cam, WalkState walkState, double mass, const dart::dynamics::SkeletonPtr& mRobot, double x_tot, double y_tot, 
        Eigen::Vector3d AngularPosition, Eigen::Vector3d AngularVelocity);

    // some stuff
    int itr;
    int fsCount, old_fsCount, adaptation_memo, ds_samples, ct;
    int sim;
    bool CAM;

    double xz_dot, yz_dot, xz_dot_cam, yz_dot_cam;

    private:
    // Matrices for prediction
    Eigen::VectorXd p;
    Eigen::MatrixXd P;
    Eigen::VectorXd pmg;
    Eigen::MatrixXd Pmg;
    Eigen::MatrixXd Pthdd;

    Eigen::MatrixXd Vu;
    Eigen::MatrixXd Vs;

    // Matrices for cost function
    Eigen::MatrixXd costFunctionHcam;
    Eigen::VectorXd costFunctionFcam;

    // Matrices for stability constraint
    Eigen::MatrixXd Aeq_cam;
    Eigen::VectorXd beq_cam;

    //Matrices for balance constraint
    Eigen::MatrixXd AZmp_cam;
    Eigen::VectorXd bZmpMax_cam;
    Eigen::VectorXd bZmpMin_cam;

    //Matricies for angle constraint
    Eigen::MatrixXd AAngleconstr;
    Eigen::VectorXd bAngleConstrMin;
    Eigen::VectorXd bAngleConstrMax;

    //Matricies for terminal constraint
    Eigen::MatrixXd Atermconstr;
    Eigen::VectorXd btermconstr;

    // Matrices for the stacked constraints
    Eigen::MatrixXd AConstraintcam;
    Eigen::VectorXd bConstraintMaxcam;
    Eigen::VectorXd bConstraintMincam;

    // Footstep plan
    FootstepPlan* footstepPlan;

    // Midpoint of ZMP constraint
    Eigen::VectorXd x_midpoint, y_midpoint;

    Eigen::Vector3d theta_max;
    Eigen::Vector3d currentTheta;
    Eigen::Vector3d currentThetaD;
    Eigen::Vector3d currentThetaDD;

    Eigen::Matrix3d MOI;
    Eigen::MatrixXd desiredTorque;
};
