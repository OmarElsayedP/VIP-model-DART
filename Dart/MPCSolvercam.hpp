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

class MPCSolvercam{
    public:
    MPCSolvercam(FootstepPlan* _footstepPlann, int sim, bool VIP);
    ~MPCSolvercam();

    // Compute the next desired state starting from the current state
    State solve(State current, State current_cam, WalkState walkState, double vRefX, double vRexfY, double omegaRef, double fx, double mass);

    // some stuff
    int itr;
    int fsCount, old_fsCount, adaptation_memo, ds_samples, ct;
    int prova, sim, robust;
    bool VIP;
    Eigen::VectorXd push;

    double xz_dot, yz_dot, xz_dot_cam, yz_dot_cam;

    private:
    // Matrices for prediction
    Eigen::VectorXd p;
    Eigen::MatrixXd P;
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
    Eigen::MatrixXd Aang;
    Eigen::VectorXd bangMax;
    Eigen::VectorXd BangMin;

    // Matrices for the stacked constraints
    Eigen::MatrixXd AConstraintcam;
    Eigen::VectorXd bConstraintMaxcam;
    Eigen::VectorXd bConstraintMincam;

    // Footstep plan
    FootstepPlan* footstepPlan;

    // Midpoint of ZMP constraint
    Eigen::VectorXd x_midpoint, y_midpoint;
};
