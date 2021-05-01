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

class MPCSolver{
    public:
    MPCSolver(FootstepPlan* _footstepPlann, int sim, bool VIP);
    ~MPCSolver();

    // Compute the next desired state starting from the current state
    State solve(State current, State current_cam, WalkState walkState, double vRefX, double vRexfY, double omegaRef, Eigen::VectorXd virtualNoise, double mass);

    // some stuff
    int itr;
    int fsCount, old_fsCount, adaptation_memo, ds_samples, ct;
    int prova, sim, robust;
    bool VIP;
    Eigen::VectorXd push;

    // hpipm timing adaptation QP
    int timingQP(State current_s, WalkState walkState_s, double anticipativeTail_x, double anticipativeTail_y);
    double xz_dot, yz_dot, xz_dot_cam, yz_dot_cam;

    private:
    // Matrices for prediction
    Eigen::VectorXd p;
    Eigen::MatrixXd P;
    Eigen::MatrixXd Vu;
    Eigen::MatrixXd Vs;

    // Matrices for cost function
    Eigen::MatrixXd costFunctionH;
    Eigen::VectorXd costFunctionF;
    Eigen::MatrixXd costFunctionHcam;
    Eigen::VectorXd costFunctionFcam;

    // Matrices for stability constraint
    Eigen::MatrixXd Aeq;
    Eigen::VectorXd beq;

    //Matrices for balance constraint
    Eigen::MatrixXd AZmp;
    Eigen::VectorXd bZmpMax;
    Eigen::VectorXd bZmpMin;

    // Matrices for stability constraint
    Eigen::MatrixXd Aeq_cam;
    Eigen::VectorXd beq_cam;

    //Matrices for balance constraint
    Eigen::MatrixXd AZmp_cam;
    Eigen::VectorXd bZmpMax_cam;
    Eigen::VectorXd bZmpMin_cam;

    // Matrices for feasibility constraints
    Eigen::MatrixXd AFootsteps;
    Eigen::VectorXd bFootstepsMax;
    Eigen::VectorXd bFootstepsMin;

    //Matricies for angle constraint
    Eigen::MatrixXd Aang;
    Eigen::VectorXd bangMax;
    Eigen::VectorXd BangMin;


    // Matrices for swing foot constraints
    Eigen::MatrixXd ASwingFoot;
    Eigen::VectorXd bSwingFoot;

    // Matrices for the stacked constraints
    Eigen::MatrixXd AConstraint;
    Eigen::VectorXd bConstraintMax;
    Eigen::VectorXd bConstraintMin;

    Eigen::MatrixXd AConstraintcam;
    Eigen::VectorXd bConstraintMaxcam;
    Eigen::VectorXd bConstraintMincam;

    // Footstep plan
    FootstepPlan* footstepPlan;

    // Midpoint of ZMP constraint
    Eigen::VectorXd x_midpoint, y_midpoint;
};
