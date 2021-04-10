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
    MPCSolver(FootstepPlan* _footstepPlan);
    ~MPCSolver();

    // Compute the next desired state starting from the current state
    State solve(State current, WalkState walkState, double vRefX, double vRefY);

    // some stuff
    int itr;
    int fsCount, old_fsCount, adaptation_memo, ds_samples, ct;
    int prova;
    Eigen::VectorXd push;
    double xz_dot, yz_dot;

    private:
    // Matrices for prediction
    Eigen::VectorXd p;
    Eigen::MatrixXd P;

    // Matrices for cost function
    Eigen::MatrixXd costFunctionH;
    Eigen::VectorXd costFunctionF;

    // Matrices for stability constraint
    Eigen::MatrixXd Aeq;
    Eigen::VectorXd beq;

    //Matrices for balance constraint
    Eigen::MatrixXd AZmp;
    Eigen::VectorXd bZmpMax;
    Eigen::VectorXd bZmpMin;

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

    // Footstep plan
    FootstepPlan* footstepPlan;
};
