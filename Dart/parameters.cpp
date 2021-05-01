#pragma once

#include <math.h>

// Definable parameters
// ********************

// Times
const double mpcTimeStep = 0.01; //0.05;
const double controlTimeStep = 0.01;
const double singleSupportDuration = 0.25;
const double doubleSupportDuration = 0.25;
const double predictionTime = 1.0;

// Walk parameters
const double stepHeight = 0.03;
// const double comTargetHeight = 0.71; //0.72
const double comTargetHeight = 0.75;
const double kSwingFoot = 0.05; //0.05;
const int nFootsteps = 20;

// Fixed step parameters
const double stepx = 0.06;//0.0;//
const double stepy = 0.1;

// Constraints
const double thetaMax = 0.30;
const double footConstraintSquareWidth = 0.08;
const double initalfootConstrainSquarewidthx = 0.08;
const double initalfootConstrainSquarewidthy = 0.08/2 + 0.2;
const double deltaXMax = 0.25;
const double deltaYIn = 0.15;
const double deltaYOut = 0.28;

// Cost function weights
const double qZd = 1;
const double qVx = 0;//100;
const double qVy = 0;//100;
const double qZ = 0; ////0;//1;
const double qF = 10000000;//100;

const double qZd_cam = 1;

// Kinematic control
const double IKerrorGain = 1.0; //0.99;  1.0

// Used in the code
// ****************

const double omega = sqrt(9.81/comTargetHeight);
const int N = round(predictionTime/mpcTimeStep);
const int S = round(singleSupportDuration/mpcTimeStep);
const int D = round(doubleSupportDuration/mpcTimeStep);
const int M = 3; //ceil(N/(S+D));
// const int M = ceil(N/(S+D))+1;
const int doubleSupportSamples = 25;
const double forceLimit = 100;

const bool VIP_global = 0;
const bool angleConstraint = 0;
const double forceLimittorques = 100.0;