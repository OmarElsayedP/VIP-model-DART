#pragma once

#include <math.h>

// Definable parameters
// ********************

// Times
const double mpcTimeStep = 0.01; //0.05;
const double controlTimeStep = 0.01;
const double singleSupportDuration = 0.3;
// const double singleSupportDuration = 0.5;
const double doubleSupportDuration = 0.2;
const double predictionTime = 1.0;

// Walk parameters
const double stepHeight = 0.025; //0.025
const double comTargetHeight = 0.71; //0.72
// const double comTargetHeight = 0.75;
const double kSwingFoot = 0.08; //0.05;
// const double kSwingFoot = 1.0/28.0; //0.05;
// const int nFootsteps = 20;/
const int nFootsteps = 30;
// const int nFootsteps = 40;


// Fixed step parameters
const double stepx = 0.06;//0.0;//
const double stepy = 0.1;
const double FootStepY = 0.075;

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
const double qZd_cam = 0.1;

const double qVx = 0;//100;
const double qVy = 0;//100;
const double qZ = 0; ////0;//1;
const double qF = 10000000;//100;

const double qThx = 1; //100  //10 with disturbance
const double qThy = 1; //100  //10 with disturbance


// Kinematic control
const double IKerrorGain = 1.0; //0.99;  1.0

// Used in the code
// ****************

const double omega = sqrt(9.81/comTargetHeight);
const int N = round(predictionTime/mpcTimeStep);
// const int S = round(singleSupportDuration/mpcTimeStep);
// const int D = round(doubleSupportDuration/mpcTimeStep);
const int M = 3; //ceil(N/(S+D));
// const int M = ceil(N/(S+D))+1;
const int doubleSupportSamples = doubleSupportDuration/controlTimeStep;
const double forceLimit = 100;

const bool VIP_global = 1;
const bool angleConstraint = 0;
const double forceLimittorques = 100.0;
const bool termConstr = 0;
const bool isVirtualNoise = 0;