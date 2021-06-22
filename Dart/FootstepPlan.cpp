#include "FootstepPlan.hpp"

FootstepPlan::FootstepPlan() {}

FootstepPlan::~FootstepPlan() {}

void FootstepPlan::plan(std::vector<Vref> vrefSequence, Eigen::VectorXd initialLeftFoot, Eigen::VectorXd initialRightFoot, bool firstSupportFootIsLeft) {
    // for (int i = 0; i < nFootsteps; ++i) {
    //     // it is in world frame!
    //     Eigen::VectorXd tempFootstep(7);
    //     if (i<1) tempFootstep << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, i*50;
    //     if (i == 1) tempFootstep << 0.0, pow(-1,i)*0.075, 0.0, 0.0, 0.0, 0.0, i*50;
    //     if(i > 1) tempFootstep << i*0.1, pow(-1,i)*0.075, 0.0, 0.0, 0.0, 0.0, i*50;
    //     footstepPlan.push_back(tempFootstep);
    //     // std::cout << tempFootstep << '\n';
    //     // std::cout << i << '\n';
    // }

        // for (int i = 0; i < nFootsteps; i++) {
        //     // it is in world frame!
        //     Eigen::VectorXd tempFootstep(7);
        //     if (i<2) tempFootstep << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, i*50;
        //     else tempFootstep << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, i*50;
        //     // if (i<2) tempFootstep << 0.0, pow(-1,i+1)*0.08, 0.0, 0.0, 0.0, 0.0, i*50;
        //     // else tempFootstep << (i-1)*0.1, pow(-1,i+1)*0.08, 0.0, 0.0, 0.0, 0.0, i*50;
        //     footstepPlan.push_back(tempFootstep);
        // }
    // //Normal footsteps
    //     for (int i = 0; i < nFootsteps; i++) {
    //         // it is in world frame!
    //         Eigen::VectorXd tempFootstep(7);
    //         // if (i<2) tempFootstep << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, i*50;
    //         // else tempFootstep << (i-1)*0.1, pow(-1,i+1)*0.08, 0.0, 0.0, 0.0, 0.0, i*50;
    //         // if(i==0) tempFootstep << 0.0, 0.075, 0.0, 0.0, 0.0, 0.0, i*50;
    //         if(i==0) tempFootstep << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, i*50;
    //         if (i==1) tempFootstep << 0.0, -0.075, 0.0, 0.0, 0.0, 0.0, i*50;
    //         if(i>1) tempFootstep << (i-1)*0.05, pow(-1,i)*0.075, 0.0, 0.0, 0.0, 0.0, i*50;

    //         // if(i>1) tempFootstep << (i-1)*0.15, pow(-1,i)*0.075, 0.0, 0.0, 0.0, 0.0, i*50;

    //         footstepPlan.push_back(tempFootstep);
    //     }

    //slower footsteps (Must change singleSupportDuration, doubleSupportDuration and doubleSupportSamples)
        for (int i = 0; i < nFootsteps; i++) {
            // it is in world frame!
            Eigen::VectorXd tempFootstep(7);
            // if (i<2) tempFootstep << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, i*50;
            // else tempFootstep << (i-1)*0.1, pow(-1,i+1)*0.08, 0.0, 0.0, 0.0, 0.0, i*50;
            // if(i==0) tempFootstp << 0.0, 0.075, 0.0, 0.0, 0.0, 0.0, i*50;
            if(i==0)  tempFootstep << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, i*70;
            if(i==1) tempFootstep << 0.0, -0.075, 0.0, 0.0, 0.0, 0.0, i*70;
            // if(i>1) tempFootstep << (i-1)*0.05, pow(-1,i)*0.075, 0.0, 0.0, 0.0, 0.0, i*70;

            if(i>1) tempFootstep << (i-1)*0.1, pow(-1,i)*0.075, 0.0, 0.0, 0.0, 0.0, i*70;

            // if(i>7) tempFootstep << (7-1)*0.1, pow(-1,i)*0.075, 0.0, 0.0, 0.0, 0.0, i*70;

            // tempFootstep << 0.0, pow(-1,i)*0.075, 0.0, 0.0, 0.0, 0.0, i*70;
            footstepPlan.push_back(tempFootstep);
        }

   // for (int i = 0; i < nFootsteps; i++) {
   //          // it is in world frame!
   //          Eigen::VectorXd tempFootstep(7);
   //          // if (i<2) tempFootstep << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, i*50;
   //          // else tempFootstep << (i-1)*0.1, pow(-1,i+1)*0.08, 0.0, 0.0, 0.0, 0.0, i*50;
   //          if(i%2==0){
   //           tempFootstep << 0.0, -0.075, 0.0, 0.0, 0.0, 0.0, i*50;
   //           std::cout << "i = " << i << std::endl;
   //          }
   //          else tempFootstep << 0.0, 0.075, 0.0, 0.0, 0.0, 0.0, i*50;
   //          footstepPlan.push_back(tempFootstep);
   //      }   
        // std::cout << "footstepPlan = " << footstepPlan;

        // std::cout << "footstepPlan = " << footstepPlan.at(0) << ", " << footstepPlan.at(1) << std::endl;

        // for(int i = 0; i < nFootsteps; i++)
        // {
        //     std::cout<< "Footstep no." << i << "= (" << footstepPlan.at(i)(0) << ", " << footstepPlan.at(i)(1) << ")" << std::endl;
        // }
}

// .at(i) returns value of index i of a std vector
Eigen::VectorXd FootstepPlan::getFootstep(int num) { return footstepPlan.at(num); }
//.head(i) returns the 3 first elements of the mini eigen vector xd in the big vector footstepPlan
Eigen::VectorXd FootstepPlan::getFootstepPosition(int num) { return footstepPlan.at(num).head(3); }

double FootstepPlan::getFootstepOrientation(int num) { return footstepPlan.at(num)(3); }

int FootstepPlan::getFootstepStartTiming(int num) { return (int)footstepPlan.at(num)(6); }

int FootstepPlan::getFootstepEndTiming(int num) { return (num + 1 < footstepPlan.size()) ? (int)footstepPlan.at(num + 1)(6) : -1; }

int FootstepPlan::getFootstepIndexAtTime(int time) {
    int footstepIndex = 0;
    while (getFootstepEndTiming(footstepIndex) < time and footstepIndex < footstepPlan.size()) footstepIndex++;

    return footstepIndex;
}

int FootstepPlan::getSize() { return footstepPlan.size(); }

std::vector<Eigen::VectorXd> FootstepPlan::getPlan() { return footstepPlan; }
