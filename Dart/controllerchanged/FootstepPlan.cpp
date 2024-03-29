#include "FootstepPlan.hpp"

FootstepPlan::FootstepPlan() {}

FootstepPlan::~FootstepPlan() {}

void FootstepPlan::plan(std::vector<Vref> vrefSequence, Eigen::VectorXd initialLeftFoot, Eigen::VectorXd initialRightFoot, bool firstSupportFootIsLeft) {
  int nFootsteps = 22;
    for (int i = 0; i < nFootsteps; ++i) {
        // it is in world frame!
        Eigen::VectorXd tempFootstep(7);
        if (i<1) tempFootstep << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, i*50;
        else tempFootstep << i*0.1, pow(-1,i+2)*0.075, 0.0, 0.0, 0.0, 0.0, i*50;
        footstepPlan.push_back(tempFootstep);
        std::cout << tempFootstep << '\n';
        std::cout << i << '\n';
    }
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
