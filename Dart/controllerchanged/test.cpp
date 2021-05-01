// # include <eigen3/Eigen/Core>
# include <iostream>
// #include <math.h>
#include <vector>
#include <limits.h> /* PATH_MAX */
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <eigen3/Eigen/Core>
#include "../plotty/include/plotty/matplotlibcpp.hpp"

using namespace std;

void print(std::vector<int> &input)
{
    for (int i = 0; i < input.size(); i++) {
        std::cout <<"small v is:"<< input.at(i) << endl;
    }
    input.push_back(3333333);
}

inline Eigen::MatrixXd matrixPower(Eigen::MatrixXd& A, int exp){

	Eigen::MatrixXd result = Eigen::MatrixXd::Identity(A.rows(),A.cols());

	for (int i=0; i<exp;++i)
        	result *= A;

	return result;
}

int main() {
  char actualpath[PATH_MAX];
  char* meow = realpath("../Controller.cpp",actualpath);
  std::vector<int> v;
  v.push_back(1);
  v.push_back(2);
  v.push_back(3);
  v.push_back(4);
  v.push_back(5);
  v.push_back(6);
  v.push_back(7);
  v.push_back(8);
  std::vector<std::vector<int>> V;
  V.push_back(v);
  V.push_back(v);
  int x = v.at(2);
  int X = V.at(0).at(0);
  // print(v);
  // std::cout << "v after referencing function" << '\n';
  // print(v);
  // std::cout << "x is : " << x << '\n';
  // std::cout << "Size of BIG V is : " << V.size() << '\n';
  if (meow) {
      // printf("This source is at %s.\n", actualpath);
  } else {
      perror("realpath");
      exit(EXIT_FAILURE);
  }
  // std::ofstream ofs ("test.txt", std::ofstream::out);
  //
  // ofs << "lorem ipsum";
  //
  // ofs.close();
  Eigen::MatrixXd CCfull = Eigen::MatrixXd::Zero(4,4);
  Eigen::MatrixXd eigVec = Eigen::VectorXd::Zero(8);

  eigVec << 1,2,3,4,5,6,7,8;
  CCfull << 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,17;
  Eigen::MatrixXd CC = CCfull.block(0,0,4,1);
  Eigen::MatrixXd CC2 = CCfull.block(0,1,2,2);
  // std::cout << CCfull;

  Eigen::VectorXd halfeigVec = eigVec.block(0,0,4,1);
  // std::cout << "Vector is : " << halfeigVec << endl;
  // std::cout << "Eigen matrix CCfull is : " << '\n' << CCfull << '\n';
  // std::cout << "Eigen matrix block(0,0,2,2) is : " << '\n' << CC << '\n';
  // std::cout << "Eigen matrix block(0,1,2,2) is : " << '\n' << CC2 << '\n';
  // std::cout << "size of matrix (rows)" << CCfull.rows () << endl;

  std::vector<Eigen::VectorXd> footstepPlan;
  footstepPlan.push_back(halfeigVec);
  double pppw = pow(-1,3);
  // std::cout << "pppw is : " << pppw << '\n';

// bool VIP = false;

// if(!VIP)
//   {
//     std::cout << "VIP is FALSE"<< std::endl;
//   }

// else{
//   std::cout << "VIP is true" << std::endl;

// }
// VIP = true;
// if(!VIP)
//   {
//     std::cout << "VIP is FALSE"<< std::endl;
//   }

// else{
//   std::cout << "VIP is true" << std::endl;

// }    
   Eigen::VectorXd costFunctionF;
   costFunctionF = Eigen::VectorXd::Zero(2*(2*208+4));
   int qZ = 0;
   int qF = 100;
    Eigen::VectorXd xCandidateFootsteps(4), yCandidateFootsteps(4);
    xCandidateFootsteps << 1,2,3,4;
    yCandidateFootsteps << 1, -1 , 2, -2;

     Eigen::VectorXd p = Eigen::VectorXd::Ones(408);
     Eigen::MatrixXd P = Eigen::MatrixXd::Ones(408,408)*0.01;

        Eigen::MatrixXd CcFull = Eigen::MatrixXd::Random(408,4+1)*4;


    //     Eigen::MatrixXd Cc = CcFull.block(0,1,408,4);
    // Eigen::VectorXd currentFootstepZmp = CcFull.col(0);

    //     costFunctionF << qZ*P.transpose()*(p*0.2),
    //     -qZ*Cc.transpose()*(p*0.2 - currentFootstepZmp*4.0) - qF*xCandidateFootsteps,
    //     qZ*P.transpose()*(- currentFootstepZmp*3.0),
    //     -qZ*Cc.transpose()*(- currentFootstepZmp*3.0) - qF*yCandidateFootsteps,
    //     costFunctionF << qZ*P.transpose()*(p*0.2 - currentFootstepZmp*4.0),
    //     -qZ*Cc.transpose()*(p*0.2 - currentFootstepZmp*4.0) - qF*xCandidateFootsteps,
    //     qZ*P.transpose()*(- currentFootstepZmp*3.0),
    //     -qZ*Cc.transpose()*( - currentFootstepZmp*3.0) - qF*yCandidateFootsteps;
    //     // , new std::vector<double>(N,0.0), 
    //     // new std::vector<double>(N,0.0);


    //     std::cout << "costFunctionF" << costFunctionF << std::endl;

for(int i = 0; i<10; i++){
  if(i!=4 || i!=5) std::cout << i;
}



  return 0;
}
