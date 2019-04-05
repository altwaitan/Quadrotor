#include <Eigen.h>     // Calls main Eigen matrix class library
#include <Eigen/LU>             // Calls inverse, determinant, LU decomp., etc.
#include "Quadrotor.h"
#include "mavlink.h"

using namespace Eigen;    // Eigen related statement; simplifies syntax for declaration of matrices

class Communication
{
public:
  // Variables
  struct _CommunicationState {
    float thrust, phi, theta, r;
  };
  _CommunicationState CommunicationState;
  float moving_average[13][5];
  float sum_average[13];
  int Counter = 0;

  // Methods
  void ROS_Send(Quadrotor *quadrotor);
  void ROS_Receive(Quadrotor *quadrotor);
private:
};
