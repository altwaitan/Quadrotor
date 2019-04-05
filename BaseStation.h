#include <Eigen.h>     // Calls main Eigen matrix class library
#include <Eigen/LU>             // Calls inverse, determinant, LU decomp., etc.
#include "Quadrotor.h"
#include "mavlink.h"

using namespace Eigen;    // Eigen related statement; simplifies syntax for declaration of matrices

class BaseStation
{
public:
  // Variables
  struct _HTCViveState {
    float phi, theta, psi, p, q, r;
  };
  _HTCViveState HTCViveState;
  float moving_average[13][5];
  float sum_average[13];
  int HTCViveCounter = 0;

  // Methods
  void HTCVive(Quadrotor *quadrotor);
  void HTCViveProcess(Quadrotor *quadrotor);
  void HTCViveMovingAverage(Quadrotor *quadrotor);
  void HTCViveComplementaryFilter(Quadrotor *quadrotor);
private:
};
