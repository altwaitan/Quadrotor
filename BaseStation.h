#include "Quadrotor.h"

class BaseStation
{
public:
  // Variables
  float moving_average[13][5];
  float sum_average[13];

  // Methods
  void HTCVive(Quadrotor *quadrotor);
  void HTCViveProcess(Quadrotor *quadrotor);
  void HTCViveMovingAverage(Quadrotor *quadrotor);
private:
};
