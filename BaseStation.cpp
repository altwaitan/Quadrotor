#include "BaseStation.h"

// HTC Vive Base Station
// Receives position and orientation of an object through Xbee
// Source: Shi Lu, "Modeling, Control and Design of a Quadrotor Platform for Indoor Environments," 2018.
void BaseStation::HTCVive(Quadrotor *quadrotor)
{
  if (quadrotor->channel.CH5 > 1900 && quadrotor->channel.CH5 < 2000)
  {
    String data = "";
    while (Serial1.available() > 0)
    {
      data += char(Serial1.read());
    }
    quadrotor->Xbee_length = data.length();

    for (int i = 0; i < quadrotor->Xbee_length; i++)
    {
      quadrotor->Xbee_data[100 + i] = data[i];
    }

    if (quadrotor->Xbee_length == 4)
    {
      short temp = 0;
      for (uint8_t i = 0; i < 4; i++)
      {
        temp = temp * 10 + (quadrotor->Xbee_data[i] - '0');
      }
      if (temp >= 1000 && temp <= 5000)
        quadrotor->Xbee_command = temp;
    }
    else if (quadrotor->Xbee_length > 5 && quadrotor->Xbee_length < 12 && (quadrotor->Xbee_data[100 + 0] == '&' && quadrotor->Xbee_data[100 + 5] == '_' && quadrotor->Xbee_data[100 + quadrotor->Xbee_length - 1] == '*'))
    {
      // Ignore
    }
    else if (quadrotor->Xbee_length >= 20)
    {
      BaseStation::HTCViveProcess(quadrotor);
      BaseStation::HTCViveMovingAverage(quadrotor);
    }
  }
}

// Process the received data and assign them to each Quadrotor state variable.
// Source: Shi Lu, "Modeling, Control and Design of a Quadrotor Platform for Indoor Environments," 2018.

void BaseStation::HTCViveProcess(Quadrotor *quadrotor)
{
  for (int16_t i = 100 + 3; i < 100 + quadrotor->Xbee_length - 5; i++)
  {
    switch (quadrotor->Xbee_data[i])
    {
      case 21:
        if (quadrotor->Xbee_data[i + 1] == 101)
        {
          if ((quadrotor->Xbee_data[i - 1] == 131 && quadrotor->Xbee_data[i - 2] == 121 && quadrotor->Xbee_data[i - 3] == 111) && (quadrotor->Xbee_data[i + 6] == 22 && quadrotor->Xbee_data[i + 7] == 102))
          {
            unsigned long temp;
            void* ptr;
            temp = (quadrotor->Xbee_data[i + 5] << 24) | (quadrotor->Xbee_data[i + 4] << 16) | (quadrotor->Xbee_data[i + 3] << 8) | quadrotor->Xbee_data[i + 2];
            ptr = &temp;
            quadrotor->X.x = *(float *) ptr;
          }
        }
        break;
      case 22:
        if (quadrotor->Xbee_data[i + 1] == 102)
        {
          if ((quadrotor->Xbee_data[i - 6] == 21 && quadrotor->Xbee_data[i - 5] == 101) && (quadrotor->Xbee_data[i + 6] == 23 && quadrotor->Xbee_data[i + 7] == 103))
          {
            unsigned long temp;
            void* ptr;
            temp = (quadrotor->Xbee_data[i + 5] << 24) | (quadrotor->Xbee_data[i + 4] << 16) | (quadrotor->Xbee_data[i + 3] << 8) | quadrotor->Xbee_data[i + 2];
            ptr = &temp;
            quadrotor->X.y = *(float *) ptr;
          }
        }
        break;
      case 23:
        if (quadrotor->Xbee_data[i + 1] == 103)
        {
          if ((quadrotor->Xbee_data[i - 6] == 22 && quadrotor->Xbee_data[i - 5] == 102) && (quadrotor->Xbee_data[i + 6] == 24 && quadrotor->Xbee_data[i + 7] == 104))
          {
            unsigned long temp;
            void* ptr;
            temp = (quadrotor->Xbee_data[i + 5] << 24) | (quadrotor->Xbee_data[i + 4] << 16) | (quadrotor->Xbee_data[i + 3] << 8) | quadrotor->Xbee_data[i + 2];
            ptr = &temp;
            quadrotor->X.z = *(float *) ptr;
          }
        }
        break;
      case 24:
        if (quadrotor->Xbee_data[i + 1] == 104)
        {
          if ((quadrotor->Xbee_data[i - 6] == 23 && quadrotor->Xbee_data[i - 5] == 103) && (quadrotor->Xbee_data[i + 6] == 25 && quadrotor->Xbee_data[i + 7] == 105))
          {
            unsigned long temp;
            void* ptr;
            temp = (quadrotor->Xbee_data[i + 5] << 24) | (quadrotor->Xbee_data[i + 4] << 16) | (quadrotor->Xbee_data[i + 3] << 8) | quadrotor->Xbee_data[i + 2];
            ptr = &temp;
            quadrotor->X.xdot = *(float *) ptr;
          }
        }
        break;
      case 25:
        if (quadrotor->Xbee_data[i + 1] == 105)
        {
          if ((quadrotor->Xbee_data[i - 6] == 24 && quadrotor->Xbee_data[i - 5] == 104) && (quadrotor->Xbee_data[i + 6] == 26 && quadrotor->Xbee_data[i + 7] == 106))
          {
            unsigned long temp;
            void* ptr;
            temp = (quadrotor->Xbee_data[i + 5] << 24) | (quadrotor->Xbee_data[i + 4] << 16) | (quadrotor->Xbee_data[i + 3] << 8) | quadrotor->Xbee_data[i + 2];
            ptr = &temp;
            quadrotor->X.ydot = *(float *) ptr;
          }
        }
        break;
      case 26:
        if (quadrotor->Xbee_data[i + 1] == 106)
        {
          if ((quadrotor->Xbee_data[i - 6] == 25 && quadrotor->Xbee_data[i - 5] == 105) && (quadrotor->Xbee_data[i + 6] == 27 && quadrotor->Xbee_data[i + 7] == 107))
          {
            unsigned long temp;
            void* ptr;
            temp = (quadrotor->Xbee_data[i + 5] << 24) | (quadrotor->Xbee_data[i + 4] << 16) | (quadrotor->Xbee_data[i + 3] << 8) | quadrotor->Xbee_data[i + 2];
            ptr = &temp;
            quadrotor->X.zdot = *(float *) ptr;
          }
        }
        break;
      case 27:
        if (quadrotor->Xbee_data[i + 1] == 107)
        {
          if ((quadrotor->Xbee_data[i - 6] == 26 && quadrotor->Xbee_data[i - 5] == 106) && (quadrotor->Xbee_data[i + 6] == 28 && quadrotor->Xbee_data[i + 7] == 108))
          {
            unsigned long temp;
            void* ptr;
            temp = (quadrotor->Xbee_data[i + 5] << 24) | (quadrotor->Xbee_data[i + 4] << 16) | (quadrotor->Xbee_data[i + 3] << 8) | quadrotor->Xbee_data[i + 2];
            ptr = &temp;
            HTCViveState.psi = *(float *) ptr;
          }
        }
        break;
      case 28:
        if (quadrotor->Xbee_data[i + 1] == 108)
        {
          if ((quadrotor->Xbee_data[i - 6] == 27 && quadrotor->Xbee_data[i - 5] == 107) && (quadrotor->Xbee_data[i + 6] == 29 && quadrotor->Xbee_data[i + 7] == 109))
          {
            // Pitch
            unsigned long temp;
            void* ptr;
            temp = (quadrotor->Xbee_data[i + 5] << 24) | (quadrotor->Xbee_data[i + 4] << 16) | (quadrotor->Xbee_data[i + 3] << 8) | quadrotor->Xbee_data[i + 2];
            ptr = &temp;
            HTCViveState.theta = *(float *) ptr;
          }
        }
        break;
      case 29:
        if (quadrotor->Xbee_data[i + 1] == 109)
        {
          if ((quadrotor->Xbee_data[i - 6] == 28 && quadrotor->Xbee_data[i - 5] == 108) && (quadrotor->Xbee_data[i + 6] == 30 && quadrotor->Xbee_data[i + 7] == 110))
          {
            // Roll
            unsigned long temp;
            void* ptr;
            temp = (quadrotor->Xbee_data[i + 5] << 24) | (quadrotor->Xbee_data[i + 4] << 16) | (quadrotor->Xbee_data[i + 3] << 8) | quadrotor->Xbee_data[i + 2];
            ptr = &temp;
            HTCViveState.phi = *(float *) ptr;
          }
        }
        break;
      case 31:
        if (quadrotor->Xbee_data[i + 1] == 111)
        {
          if ((quadrotor->Xbee_data[i - 6] == 29 && quadrotor->Xbee_data[i - 5] == 109) && (quadrotor->Xbee_data[i + 6] == 32 && quadrotor->Xbee_data[i + 7] == 112))
          {
            // q0
            unsigned long temp;
            void* ptr;
            temp = (quadrotor->Xbee_data[i + 5] << 24) | (quadrotor->Xbee_data[i + 4] << 16) | (quadrotor->Xbee_data[i + 3] << 8) | quadrotor->Xbee_data[i + 2];
            ptr = &temp;
            quadrotor->X.quaternion.q[0] = *(float *) ptr;
          }
        }
        break;
      case 32:
        if (quadrotor->Xbee_data[i + 1] == 112)
        {
          if ((quadrotor->Xbee_data[i - 6] == 31 && quadrotor->Xbee_data[i - 5] == 111) && (quadrotor->Xbee_data[i + 6] == 33 && quadrotor->Xbee_data[i + 7] == 113))
          {
            // q1
            unsigned long temp;
            void* ptr;
            temp = (quadrotor->Xbee_data[i + 5] << 24) | (quadrotor->Xbee_data[i + 4] << 16) | (quadrotor->Xbee_data[i + 3] << 8) | quadrotor->Xbee_data[i + 2];
            ptr = &temp;
            quadrotor->X.quaternion.q[1] = *(float *) ptr;
          }
        }
        break;
      case 33:
        if (quadrotor->Xbee_data[i + 1] == 113)
        {
          if ((quadrotor->Xbee_data[i - 6] == 32 && quadrotor->Xbee_data[i - 5] == 112) && (quadrotor->Xbee_data[i + 6] == 34 && quadrotor->Xbee_data[i + 7] == 114))
          {
            // q2
            unsigned long temp;
            void* ptr;
            temp = (quadrotor->Xbee_data[i + 5] << 24) | (quadrotor->Xbee_data[i + 4] << 16) | (quadrotor->Xbee_data[i + 3] << 8) | quadrotor->Xbee_data[i + 2];
            ptr = &temp;
            quadrotor->X.quaternion.q[2] = *(float *) ptr;
          }
        }
        break;
      case 34:
        if (quadrotor->Xbee_data[i + 1] == 114)
        {
          if ((quadrotor->Xbee_data[i - 6] == 33 && quadrotor->Xbee_data[i - 5] == 113) && (quadrotor->Xbee_data[i + 6] == 35 && quadrotor->Xbee_data[i + 7] == 115))
          {
            // q3
            unsigned long temp;
            void* ptr;
            temp = (quadrotor->Xbee_data[i + 5] << 24) | (quadrotor->Xbee_data[i + 4] << 16) | (quadrotor->Xbee_data[i + 3] << 8) | quadrotor->Xbee_data[i + 2];
            ptr = &temp;
            quadrotor->X.quaternion.q[3] = *(float *) ptr;
          }
        }
        break;
      case 35:
        if (quadrotor->Xbee_data[i + 1] == 115)
        {
          if ((quadrotor->Xbee_data[i - 6] == 34 && quadrotor->Xbee_data[i - 5] == 114) && (quadrotor->Xbee_data[i + 6] == 36 && quadrotor->Xbee_data[i + 7] == 116))
          {
            unsigned long temp;
            void* ptr;
            temp = (quadrotor->Xbee_data[i + 5] << 24) | (quadrotor->Xbee_data[i + 4] << 16) | (quadrotor->Xbee_data[i + 3] << 8) | quadrotor->Xbee_data[i + 2];
            ptr = &temp;
            // quadrotor->Xdes.x = *(float *) ptr;
          }
        }
        break;
      case 36:
        if (quadrotor->Xbee_data[i + 1] == 116)
        {
          if ((quadrotor->Xbee_data[i - 6] == 35 && quadrotor->Xbee_data[i - 5] == 115) && (quadrotor->Xbee_data[i + 6] == 37 && quadrotor->Xbee_data[i + 7] == 117))
          {
            unsigned long temp;
            void* ptr;
            temp = (quadrotor->Xbee_data[i + 5] << 24) | (quadrotor->Xbee_data[i + 4] << 16) | (quadrotor->Xbee_data[i + 3] << 8) | quadrotor->Xbee_data[i + 2];
            ptr = &temp;
            // quadrotor->Xdes.y = *(float *) ptr;
          }
        }
        break;
      case 37:
        if (quadrotor->Xbee_data[i + 1] == 117)
        {
          if ((quadrotor->Xbee_data[i - 6] == 36 && quadrotor->Xbee_data[i - 5] == 116) && (quadrotor->Xbee_data[i + 6] == 38 && quadrotor->Xbee_data[i + 7] == 118))
          {
            unsigned long temp;
            void* ptr;
            temp = (quadrotor->Xbee_data[i + 5] << 24) | (quadrotor->Xbee_data[i + 4] << 16) | (quadrotor->Xbee_data[i + 3] << 8) | quadrotor->Xbee_data[i + 2];
            ptr = &temp;
            // quadrotor->Xdes.z = *(float *) ptr;
          }
        }
        break;
      case 38:
        if (quadrotor->Xbee_data[i + 1] == 118)
        {
          if ((quadrotor->Xbee_data[i - 6] == 37 && quadrotor->Xbee_data[i - 5] == 117) && (quadrotor->Xbee_data[i + 6] == 30 && quadrotor->Xbee_data[i + 7] == 110))
          {
            unsigned long temp;
            void* ptr;
            temp = (quadrotor->Xbee_data[i + 5] << 24) | (quadrotor->Xbee_data[i + 4] << 16) | (quadrotor->Xbee_data[i + 3] << 8) | quadrotor->Xbee_data[i + 2];
            ptr = &temp;
            // quadrotor->Xdes.psi = *(float *) ptr;
          }
        }
        break;
      case 30:
        if (quadrotor->Xbee_data[i + 1] == 110)
        {
          if ((quadrotor->Xbee_data[i - 6] == 38 && quadrotor->Xbee_data[i - 5] == 118) && (quadrotor->Xbee_data[i + 4] == 50 && quadrotor->Xbee_data[i + 5] == 75 && quadrotor->Xbee_data[i + 6] == 100))
          {
            short command_temp;
            command_temp = (quadrotor->Xbee_data[i + 3] << 8) | quadrotor->Xbee_data[i + 2];
            if (command_temp >= 1000 && command_temp <= 4000)
              quadrotor->Xbee_command = command_temp;
          }
        }
        break;
    }
  }
  // Phi
  if (abs((quadrotor->X.phi * (180/PI)) - HTCViveState.phi) < 5.0)
  {
      HTCViveState.phi = (quadrotor->X.phi * (180/PI)) * 0.75 + HTCViveState.phi * 0.25;
  }
  else
  {
      HTCViveState.phi = (quadrotor->X.phi * (180/PI)) * 0.99 + HTCViveState.phi * 0.01;
  }
  HTCViveState.phi = HTCViveState.phi * (PI/180);
  // Theta
  if (abs((quadrotor->X.theta * (180/PI)) - HTCViveState.theta) < 5.0)
  {
      HTCViveState.theta = (quadrotor->X.theta * (180/PI)) * 0.75 + HTCViveState.theta * 0.25;
  }
  else
  {
      HTCViveState.theta = (quadrotor->X.theta * (180/PI)) * 0.99 + HTCViveState.theta * 0.01;
  }
  HTCViveState.theta = HTCViveState.theta * (PI/180);


  float phi_sin = sin(HTCViveState.phi);
  float theta_sin = sin(HTCViveState.theta);
  float phi_cos = cos(HTCViveState.phi);
  float theta_cos = cos(HTCViveState.theta);
  float pre_cos , pre_sin, ob_cos, ob_sin, inc, cof = 0.70;
  inc = (-phi_sin / theta_cos * (quadrotor->X.q * (180/PI)) + phi_cos / theta_cos * (quadrotor->X.r * (180/PI))) * dtOuter;
  pre_cos = cos(((quadrotor->X.psi * (180/PI)) + inc) * (PI/180));
  pre_sin = sin(((quadrotor->X.psi * (180/PI)) + inc) * (PI/180));
  
  ob_cos = cos((HTCViveState.psi) * (PI/180));
  ob_sin = sin((HTCViveState.psi) * (PI/180));

  pre_cos = pre_cos * cof + ob_cos * (1 - cof);
  pre_sin = pre_sin * cof + ob_sin * (1 - cof);

  quadrotor->X.psi = atan2(pre_sin , pre_cos);
}

// Moving Average to smooth the received data
// Source: Shi Lu, "Modeling, Control and Design of a Quadrotor Platform for Indoor Environments," 2018.
void BaseStation::HTCViveMovingAverage(Quadrotor *quadrotor)
{
  quadrotor->X.x = quadrotor->CONSTRAIN(quadrotor->X.x, -10.0, 10.0);
  quadrotor->X.y = quadrotor->CONSTRAIN(quadrotor->X.y, -10.0, 10.0);
  quadrotor->X.z = quadrotor->CONSTRAIN(quadrotor->X.z, -5.0, 10.0);
  quadrotor->X.xdot = quadrotor->CONSTRAIN(quadrotor->X.xdot, -50.0, 50.0);
  quadrotor->X.ydot = quadrotor->CONSTRAIN(quadrotor->X.ydot, -50.0, 50.0);
  quadrotor->X.zdot = quadrotor->CONSTRAIN(quadrotor->X.zdot, -50.0, 50.0);
  // quadrotor->Xdes.x = quadrotor->CONSTRAIN(quadrotor->Xdes.x, -1.5, 1.5);
  // quadrotor->Xdes.y = quadrotor->CONSTRAIN(quadrotor->Xdes.y, -1.5, 1.5);
  // quadrotor->Xdes.z = quadrotor->CONSTRAIN(quadrotor->Xdes.z, 0, 2.2);
  quadrotor->X.psi = quadrotor->CONSTRAIN(quadrotor->X.psi, -180.0, 180.0);
  // quadrotor->Xdes.psi = quadrotor->CONSTRAIN(quadrotor->Xdes.psi, -180, 180);
  /*------------Moving Average------------*/
  for (uint8_t j = 0; j < 13 ; j++)
  {
    sum_average[j] = sum_average[j] - moving_average[j][4] / 5;
  }
  for (uint8_t i = 4; i > 0; i--)
  {
    for (uint8_t j = 0; j < 13 ; j++)
    {
      moving_average[j][i] = moving_average[j][i - 1];
    }
  }

  moving_average[0][0] = quadrotor->X.x;
  moving_average[1][0] = quadrotor->X.y;
  moving_average[2][0] = quadrotor->X.z;
  moving_average[3][0] = quadrotor->X.xdot;
  moving_average[4][0] = quadrotor->X.ydot;
  moving_average[5][0] = quadrotor->X.zdot;
  moving_average[8][0] = quadrotor->X.psi;

  for (uint8_t j = 0; j < 13 ; j++)
  {
    sum_average[j] = sum_average[j] + moving_average[j][0] / 5;
  }
  quadrotor->X.x = sum_average[0];
  quadrotor->X.y = sum_average[1];
  quadrotor->X.z = sum_average[2];

  quadrotor->X.xdot = sum_average[3];
  quadrotor->X.ydot = sum_average[4];
  quadrotor->X.zdot = sum_average[5];
}

// Complimentary Filter
// Source: Shi Lu, "Modeling, Control and Design of a Quadrotor Platform for Indoor Environments," 2018.
void BaseStation::HTCViveComplementaryFilter(Quadrotor *quadrotor)
{
  HTCViveCounter++;
  if (HTCViveCounter >= 4)
  {
    HTCViveCounter = 0;
    float HTCViveNorm = quadrotor->invSqrt(quadrotor->X.quaternion.q[0] * quadrotor->X.quaternion.q[0] + quadrotor->X.quaternion.q[1] * quadrotor->X.quaternion.q[1] + quadrotor->X.quaternion.q[2] * quadrotor->X.quaternion.q[2] + quadrotor->X.quaternion.q[3] * quadrotor->X.quaternion.q[3]);
    quadrotor->X.quaternion.q[0] *= HTCViveNorm;
    quadrotor->X.quaternion.q[1] *= HTCViveNorm;
    quadrotor->X.quaternion.q[2] *= HTCViveNorm;
    quadrotor->X.quaternion.q[3] *= HTCViveNorm;


    MatrixXf HTCVive_R(3, 3);
    MatrixXf Acc_B(3, 1);
    MatrixXf Acc_I(3, 1);

    float q0_2, q1_2, q2_2, q3_2, q1q2, q0q3, q1q3, q0q2, q2q3, q0q1;

    q0_2 = quadrotor->X.quaternion.q[0] * quadrotor->X.quaternion.q[0];
    q1_2 = quadrotor->X.quaternion.q[1] * quadrotor->X.quaternion.q[1];
    q2_2 = quadrotor->X.quaternion.q[2] * quadrotor->X.quaternion.q[2];
    q3_2 = quadrotor->X.quaternion.q[3] * quadrotor->X.quaternion.q[3];

    q1q2 = quadrotor->X.quaternion.q[1] * quadrotor->X.quaternion.q[2];
    q0q3 = quadrotor->X.quaternion.q[0] * quadrotor->X.quaternion.q[3];
    q1q3 = quadrotor->X.quaternion.q[1] * quadrotor->X.quaternion.q[3];
    q0q2 = quadrotor->X.quaternion.q[0] * quadrotor->X.quaternion.q[2];
    q2q3 = quadrotor->X.quaternion.q[2] * quadrotor->X.quaternion.q[3];
    q0q1 = quadrotor->X.quaternion.q[0] * quadrotor->X.quaternion.q[1];

    HTCVive_R << q0_2 + q1_2 - q2_2 - q3_2, 2 * (q1q2 - q0q3),  2 * (q1q3 + q0q2),
           2 * (q1q2 + q0q3),  q0_2 - q1_2 + q2_2 - q3_2,  2 * (q2q3 - q0q1),
           2 * (q1q3 - q0q2),  2 * (q2q3 + q0q1), q0_2 - q1_2 - q2_2 + q3_2;

    Acc_B << (quadrotor->accel.filtered.x * 0.122/1000) * 9.8 , (quadrotor->accel.filtered.y * 0.122/1000) * 9.8 , (quadrotor->accel.filtered.z * 0.122/1000) * 9.8;
    Acc_I =  HTCVive_R * Acc_B;

    quadrotor->X.xdotdot = Acc_I(0, 0);
    quadrotor->X.ydotdot = -Acc_I(1, 0);
    quadrotor->X.zdotdot = Acc_I(2, 0) - 9.8;


    float CF_a = 0.8, dtOuter_2 = dtOuter * dtOuter;
    quadrotor->X.xdot = (quadrotor->X.xdot + dtOuter * quadrotor->X.xdotdot) * CF_a + quadrotor->X.xdot * (1 - CF_a);
    quadrotor->X.ydot = (quadrotor->X.ydot + dtOuter * quadrotor->X.ydotdot) * CF_a + quadrotor->X.ydot * (1 - CF_a);
    quadrotor->X.zdot = (quadrotor->X.zdot + dtOuter * quadrotor->X.zdotdot) * CF_a + quadrotor->X.zdot * (1 - CF_a);

    quadrotor->X.x = (quadrotor->X.x + dtOuter * quadrotor->X.xdot + 0.5 * dtOuter_2 * quadrotor->X.xdotdot) * CF_a + quadrotor->X.x * (1 - CF_a);
    quadrotor->X.y = (quadrotor->X.y + dtOuter * quadrotor->X.ydot + 0.5 * dtOuter_2 * quadrotor->X.ydotdot) * CF_a + quadrotor->X.y * (1 - CF_a);
    quadrotor->X.z = (quadrotor->X.z + dtOuter * quadrotor->X.zdot + 0.5 * dtOuter_2 * quadrotor->X.zdotdot) * CF_a + quadrotor->X.z * (1 - CF_a);
  }
}
