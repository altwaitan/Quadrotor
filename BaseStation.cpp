#include "BaseStation.h"

// HTC Vive Base Station
// Receives position and orientation of an object through Xbee
// Source: Shi Lu, "Modeling, Control and Design of a Quadrotor Platform for Indoor Environments," 2018.
void BaseStation::HTCVive(Quadrotor *quadrotor)
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
            quadrotor->X.psi = *(float *) ptr;
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
            quadrotor->Xdes.x = *(float *) ptr;
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
            quadrotor->Xdes.y = *(float *) ptr;
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
            quadrotor->Xdes.z = *(float *) ptr;
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
            quadrotor->Xdes.psi = *(float *) ptr;
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
  quadrotor->Xdes.x = quadrotor->CONSTRAIN(quadrotor->Xdes.x, -1.5, 1.5);
  quadrotor->Xdes.y = quadrotor->CONSTRAIN(quadrotor->Xdes.y, -1.5, 1.5);
  quadrotor->Xdes.z = quadrotor->CONSTRAIN(quadrotor->Xdes.z, 0, 2.2);
  quadrotor->X.psi = quadrotor->CONSTRAIN(quadrotor->X.psi, -180.0, 180.0);
  quadrotor->Xdes.psi = quadrotor->CONSTRAIN(quadrotor->Xdes.psi, -180, 180);
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
