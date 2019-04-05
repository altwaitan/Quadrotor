#include "BaseStation.h"

// HTC Vive Base Station
// Receives position and orientation of an object through Xbee
// Source: Shi Lu, "Modeling, Control and Design of a Quadrotor Platform for Indoor Environments," 2018.
void BaseStation::HTCVive(Quadrotor *quadrotor)
{
    // RECEVING a HEARTBEAT ----------------------------------------
    // mavlink_message_t msg;
    // mavlink_status_t status;
    //
    // while (Serial1.available())
    // {
    //   uint8_t c = Serial1.read();
    //   if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    //   {
    //     Serial.print("DEBUG msgid:");
    //     Serial.println(msg.msgid);
    //     switch (msg.msgid)
    //     {
    //       case MAVLINK_MSG_ID_HEARTBEAT:
    //         mavlink_heartbeat_t hb;
    //         mavlink_msg_heartbeat_decode(&msg, &hb);
    //         Serial.print(millis());
    //         Serial.print("\ncustom_mode: "); Serial.println(hb.custom_mode);
    //         Serial.print("Type: "); Serial.println(hb.type);
    //         Serial.print("autopilot: "); Serial.println(hb.autopilot);
    //         Serial.print("mavlink_version: "); Serial.println(hb.mavlink_version);
    //         Serial.println();
    //       break;
    //     }
    //   }
    // }
    // -------------------------------------------------------------
    // mavlink_message_t mav_msg;
    // uint8_t len;
    // uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    // uint32_t time_mav = millis();
    // mavlink_msg_attitude_pack(1, 1, &mav_msg, time_mav, quadrotor->X.phi, quadrotor->X.theta, quadrotor->X.psi, quadrotor->X.p, quadrotor->X.q, quadrotor->X.r);
    // len = mavlink_msg_to_send_buffer(buf, &mav_msg);
    // Serial1.write(buf, len);

    mavlink_message_t mav_msg;
    uint8_t len;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint32_t time_mav = micros();
    mavlink_msg_raw_imu_pack(1, 1, &mav_msg, time_mav,
      quadrotor->accel.filtered.x, quadrotor->accel.filtered.y, quadrotor->accel.filtered.z,
      quadrotor->X.p, quadrotor->X.q, quadrotor->X.r, 0.0, 0.0, 0.0);
    len = mavlink_msg_to_send_buffer(buf, &mav_msg);
    Serial1.write(buf, len);

    mavlink_message_t msg;
    mavlink_status_t status;

    while (Serial1.available())
    {
      uint8_t c = Serial1.read();
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
      {
        switch (msg.msgid)
        {
          case MAVLINK_MSG_ID_HEARTBEAT:
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);
            Serial.print(millis());
            Serial.print("\ncustom_mode: "); Serial.println(hb.custom_mode);
            Serial.print("Type: "); Serial.println(hb.type);
            Serial.print("autopilot: "); Serial.println(hb.autopilot);
            Serial.print("mavlink_version: "); Serial.println(hb.mavlink_version);
            Serial.println();
            break;
          case MAVLINK_MSG_ID_ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT:
            mavlink_roll_pitch_yaw_speed_thrust_setpoint_t des;
            mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_decode(&msg, &des);
            quadrotor->U1.current = des.thrust;
            quadrotor->Xdes.phi = des.roll_speed; // We are sending roll angle (rad) NOT body_roll_rate
            quadrotor->Xdes.theta = des.pitch_speed; // We are sending pitch angle (rad) NOT body_pitch_rate
            quadrotor->Xdes.r = des.yaw_speed;
            Serial.print(quadrotor->U1.current);
            Serial.print(", ");
            Serial.print(quadrotor->Xdes.phi);
            Serial.print(", ");
            Serial.print(quadrotor->Xdes.theta);
            Serial.print(", ");
            Serial.println(quadrotor->Xdes.r);
        }
      }
    }
}

// Process the received data and assign them to each Quadrotor state variable.
// Source: Shi Lu, "Modeling, Control and Design of a Quadrotor Platform for Indoor Environments," 2018.

void BaseStation::HTCViveProcess(Quadrotor *quadrotor)
{
  for (int16_t i = 100 + 3; i < 100 + quadrotor->Xbee_length - 2; i++)
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
  quadrotor->Xdes.psi = quadrotor->CONSTRAIN(quadrotor->Xdes.psi, -PI, PI);
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
