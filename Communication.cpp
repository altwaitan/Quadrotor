#include "Communication.h"

void Communication::ROS_Send(Quadrotor *quadrotor)
{
    mavlink_message_t mav_msg;
    uint8_t len;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint32_t time_mav = micros();
    mavlink_msg_raw_imu_pack(1, 1, &mav_msg, time_mav,
      quadrotor->accel.filtered.x*1000, quadrotor->accel.filtered.y*1000, quadrotor->accel.filtered.z*1000,
      quadrotor->X.p*1000, quadrotor->X.q*1000, quadrotor->X.r*1000, quadrotor->X.phi*1000, quadrotor->X.theta*1000, quadrotor->voltage*1000);
    len = mavlink_msg_to_send_buffer(buf, &mav_msg);
    Serial1.write(buf, len);
}

void Communication::ROS_Receive(Quadrotor *quadrotor)
{
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
          CommunicationState.thrust = des.thrust;
          CommunicationState.phi = des.roll_speed; // We are sending roll angle (rad) NOT body_roll_rate
          CommunicationState.theta = des.pitch_speed; // We are sending pitch angle (rad) NOT body_pitch_rate
          CommunicationState.r = des.yaw_speed;
          CommunicationState.mode = des.time_boot_ms;
          quadrotor->flight_mode = CommunicationState.mode;
          Serial.println(CommunicationState.thrust);

          if (quadrotor->channel.CH5 > 1600)
          {
            quadrotor->U1des.current = CommunicationState.thrust;
            quadrotor->Xdes.phi = CommunicationState.phi;
            quadrotor->Xdes.theta = CommunicationState.theta;
            quadrotor->Xdes.r = CommunicationState.r;

            quadrotor->U1des.current = quadrotor->CONSTRAIN(quadrotor->U1des.current, 0, 15);
            quadrotor->Xdes.phi = quadrotor->CONSTRAIN(quadrotor->Xdes.phi, -1.0, 1.0);
            quadrotor->Xdes.theta = quadrotor->CONSTRAIN(quadrotor->Xdes.theta, -1.0, 1.0);
            quadrotor->Xdes.r = quadrotor->CONSTRAIN(quadrotor->Xdes.r, -2, 2);

          }
      }
    }
  }
}
