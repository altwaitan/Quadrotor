// Author: Abdullah Altawaitan
// Date: April 4, 2019

#include "ros/ros.h"
#include <serial/serial.h>
#include "sensor_msgs/Imu.h"
#include "quad/quad_cmd_msg.h"
#include "mavlink/mavlink.h"

serial::Serial ser;

class quad_serial
{
public:
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Publisher pub;
  sensor_msgs::Imu imu;
  quad::quad_cmd_msg quad_cmd;

  quad_serial()
  {
    sub = n.subscribe<quad::quad_cmd_msg>("/quad_cmd", 1000, &quad_serial::callback, this);
    pub = n.advertise<sensor_msgs::Imu>("quad_serial_imu", 1000);
  }

  ~quad_serial()
  {
    // Empty
  }

  void callback(const quad::quad_cmd_msg::ConstPtr& msg)
  {
    // Serial write
    quad_cmd.mode = msg->mode;
    quad_cmd.phi_des = msg->phi_des;
    quad_cmd.theta_des = msg->theta_des;
    quad_cmd.r_des = msg->r_des;
    quad_cmd.thrust_des = msg->thrust_des;
    mavlink_message_t mav_msg2;
    uint8_t len;
    uint8_t buf2[MAVLINK_MAX_PACKET_LEN];
    ros::Time begin = ros::Time::now();
    mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_pack(255, 1, &mav_msg2,
    						       quad_cmd.mode, quad_cmd.phi_des, quad_cmd.theta_des, quad_cmd.r_des, quad_cmd.thrust_des);
    len = mavlink_msg_to_send_buffer(buf2, &mav_msg2);
    ser.write(buf2, len);
  }

  void serialRead(void)
  {
    // Serial read
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    size_t length = ser.read(buf, sizeof(buf));
    if (length > 0)
    {
      mavlink_message_t mav_msg;
    	mavlink_status_t status;

      for (int i = 0; i < length; i++)
      {
        if(mavlink_parse_char(MAVLINK_COMM_0, buf[i], &mav_msg, &status))
        {
          switch (mav_msg.msgid)
          {
            case MAVLINK_MSG_ID_HEARTBEAT:
              mavlink_heartbeat_t hb;
              mavlink_msg_heartbeat_decode(&mav_msg, &hb);
              break;
            case MAVLINK_MSG_ID_RAW_IMU:
              mavlink_raw_imu_t mav_imu;
              mavlink_msg_raw_imu_decode(&mav_msg, &mav_imu);
              imu.linear_acceleration.x = ((float)mav_imu.xacc)/1000.00;
              imu.linear_acceleration.y = ((float)mav_imu.yacc)/1000.00;
              imu.linear_acceleration.z = ((float)mav_imu.zacc)/1000.00;
              imu.angular_velocity.x = ((float)mav_imu.xgyro)/1000.00;
              imu.angular_velocity.y = ((float)mav_imu.ygyro)/1000.00;
              imu.angular_velocity.z = ((float)mav_imu.zgyro)/1000.00;
              imu.orientation.x = ((float)mav_imu.xmag)/1000.00; // Roll angle
              imu.orientation.y = ((float)mav_imu.ymag)/1000.00; // Pitch angle
              break;
          }
        }
      }
    }
    pub.publish(imu);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "quad_serial_node");
  quad_serial quad;

  try
  {
      ser.setPort("/dev/ttyUSB0");
      ser.setBaudrate(115200);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser.setTimeout(to);
      ser.open();
  }
  catch (serial::IOException& e)
  {
      ROS_ERROR_STREAM("Unable to open port ");
      return -1;
  }

  if (ser.isOpen())
  {
      ROS_INFO_STREAM("Serial Port initialized");
  }
  else
  {
      return -1;
  }

  ros::Rate loop_rate(50);

  while (ros::ok())
  {
    quad.serialRead();
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::shutdown();
  return 0;
}
