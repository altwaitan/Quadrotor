#include "ros/ros.h"
#include <serial/serial.h>
#include "sensor_msgs/Imu.h"
#include "fame/fame_msg.h"
#include "mavlink/mavlink.h"

serial::Serial ser;

class fame_serial
{
public:
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Publisher pub;
  sensor_msgs::Imu imu;
  fame::fame_msg famemsg;

  fame_serial()
  {
    sub = n.subscribe<fame::fame_msg>("/fame_command", 1000, &fame_serial::callback, this);
    pub = n.advertise<sensor_msgs::Imu>("fame_serial_imu", 1000);
  }

  ~fame_serial()
  {
    // Empty
  }

  void callback(const fame::fame_msg::ConstPtr& msg)
  {
    // Serial write
    mavlink_message_t mav_msg2;
    uint8_t len;
    uint8_t buf2[MAVLINK_MAX_PACKET_LEN];
    ros::Time begin = ros::Time::now();
    mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_pack(255, 1, &mav_msg2,
    						       0.0, msg->phi_des, msg->theta_des, msg->r_des, msg->thrust_des);
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
              imu.linear_acceleration.x = mav_imu.xacc;
              imu.linear_acceleration.y = mav_imu.yacc;
              imu.linear_acceleration.z = mav_imu.zacc;
              imu.angular_velocity.x = mav_imu.xgyro;
              imu.angular_velocity.y = mav_imu.ygyro;
              imu.angular_velocity.z = mav_imu.zgyro;
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
  ros::init(argc, argv, "fame_serial_node");
  fame_serial fame;

  try
  {
      ser.setPort("/dev/ttyUSB0");
      ser.setBaudrate(230400);
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

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    fame.serialRead();
    ros::spinOnce();

  }

  ros::shutdown();
  return 0;
}
