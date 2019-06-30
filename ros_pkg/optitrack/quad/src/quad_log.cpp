#include "ros/ros.h"
#include <fstream>
#include "quad/quad_state_msg.h"
#include "quad/quad_cmd_msg.h"
#include "sensor_msgs/Imu.h"

using namespace std;
ofstream datafile;
float p, q, r;

class quad_log
{
public:
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Subscriber sub2;
  ros::Subscriber sub3;
  ros::Subscriber sub4;
  quad::quad_state_msg desired;
  quad::quad_state_msg actual;
  quad::quad_cmd_msg cmd;

  quad_log()
  {
    sub = n.subscribe<quad::quad_state_msg>("/quad_trajectory", 1000, &quad_log::callback, this);
    sub2 = n.subscribe<quad::quad_state_msg>("/quad_state", 1000, &quad_log::callback2, this);
    sub3 = n.subscribe<quad::quad_cmd_msg>("/quad_cmd", 1000, &quad_log::callback3, this);
    sub4 = n.subscribe<sensor_msgs::Imu>("/quad_serial_imu", 1000, &quad_log::callback4, this);
  }

  ~quad_log()
  {
    // Empty
  }

  void callback(const quad::quad_state_msg::ConstPtr& msg)
  {
    desired.pose.position.x = msg->pose.position.x;
    desired.pose.position.y = msg->pose.position.y;
    desired.pose.position.z = msg->pose.position.z;
    desired.velocity.linear.x =  msg->velocity.linear.x;
    desired.velocity.linear.y =  msg->velocity.linear.y;
    desired.velocity.linear.z =  msg->velocity.linear.z;
    desired.acceleration.linear.x = msg->acceleration.linear.x;
    desired.acceleration.linear.y = msg->acceleration.linear.y;
    desired.acceleration.linear.z = msg->acceleration.linear.z;
    desired.attitude.z = msg->attitude.z;
    desired.mode = msg->mode;
  }

  void callback2(const quad::quad_state_msg::ConstPtr& msg)
  {
    actual.pose.position.x = msg->pose.position.x;
    actual.pose.position.y = msg->pose.position.y;
    actual.pose.position.z = msg->pose.position.z;
    actual.velocity.linear.x =  msg->velocity.linear.x;
    actual.velocity.linear.y =  msg->velocity.linear.y;
    actual.velocity.linear.z =  msg->velocity.linear.z;
    actual.acceleration.linear.x = msg->acceleration.linear.x;
    actual.acceleration.linear.y = msg->acceleration.linear.y;
    actual.acceleration.linear.z = msg->acceleration.linear.z;
    actual.attitude.z = msg->attitude.z;
  }

  void callback3(const quad::quad_cmd_msg::ConstPtr& msg)
  {
    cmd.phi_des = msg->phi_des;
    cmd.theta_des = msg->theta_des;
    cmd.r_des = msg->r_des;
  }

  void callback4(const sensor_msgs::Imu::ConstPtr& msg)
  {
    actual.attitude.x = msg->orientation.x; // Roll angle
    actual.attitude.y = msg->orientation.y; // Pitch angle
    p = msg->angular_velocity.x;
    q = msg->angular_velocity.y;
    r = msg->angular_velocity.z;
  }

  void writeData(void)
  {
    datafile << desired.mode << ", " << desired.pose.position.x << ", " << desired.pose.position.y
     << ", " << desired.pose.position.z << ", " << desired.velocity.linear.x
     << ", " << desired.velocity.linear.y << ", " << desired.velocity.linear.z
     << ", " << desired.acceleration.linear.x << ", " << desired.acceleration.linear.y
     << ", " << desired.acceleration.linear.z << ", " << cmd.phi_des << ", " << cmd.theta_des
     << ", " << desired.attitude.z << ", " << actual.pose.position.x
     << ", " << actual.pose.position.y << ", " << actual.pose.position.z
     << ", " << actual.velocity.linear.x << ", " << actual.velocity.linear.y
     << ", " << actual.velocity.linear.z << ", " << actual.acceleration.linear.x
     << ", " << actual.acceleration.linear.y << ", " << actual.acceleration.linear.z
     << ", " << actual.attitude.x << ", " << actual.attitude.y << ", " << actual.attitude.z
     << ", " << p << ", " << q << ", " << r << endl;
  }
};

int main(int argc, char **argv)
{
  datafile.open("/home/altwaitan/catkin_ws/src/quad/vehicle1_data.txt");
  ros::init(argc, argv, "quad_log");
  quad_log quad;

  ros::Rate loop_rate(100);


  if (datafile.is_open())
  {
    datafile << "mode" << ", " << "xdes (m)" << ", " << "ydes (m)" << ", " << "zdes (m)" << ", "
    << "vxdes (m/s)" << ", " << "vydes (m/s)" << ", " << "vzdes (m/s)" << ", " << "axdes (m/s^2)" << ", "
    << "aydes (m/s^2)" << ", " << "azdes (m/s^2)" << ", " << "phides (rad)" << ", " << "thetades (rad)"
    << ", " << "psides (rad)" << ", " << "x (m)" << ", " << "y (m)" << ", " << "z (m)" << ", " << "vx (m/s)"
    << ", " << "vy (m/s)" << ", " << "vz (m/s)" << ", " << "ax (m/s^2)" << ", " << "ay (m/s^2)" << ", "
    << "az (m/s^2)" << ", " << "phi (rad)" << ", " << "theta (rad)" << ", " << "psi (rad)" << ", "
    << "p (rad/s)" << ", " << "q (rad/s)" << ", " << "r (rad/s)" << endl;
    while (ros::ok())
    {
      quad.writeData();
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  else
  {
    ROS_ERROR_STREAM("Unable to open datafile");
  }
  datafile.close();
  ros::shutdown();
  return 0;
}
