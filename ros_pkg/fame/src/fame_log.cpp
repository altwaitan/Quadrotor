#include "ros/ros.h"
#include <fstream>
#include "fame/fame_state.h"
#include "fame/fame_msg.h"

using namespace std;
ofstream datafile;

class fame_log
{
public:
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Subscriber sub2;
  ros::Subscriber sub3;
  fame::fame_state desired;
  fame::fame_state actual;
  fame::fame_msg cmd;

  fame_log()
  {
    sub = n.subscribe<fame::fame_state>("/fame_trajectory", 1000, &fame_log::callback, this);
    sub2 = n.subscribe<fame::fame_state>("/fame_state", 1000, &fame_log::callback2, this);
    sub3 = n.subscribe<fame::fame_msg>("/fame_cmd", 1000, &fame_log::callback3, this);
  }

  ~fame_log()
  {
    // Empty
  }

  void callback(const fame::fame_state::ConstPtr& msg)
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
  }

  void callback2(const fame::fame_state::ConstPtr& msg)
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
    actual.attitude.x = msg->attitude.x;
    actual.attitude.y = msg->attitude.y;
    actual.attitude.z = msg->attitude.z;
  }

  void callback3(const fame::fame_msg::ConstPtr& msg)
  {
    cmd.phi_des = msg->phi_des;
    cmd.theta_des = msg->theta_des;
    cmd.r_des = msg->r_des;
  }

  void writeData(void)
  {
    datafile << desired.pose.position.x << ", " << desired.pose.position.y
     << ", " << desired.pose.position.z << ", " << desired.velocity.linear.x
     << ", " << desired.velocity.linear.y << ", " << desired.velocity.linear.z
     << ", " << cmd.phi_des << ", " << cmd.theta_des << ", "
     << cmd.r_des << ", " << actual.pose.position.x
     << ", " << actual.pose.position.y << ", " << actual.pose.position.z
     << ", " << actual.velocity.linear.x << ", " << actual.velocity.linear.y
     << ", " << actual.velocity.linear.z << ", " << actual.attitude.x << ", "
     << actual.attitude.y << ", " << actual.attitude.z << endl;
  }
};

int main(int argc, char **argv)
{
  datafile.open("/home/smanne1/catkin_ws/src/fame/data.txt");
  ros::init(argc, argv, "fame_log");
  fame_log fame;

  ros::Rate loop_rate(100);


  if (datafile.is_open())
  {
    datafile << "xdes" << ", " << "ydes" << ", " << "zdes" << ", "
    << "vxdes" << ", " << "vydes" << ", " << "vzdes" << ", "
    << "phides" << ", " << "thetades" << ", " << "psides" << ", "
    << "x" << ", " << "y" << ", " << "z" << ", " << "vx" << ", "
    << "vy" << ", " << "vz" << ", " << "phi" << ", " << "theta"
    << ", " << "psi" << endl;
    while (ros::ok())
    {
      fame.writeData();
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
