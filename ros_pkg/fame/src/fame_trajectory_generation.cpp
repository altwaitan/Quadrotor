// Author: Abdullah Altawaitan
// Date: April 4, 2019

#include "ros/ros.h"
#include "fame/fame_state.h"
#include "std_msgs/UInt8.h"

double k = 0;
int mode = 0;

class fame_trajectory_generation
{
public:
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Publisher pub;
  fame::fame_state state;

  fame_trajectory_generation()
  {
    sub = n.subscribe<std_msgs::UInt8>("/fame_gui", 1000, &fame_trajectory_generation::callback, this);
    pub = n.advertise<fame::fame_state>("fame_trajectory", 1000);
  }

  ~fame_trajectory_generation()
  {
    // Empty
  }

  void callback(const std_msgs::UInt8::ConstPtr& msg)
  {
    mode = msg->data;
  }

  void publishGenerateTrajectory(void)
  {
    // Trajectory Generation
    if (mode != 0)
    {
      if (mode == 1)
      {
        state.pose.position.x = 0;
        state.pose.position.y = 0;
        state.pose.position.z = 1;
        state.velocity.linear.x = 0;
        state.velocity.linear.y = 0;
        state.velocity.linear.z = 0;
        state.acceleration.linear.x = 0;
        state.acceleration.linear.y = 0;
        state.acceleration.linear.z = 0;
        state.attitude.z = 0;
        state.velocity.angular.z = 0;
      }
      else if (mode == 2)
      {
        state.pose.position.x = 0.5 * sin(k * 0.6);
        state.pose.position.y = 0.5 * cos(k * 0.6);
        state.velocity.linear.x = 0.5 * (0.6) * cos(k * 0.6);
        state.velocity.linear.y = -0.5 * (0.6) * sin(k * 0.6);
        state.acceleration.linear.x = -0.5 * (0.6*0.6) * sin(k * 0.6);
        state.acceleration.linear.y = -0.5 * (0.6*0.6) * cos(k * 0.6);
        state.pose.position.z = 1;
        state.velocity.linear.z = 0;
        state.acceleration.linear.z = 0;
        state.attitude.z = 0;
        state.velocity.angular.z = 0;
        // Increment
        k = k + 0.01;
      }
    }
    else
    {
      state.pose.position.x = 0;
      state.pose.position.y = 0;
      state.pose.position.z = 0;
      state.velocity.linear.x = 0;
      state.velocity.linear.y = 0;
      state.velocity.linear.z = 0;
      state.acceleration.linear.x = 0;
      state.acceleration.linear.y = 0;
      state.acceleration.linear.z = 0;
      state.attitude.z = 0;
      state.velocity.angular.z = 0;
    }
    pub.publish(state);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fame_trajectory_generation");
  fame_trajectory_generation fame;

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    fame.publishGenerateTrajectory();
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::shutdown();
  return 0;
}
