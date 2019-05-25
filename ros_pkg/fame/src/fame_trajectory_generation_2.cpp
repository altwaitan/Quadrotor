// Author: Abdullah Altawaitan
// Date: April 4, 2019

#include "ros/ros.h"
#include "fame/fame_state.h"
#include "std_msgs/UInt8.h"

double k = 0;
double j = 0;
int mode = 0;
double d = 1.0;

class fame_trajectory_generation
{
public:
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Subscriber sub2;
  ros::Publisher pub;
  fame::fame_state state;
  fame::fame_state vehicle1;

  fame_trajectory_generation()
  {
    sub = n.subscribe<std_msgs::UInt8>("/fame_gui", 1000, &fame_trajectory_generation::callback, this);
    sub2 = n.subscribe<fame::fame_state>("/fame_state", 1000, &fame_trajectory_generation::callback2, this);
    pub = n.advertise<fame::fame_state>("fame_trajectory_2", 1000);
  }

  ~fame_trajectory_generation()
  {
    // Empty
  }

  void callback(const std_msgs::UInt8::ConstPtr& msg)
  {
    mode = msg->data;
  }

  void callback2(const fame::fame_state::ConstPtr& msg)
  {
    vehicle1.pose.position.x = msg->pose.position.x;
    vehicle1.pose.position.y = msg->pose.position.y;
    vehicle1.pose.position.z = msg->pose.position.z;
    vehicle1.velocity.linear.x = msg->velocity.linear.x;
    vehicle1.velocity.linear.y = msg->velocity.linear.y;
    vehicle1.velocity.linear.z = msg->velocity.linear.z;
    vehicle1.acceleration.linear.x = msg->acceleration.linear.x;
    vehicle1.acceleration.linear.y = msg->acceleration.linear.y;
    vehicle1.acceleration.linear.z = msg->acceleration.linear.z;
    vehicle1.attitude.z = msg->attitude.z;
    vehicle1.velocity.angular.z = msg->velocity.angular.z;
  }

  void publishGenerateTrajectory(void)
  {
    // Trajectory Generation
    if (mode != 0)
    {
      if (mode == 1)
      {
        state.pose.position.x = 0;
        state.pose.position.y = -d;
        state.pose.position.z = 1;
        state.velocity.linear.x = 0;
        state.velocity.linear.y = 0;
        state.velocity.linear.z = 0;
        state.acceleration.linear.x = 0;
        state.acceleration.linear.y = 0;
        state.acceleration.linear.z = 0;
        state.attitude.z = 0;
        state.velocity.angular.z = 0;
        state.mode = 1;
      }
      else if (mode == 2)
      {
        state.pose.position.x = vehicle1.pose.position.x;
        state.pose.position.y = vehicle1.pose.position.y - d;
        state.pose.position.z = vehicle1.pose.position.z;
        state.velocity.linear.x = vehicle1.velocity.linear.x;
        state.velocity.linear.y = vehicle1.velocity.linear.y;
        state.velocity.linear.z = 0;
        state.acceleration.linear.x = vehicle1.acceleration.linear.x;
        state.acceleration.linear.y = vehicle1.acceleration.linear.y;
        state.acceleration.linear.z = 0;
        state.attitude.z = vehicle1.attitude.z;
        state.velocity.angular.z = 0;
        state.mode = 2;
        // Increment
        k = k + 0.01;
      }
      else if (mode == 3)
      {
        state.pose.position.x = 0;
        state.pose.position.y = -d;
        state.pose.position.z = 1 - j*0.2;
        state.velocity.linear.x = 0;
        state.velocity.linear.y = 0;
        state.velocity.linear.z = 0;
        state.acceleration.linear.x = 0;
        state.acceleration.linear.y = 0;
        state.acceleration.linear.z = 0;
        state.attitude.z = 0;
        state.velocity.angular.z = 0;
        state.mode = 3;
        // Increment
        if (state.pose.position.z > 0)
        {
          j = j + 0.01;
        }
        if (state.pose.position.z < 0.75)
        {
          state.mode = -1;
        }
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
      state.mode = 0;
    }
    pub.publish(state);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fame_trajectory_generation_2");
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
