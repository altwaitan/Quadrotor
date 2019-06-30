// Author: Abdullah Altawaitan
// Date: April 4, 2019

#include "ros/ros.h"
#include "quad/quad_state_msg.h"
#include "std_msgs/Int8.h"

double i = 0;
double k = 0;
double j = 0;
double l = 0;
int mode = 0;
double w = 0.8;
float counter = 0;
float counter2 = 0;
float counter3 = 0;
class quad_trajectory_generation
{
public:
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Publisher pub;
  quad::quad_state_msg state;

  quad_trajectory_generation()
  {
    sub = n.subscribe<std_msgs::Int8>("/quad_gui", 1000, &quad_trajectory_generation::callback, this);
    pub = n.advertise<quad::quad_state_msg>("quad_trajectory", 1000);
  }

  ~quad_trajectory_generation()
  {
    // Empty
  }

  void callback(const std_msgs::Int8::ConstPtr& msg)
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
        if (counter > 5)
        {
          state.pose.position.x = 0;
          state.pose.position.y = 0;
          state.pose.position.z = i;
          state.pose.position.z = CONSTRAIN(state.pose.position.z, 0, 1.0);
          state.velocity.linear.x = 0;
          state.velocity.linear.y = 0;
          state.velocity.linear.z = 0;
          state.acceleration.linear.x = 0;
          state.acceleration.linear.y = 0;
          state.acceleration.linear.z = 0;
          state.attitude.z = 0;
          state.velocity.angular.z = 0;
          state.mode = 1;
          // Increment
          i = i + 0.005;

          // Counter2
          if (counter2 > 10)
          {
            mode = 2;
            counter3 = 0;
            k = 0;
          }
          else
          {
            counter2 = counter2 + 0.01;
          }
        }
        else
        {
          counter = counter + 0.01;
        }
      }
      else if (mode == 2)
      {
        // Counter3
        if (counter3 > 10)
        {
          state.pose.position.x = 3.0 * sin(w * k);
          state.pose.position.y = 3.0 * cos(w * k);
          state.velocity.linear.x = 3.0 * w * cos(w * k);
          state.velocity.linear.y = -3.0 * w * sin(w * k);
          state.acceleration.linear.x = -3.0 * (w * w) * sin(w * k);
          state.acceleration.linear.y = -3.0 * (w * w) * cos(w * k);
          state.pose.position.z = 1.0;
          state.velocity.linear.z = 0.0;
          state.acceleration.linear.z = 0.0;
          state.attitude.z = 0;
          state.velocity.angular.z = 0;
          state.mode = 2;
          // Increment
          k = k + 0.01;
        }
        else
        {
          counter3 = counter3 + 0.01;
          state.pose.position.x = 0.0;
          state.pose.position.y = 0.5 * l;
          state.pose.position.y = CONSTRAIN(state.pose.position.y, 0, 3.0);
          state.velocity.linear.x = 0;
          state.velocity.linear.y = 0;
          state.acceleration.linear.x = 0;
          state.acceleration.linear.y = 0;
          state.pose.position.z = 1.0;
          state.velocity.linear.z = 0.0;
          state.acceleration.linear.z = 0.0;
          state.attitude.z = 0;
          state.velocity.angular.z = 0;
          state.mode = 2;
          // Increment
          l = l + 0.01;
        }

      }
      else if (mode == 3)
      {
        state.pose.position.x = 0;
        state.pose.position.y = 0;
        state.pose.position.z = 1.0 - j*0.2;
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
        if (state.pose.position.z < 0.02)
        {
          state.mode = -1;
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
        state.mode = -1;
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

  double CONSTRAIN(double x, double min, double max)
  {
    if (x > max)
    {
      x = max;
    }
    else if (x < min)
    {
      x = min;
    }
    else
    {
      x = x;
    }
    return x;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "quad_trajectory_generation");
  quad_trajectory_generation quad;

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    quad.publishGenerateTrajectory();
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::shutdown();
  return 0;
}
