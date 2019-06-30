// Author: Abdullah Altawaitan
// Date: April 4, 2019

#include "ros/ros.h"
#include "quad/quad_state_msg.h"
#include "std_msgs/Int8.h"

double k = 0;
double j = 0;
int mode = 0;
double d = 1.2;

const int RunningAverageCount_x = 30;
float RunningAverageBuffer_x[RunningAverageCount_x];
int NextRunningAverage_x;

const int RunningAverageCount_y = 30;
float RunningAverageBuffer_y[RunningAverageCount_y];
int NextRunningAverage_y;

const int RunningAverageCount_z = 30;
float RunningAverageBuffer_z[RunningAverageCount_z];
int NextRunningAverage_z;

const int RunningAverageCount_vx = 30;
float RunningAverageBuffer_vx[RunningAverageCount_vx];
int NextRunningAverage_vx;

const int RunningAverageCount_vy = 30;
float RunningAverageBuffer_vy[RunningAverageCount_vy];
int NextRunningAverage_vy;

const int RunningAverageCount_vz = 30;
float RunningAverageBuffer_vz[RunningAverageCount_vz];
int NextRunningAverage_vz;

const int RunningAverageCount_ax = 30;
float RunningAverageBuffer_ax[RunningAverageCount_ax];
int NextRunningAverage_ax;

const int RunningAverageCount_ay = 30;
float RunningAverageBuffer_ay[RunningAverageCount_ay];
int NextRunningAverage_ay;

const int RunningAverageCount_az = 30;
float RunningAverageBuffer_az[RunningAverageCount_az];
int NextRunningAverage_az;

double x_initial = 0;
double y_initial = 0;
int initial = 0;

class quad_trajectory_generation
{
public:
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Subscriber sub2;
  ros::Subscriber sub3;
  ros::Publisher pub;
  quad::quad_state_msg state;
  quad::quad_state_msg vehicle1;
  quad::quad_state_msg raw;

  quad_trajectory_generation()
  {
    sub = n.subscribe<std_msgs::Int8>("/quad_gui", 1000, &quad_trajectory_generation::callback, this);
    sub2 = n.subscribe<quad::quad_state_msg>("/quad_state", 1000, &quad_trajectory_generation::callback2, this);
    sub3 = n.subscribe<quad::quad_state_msg>("/quad_state_2", 1000, &quad_trajectory_generation::callback3, this);
    pub = n.advertise<quad::quad_state_msg>("quad_trajectory_2", 1000);
  }

  ~quad_trajectory_generation()
  {
    // Empty
  }

  void callback(const std_msgs::Int8::ConstPtr& msg)
  {
    mode = msg->data;
  }

  void callback2(const quad::quad_state_msg::ConstPtr& msg)
  {
    raw.pose.position.x = msg->pose.position.x;
    raw.pose.position.y = msg->pose.position.y;
    raw.pose.position.z = msg->pose.position.z;
    raw.velocity.linear.x = msg->velocity.linear.x;
    raw.velocity.linear.y = msg->velocity.linear.y;
    raw.velocity.linear.z = msg->velocity.linear.z;
    raw.acceleration.linear.x = msg->acceleration.linear.x;
    raw.acceleration.linear.y = msg->acceleration.linear.y;
    raw.acceleration.linear.z = msg->acceleration.linear.z;
    raw.attitude.z = msg->attitude.z;
    raw.velocity.angular.z = msg->velocity.angular.z;

    // Running Average Filter
    RunningAverageBuffer_x[NextRunningAverage_x++] = raw.pose.position.x;
    if (NextRunningAverage_x >= RunningAverageCount_x)
    {
      NextRunningAverage_x = 0;
    }
    float RunningAverageTemperature_x = 0;
    for(int i=0; i< RunningAverageCount_x; ++i)
    {
      RunningAverageTemperature_x += RunningAverageBuffer_x[i];
    }
    RunningAverageTemperature_x /= RunningAverageCount_x;
    vehicle1.pose.position.x = RunningAverageTemperature_x;

    RunningAverageBuffer_y[NextRunningAverage_y++] = raw.pose.position.y;
    if (NextRunningAverage_y >= RunningAverageCount_y)
    {
      NextRunningAverage_y = 0;
    }
    float RunningAverageTemperature_y = 0;
    for(int i=0; i< RunningAverageCount_y; ++i)
    {
      RunningAverageTemperature_y += RunningAverageBuffer_y[i];
    }
    RunningAverageTemperature_y /= RunningAverageCount_y;
    vehicle1.pose.position.y = RunningAverageTemperature_y;

    RunningAverageBuffer_z[NextRunningAverage_z++] = raw.pose.position.z;
    if (NextRunningAverage_z >= RunningAverageCount_z)
    {
      NextRunningAverage_z = 0;
    }
    float RunningAverageTemperature_z = 0;
    for(int i=0; i< RunningAverageCount_z; ++i)
    {
      RunningAverageTemperature_z += RunningAverageBuffer_z[i];
    }
    RunningAverageTemperature_z /= RunningAverageCount_z;
    vehicle1.pose.position.z = RunningAverageTemperature_z;

    RunningAverageBuffer_vx[NextRunningAverage_vx++] = raw.velocity.linear.x;
    if (NextRunningAverage_vx >= RunningAverageCount_vx)
    {
      NextRunningAverage_vx = 0;
    }
    float RunningAverageTemperature_vx = 0;
    for(int i=0; i< RunningAverageCount_vx; ++i)
    {
      RunningAverageTemperature_vx += RunningAverageBuffer_vx[i];
    }
    RunningAverageTemperature_vx /= RunningAverageCount_vx;
    vehicle1.velocity.linear.x = RunningAverageTemperature_vx;

    RunningAverageBuffer_vy[NextRunningAverage_vy++] = raw.velocity.linear.y;
    if (NextRunningAverage_vy >= RunningAverageCount_vy)
    {
      NextRunningAverage_vy = 0;
    }
    float RunningAverageTemperature_vy = 0;
    for(int i=0; i< RunningAverageCount_vy; ++i)
    {
      RunningAverageTemperature_vy += RunningAverageBuffer_vy[i];
    }
    RunningAverageTemperature_vy /= RunningAverageCount_vy;
    vehicle1.velocity.linear.y = RunningAverageTemperature_vy;

    RunningAverageBuffer_vz[NextRunningAverage_vz++] = raw.velocity.linear.z;
    if (NextRunningAverage_vz >= RunningAverageCount_vz)
    {
      NextRunningAverage_vz = 0;
    }
    float RunningAverageTemperature_vz = 0;
    for(int i=0; i< RunningAverageCount_vz; ++i)
    {
      RunningAverageTemperature_vz += RunningAverageBuffer_vz[i];
    }
    RunningAverageTemperature_vz /= RunningAverageCount_vz;
    vehicle1.velocity.linear.z = RunningAverageTemperature_vz;

    RunningAverageBuffer_ax[NextRunningAverage_ax++] = raw.acceleration.linear.x;
    if (NextRunningAverage_ax >= RunningAverageCount_ax)
    {
      NextRunningAverage_ax = 0;
    }
    float RunningAverageTemperature_ax = 0;
    for(int i=0; i< RunningAverageCount_ax; ++i)
    {
      RunningAverageTemperature_ax += RunningAverageBuffer_ax[i];
    }
    RunningAverageTemperature_ax /= RunningAverageCount_ax;
    vehicle1.acceleration.linear.x = RunningAverageTemperature_ax;

    RunningAverageBuffer_ay[NextRunningAverage_ay++] = raw.acceleration.linear.y;
    if (NextRunningAverage_ay >= RunningAverageCount_ay)
    {
      NextRunningAverage_ay = 0;
    }
    float RunningAverageTemperature_ay = 0;
    for(int i=0; i< RunningAverageCount_ay; ++i)
    {
      RunningAverageTemperature_ay += RunningAverageBuffer_ay[i];
    }
    RunningAverageTemperature_ay /= RunningAverageCount_ay;
    vehicle1.acceleration.linear.y = RunningAverageTemperature_ay;

    RunningAverageBuffer_az[NextRunningAverage_az++] = raw.acceleration.linear.z;
    if (NextRunningAverage_az >= RunningAverageCount_az)
    {
      NextRunningAverage_az = 0;
    }
    float RunningAverageTemperature_az = 0;
    for(int i=0; i< RunningAverageCount_az; ++i)
    {
      RunningAverageTemperature_az += RunningAverageBuffer_az[i];
    }
    RunningAverageTemperature_az /= RunningAverageCount_az;
    vehicle1.acceleration.linear.z = RunningAverageTemperature_az;
  }

  void callback3(const quad::quad_state_msg::ConstPtr& msg)
  {
    if (initial < 100)
    {
      x_initial = msg->pose.position.x;
      y_initial = msg->pose.position.y;
      initial = initial + 1;
    }
  }

  void publishGenerateTrajectory(void)
  {
    // Trajectory Generation
    if (mode != 0)
    {
      if (mode == 1)
      {
        state.pose.position.x = x_initial;
        state.pose.position.y = y_initial;
        state.pose.position.z = 0.5;
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
        state.velocity.linear.z = vehicle1.velocity.linear.z;
        state.acceleration.linear.x = vehicle1.acceleration.linear.x;
        state.acceleration.linear.y = vehicle1.acceleration.linear.y;
        state.acceleration.linear.z = vehicle1.acceleration.linear.z;
        state.attitude.z = 0;
        state.velocity.angular.z = 0;
        state.mode = 2;
        // Increment
        k = k + 0.01;
      }
      else if (mode == 3)
      {
        state.pose.position.x = 0;
        state.pose.position.y = -d;
        state.pose.position.z = 0.5 - j*0.2;
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
        if (state.pose.position.z < 0.05)
        {
          state.mode = -1;
        }
      }
      else
      {
        state.pose.position.x = x_initial;
        state.pose.position.y = y_initial;
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
      state.pose.position.x = x_initial;
      state.pose.position.y = y_initial;
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
  ros::init(argc, argv, "quad_trajectory_generation_2");
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
