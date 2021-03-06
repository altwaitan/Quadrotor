// Author: Abdullah Altawaitan
// Date: April 4, 2019

#include "ros/ros.h"
#include <serial/serial.h>
#include "Eigen/Dense"
#include "math.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "quad/quad_state_msg.h"
#include "mavlink/mavlink.h"

float x, y, z, vx, vy, vz, qw, qx, qy, qz;
float vx_prev, vy_prev, vz_prev;
float x_offset, y_offset, z_offset;
float sum_average[13];
float moving_average[13][5];
class quad_state
{
public:
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Subscriber sub2;
  ros::Subscriber sub3;
  ros::Publisher pub;
  ros::Publisher pub2;
  sensor_msgs::Imu imu;
  quad::quad_state_msg state;

  quad_state()
  {
    sub = n.subscribe<sensor_msgs::Imu>("/quad_serial_imu_2", 1000, &quad_state::callback, this);
    sub2 = n.subscribe<nav_msgs::Odometry>("/vive/LHR_AC094AF4_odom", 1000, &quad_state::callback2, this);
    sub3 = n.subscribe<quad::quad_state_msg>("quad_state", 1000, &quad_state::callback3, this);
    pub = n.advertise<quad::quad_state_msg>("quad_state_2", 1000);
  }

  ~quad_state()
  {
    // Empty
  }

  void callback(const sensor_msgs::Imu::ConstPtr& msg)
  {
    imu.linear_acceleration.x = msg->linear_acceleration.x;
    imu.linear_acceleration.y = msg->linear_acceleration.y;
    imu.linear_acceleration.z = msg->linear_acceleration.z;
    imu.angular_velocity.x = msg->angular_velocity.x;
    imu.angular_velocity.y = msg->angular_velocity.y;
    imu.angular_velocity.z = msg->angular_velocity.z;
    imu.orientation.x = msg->orientation.x; // Roll angle
    imu.orientation.y = msg->orientation.y; // Pitch angle

    state.velocity.angular.x = imu.angular_velocity.x;
    state.velocity.angular.y = imu.angular_velocity.y;
    state.velocity.angular.z = imu.angular_velocity.z;
    state.attitude.x = imu.orientation.x;
    state.attitude.y = imu.orientation.y;
  }

  void callback2(const nav_msgs::Odometry::ConstPtr& msg)
  {
    // Transformation
    tf::Quaternion q1(msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z,
                      msg->pose.pose.orientation.w);
    tf::Quaternion q2(0.707, 0.000, 0.000, 0.707);
    tf::Matrix3x3 m(q2*q1);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    state.attitude.z = -(float) yaw; // Yaw (psi) in (rads)
    tf::Quaternion myQuaternion;
    myQuaternion.setRPY(roll, pitch, yaw);
    state.pose.orientation.w = myQuaternion.w();
    state.pose.orientation.x = myQuaternion.x();
    state.pose.orientation.y = myQuaternion.y();
    state.pose.orientation.z = myQuaternion.z();


    // Moving Average
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

    // Shift axes
    x = -(msg->pose.pose.position.x) - state.x_offset;
    y = -(msg->pose.pose.position.z) - state.y_offset;
    z = msg->pose.pose.position.y - state.z_offset;

    moving_average[0][0] = x;
    moving_average[1][0] = y;
    moving_average[2][0] = z;
    moving_average[3][0] = -(msg->twist.twist.linear.x);
    moving_average[4][0] = -(msg->twist.twist.linear.z);
    moving_average[5][0] = msg->twist.twist.linear.y;
    moving_average[6][0] = state.attitude.x;
    moving_average[7][0] = state.attitude.y;
    moving_average[8][0] = state.attitude.z;
    moving_average[9][0] = state.pose.orientation.w;
    moving_average[10][0] = state.pose.orientation.x;
    moving_average[11][0] = state.pose.orientation.y;
    moving_average[12][0] = state.pose.orientation.z;

    for (uint8_t j = 0; j < 13 ; j++)
    {
      sum_average[j] = sum_average[j] + moving_average[j][0] / 5;
    }

    x = sum_average[0];
    y = sum_average[1];
    z = sum_average[2];
    vx = sum_average[3];
    vy = sum_average[4];
    vz = sum_average[5];
    qw = sum_average[9];
    qx = sum_average[10];
    qy = sum_average[11];
    qz = sum_average[12];

    state.acceleration.linear.x = (vx - vx_prev) / (0.01);
    state.acceleration.linear.y = (vy - vy_prev) / (0.01);
    state.acceleration.linear.z = (vz - vz_prev) / (0.01);
    vx_prev = vx;
    vy_prev = vy;
    vz_prev = vz;

    float CF_a = 0.8, dtOuter = 0.01, dtOuter_2 = dtOuter * dtOuter;
    state.velocity.linear.x = (state.velocity.linear.x + dtOuter * state.acceleration.linear.x) * CF_a + vx * (1 - CF_a);
    state.velocity.linear.y = (state.velocity.linear.y + dtOuter * state.acceleration.linear.y) * CF_a + vy * (1 - CF_a);
    state.velocity.linear.z = (state.velocity.linear.z + dtOuter * state.acceleration.linear.z) * CF_a + vz * (1 - CF_a);

    state.pose.position.x = (state.pose.position.x + dtOuter * state.velocity.linear.x + 0.5 * dtOuter_2 * state.acceleration.linear.x) * CF_a + x * (1 - CF_a);
    state.pose.position.y = (state.pose.position.y + dtOuter * state.velocity.linear.y + 0.5 * dtOuter_2 * state.acceleration.linear.y) * CF_a + y * (1 - CF_a);
    state.pose.position.z = (state.pose.position.z + dtOuter * state.velocity.linear.z + 0.5 * dtOuter_2 * state.acceleration.linear.z) * CF_a + z * (1 - CF_a);

    // Publish
    pub.publish(state);
  }

  void callback3(const quad::quad_state_msg::ConstPtr& msg)
  {
    state.x_offset = msg->x_offset;
    state.y_offset = msg->y_offset;
    state.z_offset = msg->z_offset;
  }

  float invSqrt(float number)
  {
    long i;
    float x2, y;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    y  = number;
    i  = * ( long * ) &y;
    i  = 0x5f3759df - ( i >> 1 );
    y  = * ( float * ) &i;
    y  = y * ( threehalfs - ( x2 * y * y ) );
    return y;
  }

};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "quad_state_2");
  quad_state quad;

  ros::Rate loop_rate(100);

  ros::spin();
  ros::shutdown();
  return 0;
}
