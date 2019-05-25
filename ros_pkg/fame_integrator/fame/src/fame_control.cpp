// Author: Abdullah Altawaitan
// Date: April 4, 2019

#include "ros/ros.h"
#include "fame/fame_state.h"
#include "fame/fame_msg.h"

double error_xi = 0;
double error_yi = 0;
double error_zi = 0;
double error_psii = 0;
class fame_control
{
public:
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Subscriber sub2;
  ros::Publisher pub;

  fame::fame_msg fame_cmd;
  fame::fame_state state;
  fame::fame_state desired;
  float mass = 0.647;


  fame_control()
  {
    sub = n.subscribe<fame::fame_state>("/fame_state", 1000, &fame_control::callback, this);
    sub2 = n.subscribe<fame::fame_state>("/fame_trajectory", 1000, &fame_control::callback2, this);
    pub = n.advertise<fame::fame_msg>("fame_cmd", 1000);
  }

  ~fame_control()
  {
    // Empty
  }

  void callback(const fame::fame_state::ConstPtr& msg)
  {
    state.pose.position.x = msg->pose.position.x;
    state.pose.position.y = msg->pose.position.y;
    state.pose.position.z = msg->pose.position.z;
    state.velocity.linear.x = msg->velocity.linear.x;
    state.velocity.linear.y = msg->velocity.linear.y;
    state.velocity.linear.z = msg->velocity.linear.z;
    state.velocity.angular.x = msg->velocity.angular.x;
    state.velocity.angular.y = msg->velocity.angular.y;
    state.velocity.angular.z = msg->velocity.angular.z;
    state.acceleration.linear.x = msg->acceleration.linear.x;
    state.acceleration.linear.y = msg->acceleration.linear.y;
    state.acceleration.linear.z = msg->acceleration.linear.z;
    state.attitude.z = msg->attitude.z;
    state.velocity.angular.z = msg->velocity.angular.z;
  }

  void callback2(const fame::fame_state::ConstPtr& msg)
  {
    desired.pose.position.x = msg->pose.position.x;
    desired.pose.position.y = msg->pose.position.y;
    desired.pose.position.z = msg->pose.position.z;
    desired.velocity.linear.x = msg->velocity.linear.x;
    desired.velocity.linear.y = msg->velocity.linear.y;
    desired.velocity.linear.z = msg->velocity.linear.z;
    desired.acceleration.linear.x = msg->acceleration.linear.x;
    desired.acceleration.linear.y = msg->acceleration.linear.y;
    desired.acceleration.linear.z = msg->acceleration.linear.z;
    desired.attitude.z = msg->attitude.z;
    desired.velocity.angular.z = msg->velocity.angular.z;
    desired.mode = msg->mode;
  }

  void LQR(void)
  {
    // LQR Control
    float error_x = desired.pose.position.x - state.pose.position.x;
    float error_y = desired.pose.position.y - state.pose.position.y;
    float error_z = desired.pose.position.z - state.pose.position.z;
    float error_vx = desired.velocity.linear.x - state.velocity.linear.x;
    float error_vy = desired.velocity.linear.y - state.velocity.linear.y;
    float error_vz = desired.velocity.linear.z - state.velocity.linear.z;
    float error_psi = desired.attitude.z - state.attitude.z;

    // Integration
    error_xi += error_x * 0.01;
    error_yi += error_y * 0.01;
    error_zi += error_z * 0.01;
    error_psii += error_psii * 0.01;

    float g1 = 4.6, g2 = 3.74, g3 =  4.6;
    float u1, u2, u3, u4;
    u1 = g1 * error_x + g2 * error_vx + g3 * error_xi;
    u2 = g1 * error_y + g2 * error_vy + g3 * error_yi;
    u3 = g1 * error_z + g2 * error_vz + g3 * error_zi;
    u4 = g1 * error_psi + g3 * error_psii;

    // Anti-Windup
    double saturated_u1 = CONSTRAIN(u1, -4, 4);
    double saturated_u2 = CONSTRAIN(u2, -4, 4);
    double saturated_u3 = CONSTRAIN(u3, -5, 5);
    double saturated_u4 = CONSTRAIN(u4, -4, 4);
    int clamp_x = antiWindup(error_x, u1, saturated_u1);
    int clamp_y = antiWindup(error_y, u2, saturated_u2);
    int clamp_z = antiWindup(error_z, u3, saturated_u3);
    int clamp_psi = antiWindup(error_psi, u4, saturated_u4);

    if (clamp_x == 1)
    {
      // integral part is set to zero
      error_xi = 0;
      u1 = g1 * error_x + g2 * error_vx;
    }
    if (clamp_y == 1)
    {
      // integral part is set to zero
      error_yi = 0;
      u2 = g1 * error_y + g2 * error_vy;
    }
    if (clamp_z == 1)
    {
      // integral part is set to zero
      error_zi = 0;
      u3 = g1 * error_z + g2 * error_vz;
    }
    if (clamp_psi == 1)
    {
      // integral part is set to zero
      error_psii = 0;
      u4 = g1 * error_psi;
    }

    // Feedforward
    float up_1, up_2, up_3, up_psi;
    up_1 = u1 + desired.acceleration.linear.x;
    up_2 = u2 + desired.acceleration.linear.y;
    up_3 = u3 + desired.acceleration.linear.z + 9.81;
    up_psi = u4 + desired.velocity.angular.z;

    // Inverse Mapping
    float z1, z2, z3;
    float thrust_cmd = mass * sqrt(up_1*up_1 + up_2*up_2 + up_3*up_3);
    thrust_cmd = CONSTRAIN(thrust_cmd, 0, 15);
    z1 = (mass/thrust_cmd) * (up_1*cos(state.attitude.z) + up_2*sin(state.attitude.z));
    z2 = (mass/thrust_cmd) * (-up_1*sin(state.attitude.z) + up_2*cos(state.attitude.z));
    z3 = (mass/thrust_cmd) * up_3;
    float phi_cmd = asin(z2);
    float theta_cmd = atan(-z1/z3);
    float r_cmd = up_psi*cos(theta_cmd)*cos(phi_cmd) + state.velocity.angular.x*sin(phi_cmd);

    phi_cmd = CONSTRAIN(phi_cmd, -1, 1);
    theta_cmd = CONSTRAIN(theta_cmd, -1, 1);
    r_cmd = CONSTRAIN(r_cmd, -1.5, 1.5);

    fame_cmd.thrust_des = thrust_cmd;
    fame_cmd.phi_des = phi_cmd;
    fame_cmd.theta_des = theta_cmd;
    fame_cmd.r_des = r_cmd;
    fame_cmd.mode = desired.mode;
    pub.publish(fame_cmd);
  }

  int antiWindup(float error, float output, float saturatedOutput)
  {
    int check1 = 0;
    int check2 = 0;
    int clamp = 0;

    // Check 1
    if (output != saturatedOutput)
    {
      check1 = 1;
    }
    else
    {
      check1 = 0;
    }

    // Check 2
    if (sign(error) != sign(output))
    {
      check2 = 1;
    }
    else
    {
      check2 = 0;
    }

    // Check 3
    if ((check1 * check2) == 1)
    {
      clamp = 1;
    }
    else
    {
      clamp = 0;
    }
    return clamp;
  }

  int sign(float num)
  {
    if (num > 0) return 1;
    if (num < 0) return -1;
    return 0;
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
  ros::init(argc, argv, "fame_control");
  fame_control fame;

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    fame.LQR();
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::shutdown();
  return 0;
}
