// Author: Abdullah Altawaitan
// Date: April 4, 2019

#include "ros/ros.h"
#include "fame/fame_state.h"
#include "fame/fame_msg.h"


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
    sub = n.subscribe<fame::fame_state>("/fame_filter_state", 1000, &fame_control::callback, this);
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
  }

  void LQR(void)
  {
    // LQR Control
    float error_x = state.pose.position.x - desired.pose.position.x;
    float error_y = state.pose.position.y - desired.pose.position.y;
    float error_z = state.pose.position.z - desired.pose.position.z;
    float error_vx = state.velocity.linear.x - desired.velocity.linear.x;
    float error_vy = state.velocity.linear.y - desired.velocity.linear.y;
    float error_vz = state.velocity.linear.z - desired.velocity.linear.z;
    float error_psi = state.attitude.z - desired.attitude.z;

    float g1 = 1, g2 = 5;
    float u1, u2, u3, u4;
    u1 = -g1 * error_x - g2 * error_vx;
    u2 = -g1 * error_y - g2 * error_vy;
    u3 = -g1 * error_z - g2 * error_vz;
    u4 = -g1 * error_psi;

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
    float r_cmd = up_psi*cos(theta_cmd)*cos(phi_cmd) + state.velocity.angular.y*sin(phi_cmd);

    phi_cmd = CONSTRAIN(phi_cmd, -0.35, 0.35);
    theta_cmd = CONSTRAIN(theta_cmd, -0.35, 0.35);
    r_cmd = CONSTRAIN(r_cmd, -1, 1);

    fame_cmd.thrust_des = thrust_cmd;
    fame_cmd.phi_des = phi_cmd;
    fame_cmd.theta_des = theta_cmd;
    fame_cmd.r_des = r_cmd;
    pub.publish(fame_cmd);
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


  ros::spin();
  ros::shutdown();
  return 0;
}
