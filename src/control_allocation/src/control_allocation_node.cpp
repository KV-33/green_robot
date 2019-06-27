#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist wheelSpeedRefLeftMsg;
geometry_msgs::Twist wheelSpeedRefRightMsg;

void cmd_velCallback(const geometry_msgs::Twist& msg_cmd_vel)
{
  double V = msg_cmd_vel.linear.x;
  double Omega = msg_cmd_vel.angular.z;

  wheelSpeedRefLeftMsg.linear.x = (V - Omega);
  wheelSpeedRefRightMsg.linear.x = (V + Omega);
  //ROS_INFO("LINEAR: [%f], ANGULAR: [%f]", msg_cmd_vel.linear.x, msg_cmd_vel.angular.z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_allocation_node");
  ros::NodeHandle nh;

  ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 50, cmd_velCallback);

  ros::Publisher pub_motor_left = nh.advertise<geometry_msgs::Twist>("cmd_vel/left", 50);
  ros::Publisher pub_motor_right = nh.advertise<geometry_msgs::Twist>("cmd_vel/right", 50);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    pub_motor_left.publish(wheelSpeedRefLeftMsg);
    pub_motor_right.publish(wheelSpeedRefRightMsg);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
