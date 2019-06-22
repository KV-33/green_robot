#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <geometry_msgs/Twist.h>

#define INTERVAL_WHEEL_DISTANCE   0.254  //Расстояние между колёсами
#define WHEEL_DIAMETER            0.144 //Диаметр колёс


float r = WHEEL_DIAMETER/2.0;

geometry_msgs::Twist wheelSpeedRefLeftMsg;
geometry_msgs::Twist wheelSpeedRefRightMsg;

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void cmd_velCallback(const geometry_msgs::Twist& msg_cmd_vel)
{
  float V = msg_cmd_vel.linear.x;
  float Omega = msg_cmd_vel.angular.z;

  wheelSpeedRefLeftMsg.linear.x = map(0.5*(2.0*V - INTERVAL_WHEEL_DISTANCE*Omega)/r, -15, 15, -255, 255);
  wheelSpeedRefRightMsg.linear.x = map(0.5*(2.0*V + INTERVAL_WHEEL_DISTANCE*Omega)/r, -15, 15, -255, 255);
  ROS_INFO("LINEAR: [%f], ANGULAR: [%f]", msg_cmd_vel.linear.x, msg_cmd_vel.angular.z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_allocation_node");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("cmd_vel", 100, cmd_velCallback);

  ros::Publisher pub_motor_left = nh.advertise<geometry_msgs::Twist>("green_robot/sensors/encoders/left", 50);
  ros::Publisher pub_motor_right = nh.advertise<geometry_msgs::Twist>("green_robot/sensors/encoders/left", 50);

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
