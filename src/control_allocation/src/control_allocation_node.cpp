#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <geometry_msgs/Twist.h>

std_msgs::Float32 wheelSpeedRefLeftMsg;
std_msgs::Float32 wheelSpeedRefRightMsg;

float interWheelDistance_ = 0.3;  //Расстояние между колёсами
float wheelDiameter_ = 0.1; //Диаметр колёс

float d = interWheelDistance_;

float rL = wheelDiameter_/2.0;
float rR = wheelDiameter_/2.0;


void cmd_velCallback(const geometry_msgs::Twist& msg_cmd_vel)
{
  float V = msg_cmd_vel.linear.x;
  float Omega = msg_cmd_vel.angular.z;

  wheelSpeedRefLeftMsg.data = 0.5*(2.0*V - d*Omega)/rL;
  wheelSpeedRefRightMsg.data = 0.5*(2.0*V + d*Omega)/rR;
  ROS_INFO("LINEAR: [%f], ANGULAR: [%f]", msg_cmd_vel.linear.x, msg_cmd_vel.angular.z);
}

int main(int argc, char **argv)



{
  ros::init(argc, argv, "controlallocation_node");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("cmd_vel", 100, cmd_velCallback);
  ros::Publisher pub_motor_left = nh.advertise<std_msgs::Float32>("wheelSpeedRef/left", 100);
  ros::Publisher pub_motor_right = nh.advertise<std_msgs::Float32>("wheelSpeedRef/right", 100);



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
