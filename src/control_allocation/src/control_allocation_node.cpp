#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <geometry_msgs/Twist.h>
#include "green_robot/Int32Stamped.h"
green_robot::Int32Stamped wheelSpeedRefLeftMsg;
green_robot::Int32Stamped wheelSpeedRefRightMsg;

float interWheelDistance_ = 0.254;  //Расстояние между колёсами
float wheelDiameter_ = 0.144; //Диаметр колёс

float d = interWheelDistance_;

float rL = wheelDiameter_/2.0;
float rR = wheelDiameter_/2.0;

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void cmd_velCallback(const geometry_msgs::Twist& msg_cmd_vel)
{
  float V = msg_cmd_vel.linear.x;
  float Omega = msg_cmd_vel.angular.z;

  wheelSpeedRefLeftMsg.data = map(0.5*(2.0*V - d*Omega)/rL, -15, 15, -255, 255);
  wheelSpeedRefRightMsg.data = map(0.5*(2.0*V + d*Omega)/rR, -15, 15, -255, 255);
  ROS_INFO("LINEAR: [%f], ANGULAR: [%f]", msg_cmd_vel.linear.x, msg_cmd_vel.angular.z);
}

int main(int argc, char **argv)



{
  ros::init(argc, argv, "controlallocation_node");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("cmd_vel", 100, cmd_velCallback);
  ros::Publisher pub_motor_left = nh.advertise<green_robot::Int32Stamped>("rawspeed/left", 30);
  ros::Publisher pub_motor_right = nh.advertise<green_robot::Int32Stamped>("rawspeed/right", 30);



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
