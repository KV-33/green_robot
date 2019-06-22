#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <geometry_msgs/Twist.h>
#include "string"

#define WHEEL_INTERVAL   0.254 //Расстояние между колёсами
#define WHEEL_DIAMETER   0.144 //Диаметр колёс

double wheel_interval;
double wheel_diameter;

geometry_msgs::Twist wheelSpeedRefLeftMsg;
geometry_msgs::Twist wheelSpeedRefRightMsg;

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void cmd_velCallback(const geometry_msgs::Twist& msg_cmd_vel)
{
  double V = msg_cmd_vel.linear.x;
  double Omega = msg_cmd_vel.angular.z;

  wheelSpeedRefLeftMsg.linear.x = map(0.5*(2.0*V - wheel_interval*Omega)/(wheel_diameter/2.0), -15, 15, -255, 255);
  wheelSpeedRefRightMsg.linear.x = map(0.5*(2.0*V + wheel_interval*Omega)/(wheel_diameter/2.0), -15, 15, -255, 255);
  ROS_INFO("LINEAR: [%f], ANGULAR: [%f]", msg_cmd_vel.linear.x, msg_cmd_vel.angular.z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_allocation_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  priv_nh.param("wheel_diameter", wheel_diameter, WHEEL_DIAMETER);
  priv_nh.param("wheel_interval", wheel_interval, WHEEL_INTERVAL);

  ros::Subscriber cmd_vel_sub = nh.subscribe("green_robot/cmd_vel", 100, cmd_velCallback);

  ros::Publisher pub_motor_left = nh.advertise<geometry_msgs::Twist>("green_robot/sensors/encoders/left", 50);
  ros::Publisher pub_motor_right = nh.advertise<geometry_msgs::Twist>("green_robot/sensors/encoders/right", 50);

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
