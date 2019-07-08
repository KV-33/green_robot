#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

#define WHEEL_DIAMETER            0.144    // диаметр колеса в метрах

double wheel_diameter;

geometry_msgs::Twist wheelSpeedRefLeftMsg;
geometry_msgs::Twist wheelSpeedRefRightMsg;

void cmd_velCallback(const geometry_msgs::Twist& msg_cmd_vel)
{
  double V = msg_cmd_vel.linear.x;        //линейная скорость
  double Omega = msg_cmd_vel.angular.z;   //угловая скорость
  double d = wheel_diameter;              //диаметр колеса
  double r = wheel_diameter/2;            //радиус колеса

  wheelSpeedRefLeftMsg.linear.x = (V - d * Omega)/r;
  wheelSpeedRefRightMsg.linear.x = (V + d * Omega)/r;
  //ROS_INFO("LINEAR: [%f], ANGULAR: [%f]", msg_cmd_vel.linear.x, msg_cmd_vel.angular.z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_allocation_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  priv_nh.param("wheel_diameter", wheel_diameter, WHEEL_DIAMETER);

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
