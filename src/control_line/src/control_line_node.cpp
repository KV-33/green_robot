#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/JointState.h"

#define KP             1.0
#define LINEAR_SPEED   0.5

geometry_msgs::Twist cmd_vel;
double Kp = 1.0;

void sensorsLineCallback(const sensor_msgs::JointState& msg)
{
    if(msg.name.size() == 2 && msg.name[0].compare("left_sensor_line") == 0 && msg.name[1].compare("right_sensor_line") == 0) {
        cmd_vel.angular.z = Kp * (msg.position[1] - msg.position[0]);
    } else {
        cmd_vel.angular.z = 0;
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_line_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  std::string topic_sensors_line;
  std::string topic_cmd_vel;

  priv_nh.param<std::string>("topic_sensors_line", topic_sensors_line, "sensors/line");
  priv_nh.param<std::string>("topic_cmd_vel", topic_cmd_vel, "cmd_vel");
  priv_nh.param("Kp", Kp, KP);
  priv_nh.param("linear_speed", cmd_vel.linear.x, LINEAR_SPEED);

  ros::Subscriber sensor_line_sub = nh.subscribe(topic_sensors_line, 50, sensorsLineCallback);

  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(topic_cmd_vel, 50);

  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    ros::spinOnce();
    cmd_vel_pub.publish(cmd_vel);
    loop_rate.sleep();
  }
  return 0;
}
