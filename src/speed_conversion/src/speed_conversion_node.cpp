#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "green_robot/Int32Stamped.h"



float interWheelDistance_ = 0.15;
float wheelDiameter_ = 0.07;
float x0_ = 0.;
float y0_ = 0.;
float theta0_ = 0.;
float encoderResolution_ = 720.0;




int main(int argc, char **argv)
{
    ros::init(argc, argv, "speed_conversion_node");
    ros::NodeHandle nh;

    geometry_msgs::Twist left_wheel_msg;
    geometry_msgs::Twist right_wheel_msg;

    ros::Publisher left_speed_pub = nh.advertise<geometry_msgs::Twist>("green_robot/sensors/wheel_speed/left", 1000);
    ros::Publisher right_speed_pub = nh.advertise<geometry_msgs::Twist>("green_robot/sensors/wheel_speed/right", 1000);

    ros::Rate loop_rate(50);
    while (ros::ok())
    {

        left_wheel_msg.linear.x = 0.0;
        right_wheel_msg.linear.x = 0.0;

        left_speed_pub.publish(left_wheel_msg);
        right_speed_pub.publish(right_wheel_msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
