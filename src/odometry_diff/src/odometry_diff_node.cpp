#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "green_robot/Int32Stamped.h"



float interWheelDistance_ = 0.15;
float wheelDiameter_ = 0.07;
float x0_ = 0.;
float y0_ = 0.;
float theta0_ = 0.;
float encoderResolution_ = 720.0;




int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_diff_node");
    ros::NodeHandle nh;

    ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        std_msgs::String msg;
        msg.data = "hello world";

        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
