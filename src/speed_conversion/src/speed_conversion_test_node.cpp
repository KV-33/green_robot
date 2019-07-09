#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "speed_conversion_test_node");
    ros::NodeHandle nh;

    ros::Publisher speed_pub = nh.advertise<sensor_msgs::JointState>("green_robot/sensors/encoders", 100);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        sensor_msgs::JointState msg;
        msg.position[0] = 1733;//3467;
        msg.position[1] = 1733;//3467;
        msg.header.stamp = ros::Time::now();

        speed_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
