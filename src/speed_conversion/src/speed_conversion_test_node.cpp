#include "ros/ros.h"
#include "speed_conversion/Int32Stamped.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "speed_conversion_test_node");
    ros::NodeHandle nh;

    ros::Publisher speed_pub = nh.advertise<speed_conversion::Int32Stamped>("green_robot/sensors/encoders/left", 100);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        speed_conversion::Int32Stamped msg;
        msg.data = 1733;//3467;
        msg.header.stamp = ros::Time::now();

        speed_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
