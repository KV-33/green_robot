#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "speed_conversion/Int32Stamped.h"


#define WHEEL_DIAMETER            0.144    // диаметр колеса в метрах
#define WHEEL_INTERVAL            0.254    // расстояние между колёсами
#define WHEEL_IMPULSE_COUNT       12*128   // количество импульсов на оборот колеса

double wheel_diameter;
double wheel_interval;



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

    ros::NodeHandle priv_nh("~");

    priv_nh.param("wheel_diameter", wheel_diameter, WHEEL_DIAMETER);
    priv_nh.param("wheel_interval", wheel_interval, WHEEL_INTERVAL);

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
