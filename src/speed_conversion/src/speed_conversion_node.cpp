#include "ros/ros.h"
#include "speed_conversion/Int32Stamped.h"
#include "geometry_msgs/Twist.h"


#define WHEEL_DIAMETER            0.144    // диаметр колеса в метрах
#define ENCODER_RESOLUTION        1536.0   // количество импульсов на оборот колеса   //12*128

double wheel_diameter;
double encoder_resolution;
int encoders_count;
ros::Time msg_time_last;
double time_interval;

void encodersCallback(const speed_conversion::Int32Stamped& msg)
{
    encoders_count = msg.data;
    time_interval = msg.header.stamp.toSec() - msg_time_last.toSec();
    msg_time_last = msg.header.stamp;
}

double impulse2metersInSeconds(double x) {
    return ((x / encoder_resolution) * M_PI * wheel_diameter)/time_interval;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "speed_conversion_node");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    std::string topic_wheel_speed;
    std::string topic_wheel_encoder;

    priv_nh.param<std::string>("topic_wheel_speed", topic_wheel_speed, "sensors/wheel_speed");
    priv_nh.param<std::string>("topic_wheel_encoder", topic_wheel_encoder, "sensors/encoders");
    priv_nh.param("wheel_diameter", wheel_diameter, WHEEL_DIAMETER);
    priv_nh.param("encoder_resolution", encoder_resolution, ENCODER_RESOLUTION);

    geometry_msgs::Twist wheel_msg;

    ros::Subscriber encoder_sub = nh.subscribe(topic_wheel_encoder, 50, encodersCallback);
    ros::Publisher speed_pub = nh.advertise<geometry_msgs::Twist>(topic_wheel_speed, 50);

    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        if(encoders_count > 0 && time_interval > 0.0){
            wheel_msg.linear.x = impulse2metersInSeconds(encoders_count);
        }
        else
        {
            wheel_msg.linear.x = impulse2metersInSeconds(encoders_count);//wheel_msg.linear.x = 0.0;
        }
        speed_pub.publish(wheel_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
