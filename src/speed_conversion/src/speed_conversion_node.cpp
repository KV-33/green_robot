#include "ros/ros.h"
#include "speed_conversion/Int32Stamped.h"
#include "geometry_msgs/Twist.h"


#define WHEEL_DIAMETER            0.144    // диаметр колеса в метрах
#define ENCODER_RESOLUTION        1536.0   // количество импульсов на оборот колеса   //12*128

double wheel_diameter;
double encoder_resolution;
int encoders_count;
ros::Time msg_time_last;
ros::Time msg_time;

void encodersCallback(const speed_conversion::Int32Stamped& msg)
{
    encoders_count = msg.data;
    msg_time = msg.header.stamp;
}

double impulse2meters(double x) {
    return ((x / (encoder_resolution / 360.0)) * M_PI / 180.0) * (wheel_diameter / 2);
}

double inSeconds(double x, long t) {
  return x / (t / 1000.0); //преобразуем в рад/с
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

    ros::Subscriber encoder_sub = nh.subscribe(topic_wheel_encoder, 100, encodersCallback);
    ros::Publisher speed_pub = nh.advertise<geometry_msgs::Twist>(topic_wheel_speed, 100);

    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        double t = msg_time.toSec()-msg_time_last.toSec();
        msg_time_last = msg_time;
        wheel_msg.linear.x = impulse2meters(encoders_count)/t;
        speed_pub.publish(wheel_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
