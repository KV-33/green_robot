#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"


#define WHEEL_DIAMETER            0.144    // диаметр колеса в метрах
#define ENCODER_RESOLUTION        1536.0   // количество импульсов на оборот колеса   //12*128

double wheel_diameter;
double encoder_resolution;
double encoder_prev_left;
double encoder_prev_right;
double encoder_count_left;
double encoder_count_right;
ros::Time msg_time_last;
double time_interval;


double getIntervalCount(double now_count, double old_count){
    if (abs(now_count-old_count)>1000000.00)
    {
        return now_count;
    }
    else
    {
        return now_count-old_count;
    }
}

void encoderCallback(const sensor_msgs::JointState& msg)
{
    if(msg.name.size() == 2 && msg.name[0].compare("left_wheel") == 0 && msg.name[1].compare("right_wheel") == 0) {
        encoder_count_left = getIntervalCount(msg.position[0], encoder_prev_left);
        encoder_count_right = getIntervalCount(msg.position[1], encoder_prev_right);
        encoder_prev_left = msg.position[0];
        encoder_prev_right = msg.position[1];
        time_interval = msg.header.stamp.toSec() - msg_time_last.toSec();
        msg_time_last = msg.header.stamp;
    }
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

    geometry_msgs::Twist wheel_msg_left;
    geometry_msgs::Twist wheel_msg_right;


    ros::Subscriber encoder_sub = nh.subscribe(topic_wheel_encoder, 50, encoderCallback);
    ros::Publisher speed_pub_left = nh.advertise<geometry_msgs::Twist>(topic_wheel_speed+"/left", 50);
    ros::Publisher speed_pub_right = nh.advertise<geometry_msgs::Twist>(topic_wheel_speed+"/right", 50);

    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        if(time_interval > 0.0){
            wheel_msg_left.linear.x = impulse2metersInSeconds(encoder_count_left);
            wheel_msg_right.linear.x = impulse2metersInSeconds(encoder_count_right);
        }
        speed_pub_left.publish(wheel_msg_left);
        speed_pub_right.publish(wheel_msg_right);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
