#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

#define WHEEL_DIAMETER            0.144    // диаметр колеса в метрах
#define WHEEL_BASE                0.13      // база колесная

double wheel_diameter;
double wheel_base;

geometry_msgs::Twist wheelSpeedRefLeftMsg;
geometry_msgs::Twist wheelSpeedRefRightMsg;

void cmd_velCallback(const geometry_msgs::Twist& msg_cmd_vel)
{
    double V = msg_cmd_vel.linear.x;        //линейная скорость
    double W = msg_cmd_vel.angular.z;   //угловая скорость
    double r = wheel_diameter/2;            //радиус колеса
    double d = wheel_base;                  //база робота

    // Equations are as follows:
    // r = Wheel radius, meters
    // d = Wheel seperation, meters
    // Wr = Right wheel angular velocity, radians/sec
    // Wl = Left wheel angular velocity, radians/sec
    // V = Forward velocity, meters/sec
    // W = Angular velocity, radians/sec
    //
    // V = (r/2)(Wr + Wl) = Forward velocity, meters/sec
    // W = (r/d)(Wr - Wl) = Angular velocity, radians/sec
    //
    // Inverting matrix equation:
    // Wr = (1/r)V + (d/r)W
    // Wl = (1/r)V - (d/r)W

    if((r > 0.0) && (d > 0.0))
    {
        wheelSpeedRefLeftMsg.linear.x = r * ((1 / r) * V + (d / r) * W); //1 / (WHEEL_DIAMETER/2) * (V - (Omega * WHEEL_BASE)/2);//(V - wheel_base * Omega)/r;
        wheelSpeedRefRightMsg.linear.x = r * ((1 / r) * V - (d / r) * W);//1 / (WHEEL_DIAMETER/2) * (V + (Omega * WHEEL_BASE)/2);//(V + wheel_base * Omega)/r;
        //ROS_INFO("LINEAR: [%f], ANGULAR: [%f]", msg_cmd_vel.linear.x, msg_cmd_vel.angular.z);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_allocation_node");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    priv_nh.param("wheel_diameter", wheel_diameter, WHEEL_DIAMETER);
    priv_nh.param("wheel_base", wheel_base, WHEEL_BASE);

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
