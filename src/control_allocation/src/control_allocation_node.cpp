#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

#define WHEEL_DIAMETER            0.144    // диаметр колеса в метрах
#define WHEEL_BASE                0.13     // база колесная

double wheel_diameter;
double wheel_base;

geometry_msgs::Twist wheel_speed_left_msg;
geometry_msgs::Twist wheel_speed_right_msg;

void cmd_velCallback(const geometry_msgs::Twist& msg_cmd_vel)
{
    double V = msg_cmd_vel.linear.x;        //линейная скорость
    double W = msg_cmd_vel.angular.z;       //угловая скорость
    double R = wheel_diameter/2;            //радиус колеса
    double L = wheel_base;                  //колесная база робота

    // V = ((Wr + Wl)/2)*R
    // W = ((Wr - Wl)/L)*R
    //
    // Wl = (1/R)*V - (W*L/2)
    // Wr = (1/R)*V + (W*L/2)
    //
    // где:
    // R = Радиус колеса, м
    // L = Колесная база, м
    // Wr = Угловая скорость правого колеса, рад/с
    // Wl = Угловая скорость левого колеса, рад/с
    // V = Линейная скорость, м/с
    // W = Угловая скорость, рад/с

    if((R > 0.0) && (L > 0.0))
    {
        wheel_speed_left_msg.angular.x = (1 / R) * (V - ((W * L)/2));    // угловая скорость вращения левого колеса
        wheel_speed_right_msg.angular.x = (1 / R) * (V + ((W * L)/2));   // угловая скорость вращения правого колеса
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
        pub_motor_left.publish(wheel_speed_left_msg);
        pub_motor_right.publish(wheel_speed_right_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
