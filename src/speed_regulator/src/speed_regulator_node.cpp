#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>


#define Kp                        167.0    // пропорциональный коэффициент для ПИД регулятора (41.7)
#define Ki                        0.1      // интегральный коэффициент для ПИД регулятора
#define Kd                        0.4      // дифференциальный коэффициент для ПИД регулятора
/*#define K_SPEED                   0.63     // коэффициент неадекватной скорости*/
#define WHEEL_DIAMETER            0.144    // диаметр колеса в метрах
#define WHEEL_IMPULSE_COUNT       12*128    // количество импульсов на оборот колеса

float pid_constants[3];
float cmd_linear;
float cmd_angular;


float linear = 0;           //значение для драйвера моторов
float angular = 0;          //значение для руля моторов

float e_prev = 0;           //последнее значение разницы скорости движения
float I_prev = 0;           //последнее значение интегральной составляющей ПИД регулятора
float linear_speed;


int linear2driverMotor()
{
  if(linear_speed == 0){
    I_prev = 0.0;
    e_prev = 0.0;
    return 0;
  }

  //Расчет средней скорости движения между публикациями
  float speed_actual;
  float e = speed_actual * WHEEL_DIAMETER/2 - linear_speed;       //разница в скорости средней от последней публикации в m/s и желаемая m/s

  //ПИД регулятор для рассчета значения для драйвера моторов
  float P = pid_constants[0] * e;
  float I = I_prev + pid_constants[1] * e;
  float D = pid_constants[2] * (e - e_prev);
  float motor_value = round(P + I + D);

  if((motor_value < 0 && linear_speed < 0) || (motor_value > 0 && linear_speed > 0))
    motor_value = 0;


  //Для отладки
  //state_eff[1] = e;
  //state_eff[2] = motor_value;

  I_prev = I;                     //фиксируем интегральную составляющую
  e_prev = e;                     //фиксируем последнее значение разницы в скорости

  int motor_val_min = 45;

  if(motor_value < 0 && motor_value >= -motor_val_min){
    motor_value = -motor_val_min;
  }
  if(motor_value > 0 && motor_value <= motor_val_min){
    motor_value = motor_val_min;
  }

  //Убираем переполнение ШИМ
  if (motor_value>255){
    return 255;
  }

  if (motor_value<-255){
    return -255;
  }

  return motor_value;
}

void leftCmd_velCallback(const geometry_msgs::Twist& msg_cmd_vel)
{
  //float x = msg_cmd_vel.linear.x;
  //ROS_INFO("LINEAR: [%f], ANGULAR: [%f]", msg_cmd_vel.linear.x, msg_cmd_vel.angular.z);
}

void rightCmd_velCallback(const geometry_msgs::Twist& msg_cmd_vel)
{
  //float x = msg_cmd_vel.linear.x;
  //ROS_INFO("LINEAR: [%f], ANGULAR: [%f]", msg_cmd_vel.linear.x, msg_cmd_vel.angular.z);
}


void leftWheelSpeedCallback(const geometry_msgs::Twist& msg_cmd_vel)
{
  //float x = msg_cmd_vel.linear.x;
  //ROS_INFO("LINEAR: [%f], ANGULAR: [%f]", msg_cmd_vel.linear.x, msg_cmd_vel.angular.z);
}

void rightWheelSpeedCallback(const geometry_msgs::Twist& msg_cmd_vel)
{
  //float x = msg_cmd_vel.linear.x;
  //ROS_INFO("LINEAR: [%f], ANGULAR: [%f]", msg_cmd_vel.linear.x, msg_cmd_vel.angular.z);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "speed_regulator_node");
    ros::NodeHandle nh;

    //Sensors speed
    ros::Subscriber left_wheel_speed_sub = nh.subscribe("green_robot/sensors/wheel_speed/left", 50, leftWheelSpeedCallback);
    ros::Subscriber right_wheel_speed_sub = nh.subscribe("green_robot/sensors/wheel_speed/right", 50, rightWheelSpeedCallback);

    //Speed cmd_vel
    ros::Subscriber left_cmd_vel_sub = nh.subscribe("green_robot/cmd_vel/left", 50, leftCmd_velCallback);
    ros::Subscriber right_cmd_vel_sub = nh.subscribe("green_robot/cmd_vel/left", 50, rightCmd_velCallback);

    //Signal on motors
    ros::Publisher left_motor_pub = nh.advertise<std_msgs::Int32>("green_robot/motor/left", 50);
    ros::Publisher right_motor_pub = nh.advertise<std_msgs::Int32>("green_robot/motor/right", 50);

    ROS_INFO("It's not an Eastern Egg!");
}
