#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include "green_robot/Int32Stamped.h"
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
#define NUM_JOINTS                2



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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "wheel_pid_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("rawspeed/left", 30, linear2driverMotor);
    ros::Publisher pub = nh.advertise<green_robot::Int32Stamped>("green_robot/cmdMotor/left", 30);
    ROS_INFO("It's not an Eastern Egg!");
}
