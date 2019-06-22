#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>


#define MOTOR_MAX_VALUE           255
#define MOTOR_START_VALUE         45

/*#define K_SPEED                   0.63     // коэффициент неадекватной скорости*/

#define Kp                        167.0    // пропорциональный коэффициент для ПИД регулятора (41.7)
#define Ki                        0.1      // интегральный коэффициент для ПИД регулятора
#define Kd                        0.4      // дифференциальный коэффициент для ПИД регулятора

class PID{
private:
    std::vector<double> pid_constants;

    int motor_max_value;
    int motor_start_value;

    double e_prev = 0;           //последнее значение разницы скорости движения
    double I_prev = 0;           //последнее значение интегральной составляющей ПИД регулятора

    double linear_speed_actual;
    double linear_speed_cmd_vel;

    ros::Subscriber wheel_speed_sub;
    ros::Subscriber cmd_vel_sub;
    ros::Publisher motor_pub;
    ros::NodeHandle nh;

    void cmd_velCallback(const geometry_msgs::Twist& msg)
    {
        linear_speed_cmd_vel = msg.linear.x;
    }

    void wheelSpeedCallback(const geometry_msgs::Twist& msg)
    {
        linear_speed_actual = msg.linear.x;
    }

    int linear2motor()
    {
      if(linear_speed_cmd_vel == 0){
        I_prev = 0.0;
        e_prev = 0.0;
        return 0;
      }

      //Расчет средней скорости движения между публикациями
      float e = linear_speed_actual - linear_speed_cmd_vel;       //разница в скорости от последней публикации в m/s и желаемая m/s

      //ПИД регулятор для рассчета значения для драйвера моторов
      float P = pid_constants.at(0) * e;
      float I = I_prev + pid_constants.at(1) * e;
      float D = pid_constants.at(2) * (e - e_prev);
      float motor_value = round(P + I + D);

      if((motor_value < 0 && linear_speed_cmd_vel < 0) || (motor_value > 0 && linear_speed_cmd_vel > 0))
        motor_value = 0;

      I_prev = I;                     //фиксируем интегральную составляющую
      e_prev = e;                     //фиксируем последнее значение разницы в скорости

      if(motor_value < 0 && motor_value >= -motor_start_value){
        motor_value = motor_start_value;
      }
      if(motor_value > 0 && motor_value <= motor_start_value){
        motor_value = -motor_start_value;
      }

      //Убираем переполнение ШИМ
      if (motor_value>motor_max_value){
        return -motor_max_value;
      }

      if (motor_value<-motor_max_value){
        return motor_max_value;
      }

      return motor_value;
    }

public:
    PID(std::string topic_wheel_speed, std::string topic_cmd_vel, std::string topic_motor, ros::NodeHandle &n, int motor_max_value, int motor_start_value){
        nh = n;
        this->motor_max_value = motor_max_value;
        this->motor_start_value = motor_start_value;
        wheel_speed_sub = nh.subscribe(topic_wheel_speed, 50, &PID::wheelSpeedCallback, this);
        cmd_vel_sub = nh.subscribe(topic_cmd_vel, 50, &PID::cmd_velCallback, this);
        motor_pub = nh.advertise<std_msgs::Int32>(topic_motor, 50);
    }

    void update_pid_params(ros::NodeHandle &n, std::string pid_params)
    {
        if (!n.getParam(pid_params, pid_constants)){
           //default values
           pid_constants.at(0) = Kp;
           pid_constants.at(1) = Ki;
           pid_constants.at(2) = Kd;
        }
    }

    void publish(){
        std_msgs::Int32 msg;
        msg.data = -linear2motor();
        motor_pub.publish(msg);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "speed_regulator_node");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    std::string topic_wheel_speed_left;
    std::string topic_cmd_vel_left;
    std::string topic_motor_left;
    priv_nh.param<std::string>("topic_wheel_speed_left", topic_wheel_speed_left, "sensors/wheel_speed/left");
    priv_nh.param<std::string>("topic_cmd_vel_left", topic_cmd_vel_left, "cmd_vel/left");
    priv_nh.param<std::string>("topic_motor_left", topic_motor_left, "motor/left");

    std::string topic_wheel_speed_right;
    std::string topic_cmd_vel_right;
    std::string topic_motor_right;
    priv_nh.param<std::string>("topic_wheel_speed_right", topic_wheel_speed_right, "sensors/wheel_speed/right");
    priv_nh.param<std::string>("topic_cmd_vel_right", topic_cmd_vel_right, "cmd_vel/right");
    priv_nh.param<std::string>("topic_motor_right", topic_motor_right, "motor/left");

    std::string pid_params;
    priv_nh.param<std::string>("pid_params", pid_params, "/pid");

    int motor_max_value;
    priv_nh.param("motor_max_value", motor_max_value, MOTOR_MAX_VALUE);
    int motor_start_value;
    priv_nh.param("motor_start_value", motor_start_value, MOTOR_START_VALUE);

    PID left_wheel(topic_wheel_speed_left, topic_cmd_vel_left, topic_motor_left, nh, motor_max_value, motor_start_value);
    PID right_wheel(topic_wheel_speed_right, topic_cmd_vel_right, topic_motor_right, nh, motor_max_value, motor_start_value);

    while(ros::ok()){
        ros::spinOnce();
        ros::Rate r(10);

        left_wheel.update_pid_params(priv_nh, pid_params);
        right_wheel.update_pid_params(priv_nh, pid_params);

        left_wheel.publish();
        right_wheel.publish();

        r.sleep();
    }
    return(0);
}
