#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>


#define MOTOR_MAX_VALUE           255
#define MOTOR_START_VALUE         20

#define def_Kp                        80.0     // пропорциональный коэффициент для ПИД регулятора (41.7)
#define def_Ki                        0.0      // интегральный коэффициент для ПИД регулятора
#define def_Kd                        0.0      // дифференциальный коэффициент для ПИД регулятора

class PID{
private:
    float kP;
    float kI;
    float kD;

    int motor_max_value;
    int motor_start_value;

    double e_prev;           //последнее значение разницы скорости движения
    double I_prev;           //последнее значение интегральной составляющей ПИД регулятора

    double angular_speed_actual;
    double angular_speed_cmd_vel;

    ros::Subscriber wheel_speed_sub;
    ros::Subscriber cmd_vel_sub;
    ros::Publisher motor_pub;
    ros::NodeHandle nh;

    void cmd_velCallback(const geometry_msgs::Twist& msg)
    {
        angular_speed_cmd_vel = msg.angular.x;
    }

    void wheelSpeedCallback(const geometry_msgs::Twist& msg)
    {
        angular_speed_actual = msg.angular.x;
    }

    // ПИД-регулятор
    int motorsPID(double speed_control, double speed_actual)
    {
        // при управляющем воздействии равным нулю фиксируем составляющие на текущем шаге и возвращаем управляющее возжействие равным нулю
        if (speed_control == 0.0) {
            I_prev = 0.0;
            e_prev = 0.0;
            return 0;
        }

        // расчет ошибки между требуемой скоростью и фактической
        double e = speed_control - speed_actual;          //разница в скорости текущая в m/s и желаемая m/s

        // ПИД регулятор для рассчета значения для драйвера моторов
        double P = kP * e;
        double I = I_prev + kI * e;
        double D = kD * (e - e_prev);
        double value = round(P + I + D);

        I_prev = I;            //фиксируем интегральную составляющую
        e_prev = e;            //фиксируем последнее значение разницы в скорости

        if(value < 0 && value >= -motor_start_value){
            value = -motor_start_value;
        }
        if(value > 0 && value <= motor_start_value){
            value = motor_start_value;
        }

        //Убираем переполнение ШИМ
        if (value>motor_max_value){
            return motor_max_value;
        }

        if (value<-motor_max_value){
            return -motor_max_value;
        }

        return value;
    }

public:
    PID(std::string topic_wheel_speed, std::string topic_cmd_vel, std::string topic_motor, ros::NodeHandle &n, int motor_max_value, int motor_start_value){
        nh = n;
        this->motor_max_value = motor_max_value;
        this->motor_start_value = motor_start_value;
        wheel_speed_sub = nh.subscribe(topic_wheel_speed, 50, &PID::wheelSpeedCallback, this);
        cmd_vel_sub = nh.subscribe(topic_cmd_vel, 50, &PID::cmd_velCallback, this);
        motor_pub = nh.advertise<std_msgs::Int32>(topic_motor, 50);
        e_prev = 0.0;
        I_prev = 0.0;
        kP = def_Kp;
        kI = def_Ki;
        kD = def_Kd;
    }

    void update_pid_params(ros::NodeHandle &n, std::string pid_params)
    {
        std::vector<double> pid_constants;
        if (n.getParam(pid_params, pid_constants)){
            if(pid_constants.size() == 3) {
                kP = pid_constants[0];
                kI = pid_constants[1];
                kD = pid_constants[2];
            }
        }
    }

    void publish(){
        std_msgs::Int32 msg;
        msg.data = motorsPID(angular_speed_cmd_vel, angular_speed_actual);
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
    //    left_wheel.update_pid_params(priv_nh, pid_params);
    //    right_wheel.update_pid_params(priv_nh, pid_params);

    ros::Rate r(10);

    while(ros::ok()){
        ros::spinOnce();

        left_wheel.update_pid_params(priv_nh, pid_params);
        right_wheel.update_pid_params(priv_nh, pid_params);

        left_wheel.publish();
        right_wheel.publish();

        r.sleep();
    }
    return(0);
}
