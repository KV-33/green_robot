// подключение библиотек
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>

// константы для конфигурации
#define LEFT_ENCODER_PIN                11      // пин направления движения левого энкодера (не используется)
#define RIGHT_ENCODER_PIN               12      // пин направления движения правого энкодера (не используется)
#define LEFT_ENCODER_INTERRUPT_NB       0       // номер прерывания левого энкодера (pin 2)
#define RIGHT_ENCODER_INTERRUPT_NB      1       // номер прерывания правогоы энкодера (pin 3)

#define LEFT_MOTOR_PWM_PIN              5       // пин левого мотора (ШИМ-сигнал)
#define LEFT_MOTOR_DIR_PIN              4       // пин левого мотора (направления движения)
#define RIGHT_MOTOR_PWM_PIN             6       // пин правого мотора (ШИМ-сигнал)
#define RIGHT_MOTOR_DIR_PIN             7       // пин правого мотора (направления движения)

#define SENSOR_LEFT_PIN                 8       // пин датчика касания левый
#define SENSOR_CENTER_PIN               9       // пин датчика касания центральный
#define SENSOR_RIGHT_PIN                10      // пин датчика касания правый

#define COUNT_MOTORS                    2       // количество моторов, энкодеров
#define COUNT_TOUCH_SENSORS             3       // количество датчиков касания

#define TIME_PUB_INFO_ENCODERS_MS       20      // интервал для публикации сообщений с энкодеров в топик
#define TIME_PUB_INFO_TOUCH_SENSORS_MS  20      // интервал для публикации сообщений с датчиков касания в топик
#define TIME_NO_MSG_MS                  2000    // время ожидания восстановления связи при отсутствии сообщений с управляющим воздействием (далее стоп моторы)

#define MOTOR_VALUE_MAX                 255     // максимальное значение подаваемое на драйвер
#define MOTOR_VALUE_MIN                 20      // минимальное значение подаваемое на драйвер


// стороны робота
#define LEFT                        0
#define RIGHT                       1

///////////////////////////////////////////////////////////////////////////////////////

// объявление обработчиков прерываний с энкодеров
void callBackInterruptLeftEncoder();
void callBackInterruptRightEncoder();

// объявление обработчиков сообщений из топиков предназначеных для управления моторами
void callBackCmdMotorLeft( const std_msgs::Int32& msg);
void callBackCmdMotorRight( const std_msgs::Int32& msg);

// объявление узла
ros::NodeHandle nh;

// объявление массивов для хранения и публикации данных с энкодеров в топик
char *encoders_names[COUNT_MOTORS] = {"wheel_left", "wheel_right"};
float encoders_pos[COUNT_MOTORS] = {0.0, 0.0};
float encoders_vel[COUNT_MOTORS] = {0.0, 0.0};
float encoders_eff[COUNT_MOTORS] = {0.0, 0.0};

// объявление массивов для хранения и публикации данных с датчиков касания в топик
char *sensors_touch_names[COUNT_TOUCH_SENSORS] = {"sensor_left", "sensor_center", "sensor_right"};
float sensors_touch_pos[COUNT_TOUCH_SENSORS] = {0.0, 0.0, 0.0};
float sensors_touch_vel[COUNT_TOUCH_SENSORS] = {0.0, 0.0, 0.0};
float sensors_touch_eff[COUNT_TOUCH_SENSORS] = {0.0, 0.0, 0.0};

float cmd_motors[COUNT_MOTORS] = {0.0, 0.0};                             // управляющие воздействия на моторы

// объявление переменных для учета последних действий
unsigned long time_last_pub_encoders = 0;                              // последнее время публикации значений энкодеров в топик
unsigned long time_last_pub_sensors_touch = 0;                         // последнее время публикации значений датчиков касания в топик
unsigned long time_last_msgs_cmd_vel = 0;                              // последнее время получения сообщения с управляющим воздействием

// обявление типов сообщений для энкодеров и датчиков касания
std_msgs::Int32 encoders_msg;
sensor_msgs::JointState sensors_touch_msg;

// объявление издателей топиков с данными от энкодеров и датчиков касания
ros::Publisher pub_encoders("sensors/encoders", &encoders_msg);
ros::Publisher pub_sensors_touch("sensors/touch", &sensors_touch_msg);

// объявление подписчиков на топики управления моторами
ros::Subscriber<std_msgs::Int32> sub_cmd_motor_left("motor/left", &callBackCmdMotorLeft );
ros::Subscriber<std_msgs::Int32> sub_cmd_motor_right("motor/right", &callBackCmdMotorRight );

//стандартная функция setup
void setup() {
    nh.getHardware()->setBaud(115200);       // задаем скорость обмена информацией по интефейсу serial
    nh.initNode();                           // инициализируем узел
    nh.advertise(pub_encoders);              // инициализируем издателя сообщений с энкодеров (публикатора топика)
    nh.advertise(pub_sensors_touch);         // инициализируем издателя сообщений с датчиков касания (публикатора топика)
    nh.subscribe(sub_cmd_motor_left);        // инициализируем подписчика на управляющее воздействие для левого мотора
    nh.subscribe(sub_cmd_motor_right);       // инициализируем подписчика на управляющее воздействие для правого мотора

    // подписываемся на прерывания (для подсчета количества импульсов с энкодеров)
    attachInterrupt(LEFT_ENCODER_INTERRUPT_NB,  callBackInterruptLeftEncoder, CHANGE);
    attachInterrupt(RIGHT_ENCODER_INTERRUPT_NB, callBackInterruptRightEncoder, CHANGE);

    // объявляем режимы работы для выводов микроконтроллера
    pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
    pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);

    pinMode(SENSOR_LEFT_PIN, INPUT);
    pinMode(SENSOR_CENTER_PIN, INPUT);
    pinMode(SENSOR_RIGHT_PIN, INPUT);

    // инициализируем сообщение для энкодеров
  /*  encoders_msg.header.frame_id =  "/encoders";
    encoders_msg.name_length = COUNT_MOTORS;
    encoders_msg.velocity_length = COUNT_MOTORS;
    encoders_msg.position_length = COUNT_MOTORS;
    encoders_msg.effort_length = COUNT_MOTORS;
    encoders_msg.name = encoders_names;
    encoders_msg.position = encoders_pos;
    encoders_msg.velocity = encoders_vel;
    encoders_msg.effort = encoders_eff;
    */
    // инициализируем сообщение для датчиков касания
    sensors_touch_msg.header.frame_id =  "/touch";
    sensors_touch_msg.name_length = COUNT_TOUCH_SENSORS;
    sensors_touch_msg.velocity_length = COUNT_TOUCH_SENSORS;
    sensors_touch_msg.position_length = COUNT_TOUCH_SENSORS;
    sensors_touch_msg.effort_length = COUNT_TOUCH_SENSORS;
    sensors_touch_msg.name = sensors_touch_names;
    sensors_touch_msg.position = sensors_touch_pos;
    sensors_touch_msg.velocity = sensors_touch_vel;
    sensors_touch_msg.effort = sensors_touch_eff;
}

//стандартная функция loop
void loop(){
    // Останавливаем робота если не приходят сообщения с управляющим воздействием
    if (millis() - time_last_msgs_cmd_vel > TIME_NO_MSG_MS) {
        cmd_motors[LEFT] = 0;
        cmd_motors[RIGHT] = 0;
    }

    // передаем управляющее воздействие на драйвер
    cmd_velMotors(cmd_motors[LEFT], LEFT);
    cmd_velMotors(cmd_motors[RIGHT], RIGHT);

    // проверяем нужно ли публиковать данные энкодеров
    if (millis() - time_last_pub_encoders > TIME_PUB_INFO_ENCODERS_MS) {
        encoders_msg.header.stamp = nh.now();                           // заполняем время сообщения
        pub_encoders.publish(&encoders_msg);                            // публикуем сообщение с значениями энкодеров

        time_last_pub_encoders = millis();                              // фиксируем время последней публикации значений энкодеров
    }

    // проверяем нужно ли публиковать данные датчиков касания
    if (millis() - time_last_pub_sensors_touch > TIME_PUB_INFO_TOUCH_SENSORS_MS) {
        pubInfoSensorTouch();                                           // опрашиваем и публикуем состояния датчиков касания

        time_last_pub_sensors_touch = millis();                         // фиксируем время последней публикации значений датчиков касания
    }
    nh.spinOnce();
}

// функция опроса датчиков касания и передачи данных в топик
void pubInfoSensorTouch(){
    sensors_touch_msg.header.stamp = nh.now();                         // заполняем поле времени в сообшении
    sensors_touch_msg.position[0] = !digitalRead(SENSOR_LEFT_PIN);     // опрашиваем левый датчик касания и записываем в сообщение инверсию значения
    sensors_touch_msg.position[1] = !digitalRead(SENSOR_CENTER_PIN);   // опрашиваем центральный датчик касания и записываем в сообщение инверсию значения
    sensors_touch_msg.position[2] = !digitalRead(SENSOR_RIGHT_PIN);    // опрашиваем правый датчик касания и записываем в сообщение инверсию значения

    pub_sensors_touch.publish(&sensors_touch_msg);                     // публикуем сообщение в топик
}

// обработчик полученного сообщения с управляющим воздействием для левого колеса
void callBackCmdMotorLeft( const std_msgs::Int32& msg){
    cmd_motors[LEFT] = msg.data;                                       // заполняем массив полученным значением в сообщении
    time_last_msgs_cmd_vel = millis();                                 // фиксируем время последнего управляющего воздействия
}

// обработчик полученного сообщения с управляющим воздействием для правого колеса
void callBackCmdMotorRight( const std_msgs::Int32& msg){
    cmd_motors[RIGHT] = msg.data;                                      // заполняем массив полученным значением в сообщении
    time_last_msgs_cmd_vel = millis();                                 // фиксируем время последнего управляющего воздействия
}

void callBackInterruptLeftEncoder(){
  double t = millis()/1000.0;
  if (t>timeOfLastChangeLeftEncoder){
    countLeftEncoder.data = countLeftEncoder.data + 1*leftWheelRotationDir;
    timeOfLastChangeLeftEncoder = t;
  }
}

void callBackInterruptRightEncoder(){
  double t = millis()/1000.0;
  if (t>timeOfLastChangeRightEncoder){
    countRightEncoder.data = countRightEncoder.data + 1*rightWheelRotationDir;
    timeOfLastChangeRightEncoder = t;
  }
}


// обработчик прерывания для левого колеса
/*inline void callBackInterruptLeftEncoder() {
    encoders_pos[LEFT] += getRotationDir(cmd_motors[LEFT]);            // получаем необходимое значение для суммирования с счетчиком  импульсов в зависимости от направления вращения мотора
}

// обработчик прерывания для правого колеса
inline void callBackInterruptRightEncoder() {
    encoders_pos[RIGHT] += getRotationDir(cmd_motors[RIGHT]);          // получаем необходимое значение для суммирования с счетчиком  импульсов в зависимости от направления вращения мотора
}
*/
// определение направления вращения мотора по последнему управляющему воздействию
float getRotationDir(int value) {
    if (value >= 0) {
        if (value == 0) {
            return 0.0;     // стоп
        }
        else
        {
            return 1.0;     // вращение вперед
        }
    }
    else
    {
        return -1.0;        // вращение назад
    }
}

// управление мотором на определенной стороне робота (1 пин определяет направление, 2 пин ШИМ-сигнал)
void cmd_velMotors(int value, int side) {
    // избавляемся от переполнения ШИМ
    if (value > MOTOR_VALUE_MAX)
        value = MOTOR_VALUE_MAX;
    if (value < -MOTOR_VALUE_MAX)
        value = -MOTOR_VALUE_MAX;

    // убираем значения ниже минимального значения при котором моторы могут вращаться
    if (value < 0 && value >= -MOTOR_VALUE_MIN)
        value = -MOTOR_VALUE_MIN;
    if (value > 0 && value <= MOTOR_VALUE_MIN)
        value = MOTOR_VALUE_MIN;

    // передаем значение ШИМ-сигнала на мотор

    analogWrite(side==LEFT ? LEFT_MOTOR_PWM_PIN : RIGHT_MOTOR_PWM_PIN, abs(value));

    // передаем значение определяющее направление вращения мотора
    if (value >= 0) {
        // вращение вперед или стоп
        digitalWrite(side==LEFT ? LEFT_MOTOR_DIR_PIN : RIGHT_MOTOR_DIR_PIN, side==LEFT ? HIGH : LOW);
    } else {
        // вращение назад
        digitalWrite(side==LEFT ? LEFT_MOTOR_DIR_PIN : RIGHT_MOTOR_DIR_PIN, side==LEFT ? LOW : HIGH);
    }
}
