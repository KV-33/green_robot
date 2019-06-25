// Sketch for green_robot

// includes
#include <ros.h>
#include <speed_conversion/Int32Stamped.h>
#include <green_robot/SensorBoolStamped.h>
#include <std_msgs/Int32.h>

// constants
#define LEFT_ENCODER_PIN            11  // no use
#define RIGHT_ENCODER_PIN           12  // no use
#define LEFT_ENCODER_INTERRUPT_NB   0  // pin 2
#define RIGHT_ENCODER_INTERRUPT_NB  1  // pin 3

#define LEFT_MOTOR_PWM_PIN          5  // motor A blue motorshield
#define LEFT_MOTOR_DIR_PIN          4
#define RIGHT_MOTOR_PWM_PIN         6  // motor B blue motorshield
#define RIGHT_MOTOR_DIR_PIN         7

#define SENSOR_LEFT_PIN             8
#define SENSOR_CENTER_PIN           9
#define SENSOR_RIGHT_PIN            10

#define LEFT           0
#define RIGHT          1

// declaration of callbacks interrupts of encoders
void callBackInterruptLeftEncoder();
void callBackInterruptRightEncoder();

// declaration of callbacks for cmd msg reception
void callBackCmdLeftMotor( const std_msgs::Int32& msg);
void callBackCmdRightMotor( const std_msgs::Int32& msg);

// node
ros::NodeHandle nh;

// global variables
speed_conversion::Int32Stamped countLeftEncoder;
speed_conversion::Int32Stamped countRightEncoder;
green_robot::SensorBoolStamped sensors;

int leftWheelRotationDir = 1;                 // 0: stop, +1: forward, -1: backward
int rightWheelRotationDir = 1;                // 0: stop, +1: forward, -1: backward
double timeOfLastChangeLeftEncoder = 0.0;
double timeOfLastChangeRightEncoder = 0.0;

double publicationPeriodEncoders = 0.10;//0.05;
double timeOfLastPubEncoders = 0.0;

// publishers
ros::Publisher pubCountLeftEncoder("green_robot/sensors/encoders/left", &countLeftEncoder);
ros::Publisher pubCountRightEncoder("green_robot/sensors/encoders/right", &countRightEncoder);
ros::Publisher pubSensorsInfo("green_robot/sensors/bumper", &sensors);

// suscribers
ros::Subscriber<std_msgs::Int32> subCmdLeftMotor("green_robot/cmd_vel/left", &callBackCmdLeftMotor );
ros::Subscriber<std_msgs::Int32> subCmdRightMotor("green_robot/cmd_vel/right", &callBackCmdRightMotor );


void setup() {
 nh.getHardware()->setBaud(115200);
 nh.initNode();
 nh.advertise(pubCountLeftEncoder);
 nh.advertise(pubCountRightEncoder);
 nh.advertise(pubSensorsInfo);
 nh.subscribe(subCmdLeftMotor);
 nh.subscribe(subCmdRightMotor);
 
 attachInterrupt(LEFT_ENCODER_INTERRUPT_NB,  callBackInterruptLeftEncoder, CHANGE);
 attachInterrupt(RIGHT_ENCODER_INTERRUPT_NB, callBackInterruptRightEncoder, CHANGE);
 
 pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
 pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
 pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
 pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);

 pinMode(SENSOR_LEFT_PIN, INPUT);
 pinMode(SENSOR_CENTER_PIN, INPUT);
 pinMode(SENSOR_RIGHT_PIN, INPUT);
}

void loop(){
 
 // Timer for encoder msg publication  
 if (  ( (millis()/1000.0) - timeOfLastPubEncoders ) > publicationPeriodEncoders) {

  // publish messages
  countLeftEncoder.header.stamp = nh.now();
  countRightEncoder.header.stamp = countLeftEncoder.header.stamp;
  pubCountLeftEncoder.publish(&countLeftEncoder);
  pubCountRightEncoder.publish(&countRightEncoder);
  countLeftEncoder.data = 0.0;
  countRightEncoder.data = 0.0;

  
  sensorsPubInfo();
   
  // Reset timer
  // if timer expired several times
  // => reset to the integer part of the number of timer periods from last reset of the timer
  timeOfLastPubEncoders = timeOfLastPubEncoders + publicationPeriodEncoders*floor( (millis()/1000.0 - timeOfLastPubEncoders) / publicationPeriodEncoders); 
 }
 nh.spinOnce();   
}

void sensorsPubInfo(){
  sensors.header.stamp = nh.now();
  sensors.left = !digitalRead(SENSOR_LEFT_PIN);
  sensors.center = !digitalRead(SENSOR_CENTER_PIN);
  sensors.right = !digitalRead(SENSOR_RIGHT_PIN);
  
  pubSensorsInfo.publish(&sensors);
  }

// definition of callback functions
void callBackCmdLeftMotor( const std_msgs::Int32& msg){
   leftWheelRotationDir = cmd_velMotors(msg.data, LEFT);
}

void callBackCmdRightMotor( const std_msgs::Int32& msg){
   rightWheelRotationDir = cmd_velMotors(msg.data, RIGHT);
}

int cmd_velMotors(int u, int direction){
  // saturation
  if (u>255)
    u = 255;
  if (u<-255)
    u = -255;

  // write rotation speed module
  analogWrite(direction==LEFT ? LEFT_MOTOR_PWM_PIN : RIGHT_MOTOR_PWM_PIN, abs(u));

  // write rotation speed direction
  if (u>=0) {
    digitalWrite(direction==LEFT ? LEFT_MOTOR_DIR_PIN : RIGHT_MOTOR_DIR_PIN, direction==LEFT ? HIGH : LOW);  // forward or stop
    if (u==0){
      return 0;  // stop
    } else {
      return 1;  // forward
    }
  } else {
    digitalWrite(direction==LEFT ? LEFT_MOTOR_DIR_PIN : RIGHT_MOTOR_DIR_PIN, direction==LEFT ? LOW : HIGH);  // backward
    return -1;
  }
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
