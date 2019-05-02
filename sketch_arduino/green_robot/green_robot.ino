// Sketch for green_robot

// includes
#include <ros.h>
#include <green_robot/Int32Stamped.h>

// constants
#define WHEEL_DIAMETER              0.150
#define ANGULAR_RES                 0.314159265

#define LEFT_ENCODER_PIN            8  // no use
#define RIGHT_ENCODER_PIN           9  // no use
#define LEFT_ENCODER_INTERRUPT_NB   2
#define RIGHT_ENCODER_INTERRUPT_NB  3

#define LEFT_MOTOR_PWM_PIN          5  // motor A blue motorshield
#define LEFT_MOTOR_DIR_PIN          4
#define RIGHT_MOTOR_PWM_PIN         6  // motor B blue motorshield
#define RIGHT_MOTOR_DIR_PIN         7 


// declaration of callbacks interrupts of encoders
void callBackInterruptLeftEncoder();
void callBackInterruptRightEncoder();

// declaration of callbacks for cmd msg reception
void callBackCmdLeftMotor( const green_robot::Int32Stamped& msg); 
void callBackCmdRightMotor( const green_robot::Int32Stamped& msg);

// node
ros::NodeHandle nh;

// global variables
green_robot::Int32Stamped countLeftEncoder;
green_robot::Int32Stamped countRightEncoder;
green_robot::Int32Stamped ctrlLeftMotor;
green_robot::Int32Stamped ctrlRightMotor;
int leftWheelRotationDir = 1;                 // 0: stop, +1: forward, -1: backward
int rightWheelRotationDir = 1;                // 0: stop, +1: forward, -1: backward
double timeOfLastChangeLeftEncoder = 0.0;
double timeOfLastChangeRightEncoder = 0.0;

double publicationPeriodEncoders = 0.10;//0.05;
double timeOfLastPubEncoders = 0.0;

// publishers
ros::Publisher pubCountLeftEncoder("green_robot/countEncoder/left", &countLeftEncoder);
ros::Publisher pubCountRightEncoder("green_robot/countEncoder/right", &countRightEncoder);

// suscribers
ros::Subscriber<green_robot::Int32Stamped> subCmdLeftMotor("green_robot/cmdMotor/left", &callBackCmdLeftMotor );
ros::Subscriber<green_robot::Int32Stamped> subCmdRightMotor("green_robot/cmdMotor/right", &callBackCmdRightMotor );


void setup() {
 nh.initNode(); 
 nh.advertise(pubCountLeftEncoder);
 nh.advertise(pubCountRightEncoder);
 nh.subscribe(subCmdLeftMotor);
 nh.subscribe(subCmdRightMotor);
 
 attachInterrupt(LEFT_ENCODER_INTERRUPT_NB,  callBackInterruptLeftEncoder, CHANGE);
 attachInterrupt(RIGHT_ENCODER_INTERRUPT_NB, callBackInterruptRightEncoder, CHANGE);
 
 pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
 pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
 pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
 pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
}


void loop(){
 
 // Timer for encoder msg publication  
 if (  ( (millis()/1000.0) - timeOfLastPubEncoders ) > publicationPeriodEncoders) {

  // publish messages
  countLeftEncoder.header.stamp = nh.now();
  countRightEncoder.header.stamp = countLeftEncoder.header.stamp;
  pubCountLeftEncoder.publish(&countLeftEncoder);
  pubCountRightEncoder.publish(&countRightEncoder);
   
  // Reset timer
  // if timer expired several times
  // => reset to the integer part of the number of timer periods from last reset of the timer
  timeOfLastPubEncoders = timeOfLastPubEncoders + publicationPeriodEncoders*floor( (millis()/1000.0 - timeOfLastPubEncoders) / publicationPeriodEncoders); 
 }
 nh.spinOnce();   
}


// definition of callback functions
void callBackCmdLeftMotor( const green_robot::Int32Stamped& msg){
  int u = msg.data;
 
  // saturation
  if (u>255)
    u = 255;
  if (u<-255)
    u = -255;
	
  // write rotation speed module
  analogWrite(LEFT_MOTOR_PWM_PIN, abs(u));

  // write rotation speed direction
  if (u>=0) {
    digitalWrite(LEFT_MOTOR_DIR_PIN, LOW);  // forward or stop
    if (u==0){
      leftWheelRotationDir = 0;  // stop
    } else {
      leftWheelRotationDir = 1;  // forward
    }
  } else {
    digitalWrite(LEFT_MOTOR_DIR_PIN, HIGH);  // backward
    leftWheelRotationDir = -1;
  }
}

void callBackCmdRightMotor( const green_robot::Int32Stamped& msg){

  int u = msg.data;
  
  // saturation
  if (u>255)
    u = 255;
  if (u<-255)
    u = -255;
	
  // write rotation speed module
  analogWrite(RIGHT_MOTOR_PWM_PIN, abs(u));

  // write rotation speed direction
  if (u>=0) {
    digitalWrite(RIGHT_MOTOR_DIR_PIN, LOW);  // forward or stop
    if (u==0){
      rightWheelRotationDir = 0;  // stop
    } else {
      rightWheelRotationDir = 1;  // forward
    }
  } else {
    digitalWrite(RIGHT_MOTOR_DIR_PIN, HIGH);  // backward
    rightWheelRotationDir = -1;
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
