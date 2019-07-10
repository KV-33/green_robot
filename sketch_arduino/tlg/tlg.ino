// includes
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>

// constants
#define LEFT_ENCODER_PIN            11  // no use
#define RIGHT_ENCODER_PIN           12  // no use
#define LEFT_ENCODER_INTERRUPT_NB   4  // pin 2
#define RIGHT_ENCODER_INTERRUPT_NB  5  // pin 3

#define LEFT_MOTOR_PWM_PIN          3  // motor A blue motorshield
#define LEFT_MOTOR_DIR_PIN          4
#define RIGHT_MOTOR_PWM_PIN         5  // motor B blue motorshield
#define RIGHT_MOTOR_DIR_PIN         6

#define POWER_FOR_SENSOR_LINE       23
#define SENSOR_LINE_LEFT            A0
#define SENSOR_LINE_RIGHT           A1

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

#define NUM_JOINTS                2

char *state_names[NUM_JOINTS] = {"left_wheel", "right_wheel"};
float state_pos[NUM_JOINTS] = {0, 0};
float state_vel[NUM_JOINTS] = {0, 0};
float state_eff[NUM_JOINTS] = {0, 0};

char *sensors_line_names[NUM_JOINTS] = {"left_sensor_line", "right_sensor_line"};
float sensors_line_pos[NUM_JOINTS] = {0, 0};
float sensors_line_vel[NUM_JOINTS] = {0, 0};
float sensors_line_eff[NUM_JOINTS] = {0, 0};

int leftWheelRotationDir = 1;                 // 0: stop, +1: forward, -1: backward
int rightWheelRotationDir = 1;                // 0: stop, +1: forward, -1: backward
double timeOfLastChangeLeftEncoder = 0.0;
double timeOfLastChangeRightEncoder = 0.0;

double publicationPeriodEncoders = 0.10;//0.05;
double timeOfLastPubEncoders = 0.0;

double publicationPeriodSensorsLine = 0.50;//0.05;
double timeOfLastPubSensorsLine = 0.0;


sensor_msgs::JointState state_msg;
ros::Publisher state_pub("sensors/encoders", &state_msg);                    //инициализация издателя топика "joint_states"

sensor_msgs::JointState sensor_line_msg;
ros::Publisher sensor_line_pub("sensors/line", &sensor_line_msg);                    //инициализация издателя топика "joint_states"

// suscribers
ros::Subscriber<std_msgs::Int32> subCmdLeftMotor("motor/left", &callBackCmdLeftMotor );
ros::Subscriber<std_msgs::Int32> subCmdRightMotor("motor/right", &callBackCmdRightMotor );

void setup() {
 pinMode(POWER_FOR_SENSOR_LINE, OUTPUT);
 digitalWrite(POWER_FOR_SENSOR_LINE, HIGH);
 
 nh.getHardware()->setBaud(115200);
 nh.initNode();
 nh.subscribe(subCmdLeftMotor);
 nh.subscribe(subCmdRightMotor);
 
 attachInterrupt(LEFT_ENCODER_INTERRUPT_NB,  callBackInterruptLeftEncoder, CHANGE);
 attachInterrupt(RIGHT_ENCODER_INTERRUPT_NB, callBackInterruptRightEncoder, CHANGE);
 
 pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
 pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
 pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
 pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);

 
  nh.advertise(state_pub);
  state_msg.header.frame_id =  "/driver_states";
  state_msg.name_length = NUM_JOINTS;
  state_msg.velocity_length = NUM_JOINTS;
  state_msg.position_length = NUM_JOINTS;
  state_msg.effort_length = NUM_JOINTS;
  state_msg.name = state_names;
  state_msg.position = state_pos;
  state_msg.velocity = state_vel;
  state_msg.effort = state_eff;

  nh.advertise(sensor_line_pub);
  sensor_line_msg.header.frame_id =  "/sensor_line";
  sensor_line_msg.name_length = NUM_JOINTS;
  sensor_line_msg.velocity_length = NUM_JOINTS;
  sensor_line_msg.position_length = NUM_JOINTS;
  sensor_line_msg.effort_length = NUM_JOINTS;
  sensor_line_msg.name = sensors_line_names;
  sensor_line_msg.position = sensors_line_pos;
  sensor_line_msg.velocity = sensors_line_vel;
  sensor_line_msg.effort = sensors_line_eff;
}

void loop(){
 if ( ( (millis()/1000.0) - timeOfLastPubSensorsLine ) > publicationPeriodSensorsLine) {   
  sensors_line_pos[0] = analogRead(SENSOR_LINE_LEFT);
  sensors_line_pos[1] = analogRead(SENSOR_LINE_RIGHT);
  sensor_line_msg.header.stamp = nh.now();
  sensor_line_pub.publish(&sensor_line_msg);
  timeOfLastPubSensorsLine = timeOfLastPubSensorsLine + publicationPeriodSensorsLine*floor( (millis()/1000.0 - timeOfLastPubSensorsLine) / publicationPeriodSensorsLine); 
 }

 // Timer for encoder msg publication  
 if (  ( (millis()/1000.0) - timeOfLastPubEncoders ) > publicationPeriodEncoders) {

  // publish messages
  state_msg.header.stamp = nh.now();
  state_pub.publish(&state_msg);

  // Reset timer
  // if timer expired several times
  // => reset to the integer part of the number of timer periods from last reset of the timer
  timeOfLastPubEncoders = timeOfLastPubEncoders + publicationPeriodEncoders*floor( (millis()/1000.0 - timeOfLastPubEncoders) / publicationPeriodEncoders); 
 }
 nh.spinOnce();   
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

  // write rotation speed direction
  if (u>=0) {
    if (u==0){
      analogWrite(direction==LEFT ? LEFT_MOTOR_PWM_PIN : RIGHT_MOTOR_PWM_PIN, 0);
      analogWrite(direction==LEFT ? LEFT_MOTOR_DIR_PIN : RIGHT_MOTOR_DIR_PIN, 0);
      return 0;  // stop
    } else {
      analogWrite(direction==LEFT ? LEFT_MOTOR_PWM_PIN : RIGHT_MOTOR_PWM_PIN, abs(u));
      analogWrite(direction==LEFT ? LEFT_MOTOR_DIR_PIN : RIGHT_MOTOR_DIR_PIN, 0);
      return 1;  // forward
    }
  } else {
    analogWrite(direction==LEFT ? LEFT_MOTOR_PWM_PIN : RIGHT_MOTOR_PWM_PIN, 0);
    analogWrite(direction==LEFT ? LEFT_MOTOR_DIR_PIN : RIGHT_MOTOR_DIR_PIN, abs(u));
    return -1;
  }
}



void callBackInterruptLeftEncoder(){
  double t = millis()/1000.0;
  if (t>timeOfLastChangeLeftEncoder){
    state_pos[0] = state_pos[0] + 1*leftWheelRotationDir;
    timeOfLastChangeLeftEncoder = t;
  } 
}

void callBackInterruptRightEncoder(){
  double t = millis()/1000.0;
  if (t>timeOfLastChangeRightEncoder){
    state_pos[1] = state_pos[1] + 1*rightWheelRotationDir;
    timeOfLastChangeRightEncoder = t;
  } 
}
