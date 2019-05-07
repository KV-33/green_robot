#include "ros/ros.h"
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <green_robot/Int32Stamped.h>
#include <geometry_msgs/TwistStamped.h>

// wheel speed filter: compute angular (angular.z) and linear (linear.x) speed of wheel from encoder counts

// **** TO DO : to be passed as paramets in YAML
#define WHEEL_DIAMETER 0.065
#define ANGULAR_RES 0.314159265

// time instants
ros::Time t;
ros::Time tPrec;
bool firstIteration = true;

// counter and speed values
int count = 0;
int countPrec = 0;
double angularSpeed = 0.0;
double linearSpeed = 0.0;

// publisher
ros::Publisher pubWheelSpeed;

// message to be published
geometry_msgs::TwistStamped wheelSpeedMsg;


// call back for encoder counter of wheel
void callbackEncoderCounter(const ardupi_robot::Int32Stamped::ConstPtr& msg)
{
	// get time duration from last message
	t = msg->header.stamp;
	if (firstIteration) {
		tPrec = t;
	   firstIteration = false;
	}
	ros::Duration TeRos = t - tPrec;
	double Te = TeRos.toNSec() / 1.0e9;

	// get current counter number frome encoder
	count = msg->data;


	// *****  TO DO : filter on count nb and / or speeds
	
	// compute speed
	if (Te!=0) {
		angularSpeed = (count - countPrec)*ANGULAR_RES / Te;
		linearSpeed = angularSpeed * (WHEEL_DIAMETER / 2.0);
	} // else : last value
	
	
	// build message to be published
	wheelSpeedMsg.header.stamp = msg->header.stamp;//ros::Time::now();
	wheelSpeedMsg.twist.linear.x = linearSpeed;
	wheelSpeedMsg.twist.angular.z = angularSpeed;

	// publish msg
	pubWheelSpeed.publish(wheelSpeedMsg);

	// update time
	tPrec = t;
	countPrec = count;
   
}



int main(int argc, char **argv)
{
	// node
	ros::init(argc, argv, "wheelSpeedFilter");
	ros::NodeHandle n;

	// **** TO DO **** 
	// get parameters
	//	n.param("/ardupi_robot/angularRes",angularRes, 0.314159265);
	
	// published topic
	pubWheelSpeed = n.advertise<geometry_msgs::TwistStamped>("wheelSpeed", 10);

	// subscribed topic
	ros::Subscriber subEncoderCounter = n.subscribe("encoderCounter", 100, callbackEncoderCounter);

	// main loop
	ros::spin();

	return 0;
}
