#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "yeti_snowplow/location_point.h"
#include "yeti_snowplow/target.h"
#include "yeti_snowplow/waypoint.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <string>
#include <time.h>
#include <vector>
using namespace std;

// double targdist;
// double targbearing;
// double front;
// double right;
// double speed;               // between -1 to 1
double turn;                // between -1 to 1
double pErr;
double lastpErr;
double kP;
double dErr;
double kD;
double iErr;
double kI;
// double lookAhead;

ros::Publisher pub;
geometry_msgs::Twist previousTargetVelocity;
geometry_msgs::Twist currentTargetVelocity;
geometry_msgs::Twist realVelocity;
double lastTime, thisTime;
double maxIntErr = 0.5;

double mathSign(double number){
	//Returns the number's sign
	//Equivalent to .NET's Math.Sign()
	//number>0 = 1
	//number=0 = 0
	//number<0 = -1
	if (number == 0){
		return 0;
	}
	else if (number > 0) {
		return 1;
	}
	else {
		return -1;
	}
}

double adjust_angle(double angle, double circle){
	//circle = 2pi for radians, 360 for degrees
	// Subtract multiples of circle
	angle -= floor(angle / circle) * circle;
	angle -= floor(2 * angle / circle) * circle;

	return angle;
}

void initPID(){
	lastTime = ((double)clock()) / CLOCKS_PER_SEC;
	pErr = iErr = dErr = 0;
}

void obstacleReactanceVelocityCallback(const geometry_msgs::Twist::ConstPtr& velocity){	
	/* This fires every time a new velocity is published */
	if(velocity->linear.x != currentTargetVelocity.linear.x || velocity->angular.z != currentTargetVelocity.angular.z){
		previousTargetVelocity = currentTargetVelocity;
		currentTargetVelocity = *velocity;
		initPID();
	}
}

void localizationVelocityCallback(const geometry_msgs::Twist::ConstPtr& velocity){
	realVelocity = *velocity;
}

void pid(){
	double heading = realVelocity.angular.z;//location->theta;
	int dir = mathSign(realVelocity.linear.x);//(int)currentTarget.dir;
	double dx, dy, s, c, dt;
	double desiredAngle;

	if (dir < 0){
		heading = heading - M_PI * mathSign(heading);
	}

	// dx = currentTarget.location.x - location->x;
	// dy = currentTarget.location.y - location->y;
	
	//FIND DISTANCE AND ANGLE TO DESTINATION
	// desired angle is the desired Heading the robot should have at this instance if it were to be facing the target.
	desiredAngle = adjust_angle(currentTargetVelocity.angular.z/*atan2(dx, dy)*/, 2.0*M_PI);

	//USED FOR WAYPOINT NAVIGATION
	// cvar.right = dx * c - dy * s;
	// cvar.front = dy * c + dx * s;
	// c = cos(heading); //find Cosine term of the robots heading
	// s = sin(heading); //find sine term of the robots heading

	thisTime = ((double)clock()) / CLOCKS_PER_SEC;
	dt = thisTime - lastTime;

	lastpErr = pErr;
	pErr = adjust_angle(heading - desiredAngle, 2.0 * M_PI);
	iErr = iErr + pErr * dt;
	iErr = mathSign(iErr) * fmin(abs(iErr), maxIntErr);

	if (dt != 0){
		dErr = (pErr - lastpErr) / dt;
	}
	if (cos(pErr) > 0.5){ // +-60 degrees
		kP = 0.5;
		turn = -(kP * sin(pErr) *2 + kI * iErr + kD * dErr);  // Nattu
	}
	else {
		turn = -0.5 * mathSign(pErr); //if you need to turnin place, then ignore PID
	}
	lastTime = thisTime;

	std_msgs::Float64 msg;
	msg.data = turn;
	pub.publish(msg);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "navigation_pid_turn");

	ros::NodeHandle n;

	pub = n.advertise<std_msgs::Float64>("/navigation/turn", 5);

	initPID();

	ros::Subscriber reactanceVelocitySub = n.subscribe("/obstacle_reactance/velocity", 5, obstacleReactanceVelocityCallback);
	ros::Subscriber localizationVelocitySub = n.subscribe("/localization/velocity", 5, localizationVelocityCallback);

	// ros::spin();
	ros::Rate loopRate(100); //Hz
	while(ros::ok()) {
		ros::spinOnce();
		pid();
		loopRate.sleep();
	}
	
	return 0;
}
