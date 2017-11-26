#include "ros/ros.h"
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/PointCloud.h>
#include "yeti_snowplow/obstacles.h"
#include "yeti_snowplow/obstacle.h"
#include "yeti_snowplow/robot_position.h"
#include "yeti_snowplow/location_point.h"
#include "yeti_snowplow/lidar_point.h"
#include "yeti_snowplow/turn.h"
#include "yeti_snowplow/target"
#include "buffer.cpp"
#include <math.h>
#include <vector>
#include <algorithm>

class ObstacleReaction
{
public:
	// void obstaclePositionsCallback(const yeti_snowplow::obstacles::ConstPtr& groupOfObstacles)
	// {
	// 	obstacles = groupOfObstacles->obstacles;
	// }
	ObstacleReaction()
	{
		wayPointID = 0;

		turnPub = n.advertise<yeti_snowplow::turn>("obstacle_reaction_turn", 1000);
		obstaclePositionsSub = n.subscribe("obstacles", 1000, obstaclePositionsCallback);
		robotPositionSub = n.subscribe("robot_position", 1000, &obstacleReaction::robotPositionCallback, this);
		waypointClient = n.serviceClient<yeti::target>("waypoint");
		scanSub = n.subscribe("obstacles", 1000, &obstacleReaction::scanCallback,this);
		speedPub  = n.advertise<geometry::Twist>("motor_speed", 1000);
		
		nh.param("maximum_navigation_speed", maxSpeed, 0.7);
		nh.param("obstacle_return_boost", turnBoost, -1.2);
		nh.param("obstacle_reaction_reverse_speed", reverseSpeed, -0.5);

		
	}
	void robotPositionCallback(const geometry::Pose2D::ConstPtr& robotPosition)
	{
		robotLocation.x = robotPosition->x;
		robotLocation.y = robotPosition->y;
		robotLocation.theta = robotPosition.theta;
	}

	void getNextWaypoint()
	{
		yeti::target wayPointInfo;
		wayPointInfo.request.ID = wayPointID; 
		

		if(waypointClient.call(wayPointInfo))
		{
			ROS_INFO("Calling waypoint service");
			dir = wayPointInfo.response.waypoint.dir;
			navSpeed = wayPointInfo.response.waypoint.speed;
			nextWayPoint = wayPointInfo.response.waypoint.location;
		}
		else{
			ROS_INFO("Failed to call service")
		}

		wayPointID++;
	}

	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scannedData)
	{
		/* This fires every time a new obstacle scan is published */
		//location contains an array of points, which contains an x and a y relative to the robot
	
		//Projecting LaserScanData to PointCloudData
		laser_geometry::LaserProjection projector_;
		sensor_msgs::PointCloud cloudData;
		projector_.projectLaser(*scannedData, cloudData);
		lidarData = cloudData.points;
	}

	void obstacleReactance()
	{
		Buffer buffer;
		geometry::msgs msg;
		yeti::turn turnMsg;
		getNextWaypoint();
		buffer.combinedUpdatePoints(lidarData);
		//Creates List of LiDAR points which have a positive Y value, and are within the Buffer distance threshold
		
		//look at angle needed to go to target waypoint, if there is an obstacle in the way, then find what turn angle is needed to avoid it to the right. 
		double rightAngle = buffer.combinedRightWheelScan(nextWayPoint);
		//look at angle needed to go to target waypoint, if there is an obstacle in the way, then find what turn angle is needed to avoid it to the left. 
		double leftAngle = buffer.combinedLeftWheelScan(nextWayPoint);

		
		for(obstacle object : obstacles)
		{
			if(object.isAMovingObstacle)
			{
				movingObstacleDetected = true;
				break;
			}
			else
			movingObstacleDetected = false;
		}

		if(movingObstacleDetected)
		{
			speed = 0;
			turn = 0;
			leftSpeed = 0;
			rightspeed = 0;
		}

		else if(rightAngle == 0 && leftAngle == 0)
		{
			//turn unchanged
			speed = 1 / (1 + 1 * abs(turn)) * (double)dir;
            speed = (double)turn * min(abs(speed), 1.0);
            lSpeed = (float)((speed + turnBoost * turn) * maxSpeed * navSpeed);//controlvarspeed is read in from text file, and limits speed by a percentage
            rSpeed = (float)((speed - turnBoost * turn) * maxSpeed * navSpeed);
		}
		else if (rightAngle == buffer.DOOM && leftAngle == buffer.DOOM )//There is no way to avoid anything to the left or the right, so back up.
        {
            leftSpeed = reverseSpeed * (float)maxSpeed * (float) .25;
            rightSpeed = reverseSpeed * (float)maxSpeed * (float) .25;
            ROS_INFO("I reached DOOM!");
        }
		else
		{
			if(abs(rightAngle - turn) <= abs(leftAngle - turn))
            {
                //move right of obstacle
                turn = rightAngle;
            }
            else if (abs(rightAngle - turn) > abs(leftAngle - turn))
            {
                //move Left of obstacle
                turn = leftAngle;
            }
            //speed slower as turn steeper 
            speed = 1 / (1 + 1 * abs(turn)) * (double)dir;
            speed = (double)dir * min(abs(speed), 1.0);
            leftSpeed = (float)((speed + turnBoost * .5 *  turn) * maxSpeed  * navSpeed);
            rightSpeed = (float)((speed - turnBoost * .5* turn) * maxSpeed *   navSpeed);
		}

		msg.linear.x = leftSpeed;
		msg.linear.y = rightSpeed;
		speedPub.publish(msg);
		turnMsg.angle = turn;
		turnMsg.stop = movingObstacleDetected;
		turnPub.publish(turnMsg);

	}
private:
//ROS Variables
	ros::NodeHandle n;
	ros::Publisher turnPub;
	//ros::Subscriber obstaclePositionsSub ;
	ros::Subscriber robotPositionSub;
	ros::ServiceClient waypointClient;
	ros::Subscriber scanSub;
	ros::Publisher speedPub;
	vector<obstacle> obstacles;
	vector<obstacle> obstaclesInFront;
	vector<geometry_msgs::Point32> lidarData;
	geometry::Pose2D robotLocation;
	double rightAngle; 
	double leftAngle;
	double leftSpeed;
	double rightSpeed;
	location_point nextWayPoint;
	double navSpeed;
	double speed;
	double turn;
	double turnBoost;
	double maxSpeed;
	double reverseSpeed;
	bool movingObstacleDetected;
	int dir;
	int wayPointID;
};

int main(int argc, char **argv){
	ros::init(argc, argv, "obstacle_reaction");

	ObstacleReaction obstacleReaction;
	
	while(ros::ok())
	{
		ros::spin();
		obstacleReaction.obstacleReactance();
	}
	
	
	return 0;
}