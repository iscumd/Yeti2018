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

class ObstacleReaction
{
public:
	// void obstaclePositionsCallback(const yeti_snowplow::obstacles::ConstPtr& groupOfObstacles)
	// {
	// 	obstacles = groupOfObstacles->obstacles;
	// }
	void robotPositionCallback(const geometry::Pose2D::ConstPtr& robotPosition)
	{
		robotLocation.x = robotPosition->x;
		robotLocation.y = robotPosition->y;
		robotLocation.theta = robotPosition.theta;
	}

	void nextWaypointCallback(const yeti_snowplow::waypoint::ConstPtr& nextWayPointInfo)
	{
		yeti::target wayPointInfo = nextWayPointInfo;
		turn = wayPointInfo->dir;
		navSpeed = wayPointInfo->speed;
		nextWayPoint = wayPointInfo->location;

		if(waypointClient.call(wayPointInfo))
		{
			ROS_INFO("Calling waypoint service");
		}
		else{
			ROS_INFO("Failed to call service")
		}
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
		Buffer.combinedUpdatePoints(lidarData);
		//Creates List of LiDAR points which have a positive Y value, and are within the Buffer distance threshold
		
		//look at angle needed to go to target waypoint, if there is an obstacle in the way, then find what turn angle is needed to avoid it to the right. 
		double Right = Buffer.combinedRightWheelScan(nextWayPoint);
		//look at angle needed to go to target waypoint, if there is an obstacle in the way, then find what turn angle is needed to avoid it to the left. 
		double Left = Buffer.combinedLeftWheelScan(nextWayPoint);

		
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
	}
private:
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
	bool movingObstacleDetected;
};

int main(int argc, char **argv){
	ros::init(argc, argv, "obstacle_reaction");


	ros::NodeHandle n;

	ros::Publisher turnPub;
	turnPub = n.advertise<yeti_snowplow::turn>("obstacle_reaction_turn", 1000);

	// ros::Subscriber obstaclePositionsSub = n.subscribe("obstacles", 1000, obstaclePositionsCallback);
	ros::Subscriber robotPositionSub = n.subscribe("robot_position", 1000, robotPositionCallback);
	ros::ServiceClient waypointClient = n.serviceClient<yeti::target>("waypoint");
	ros::Subscriber scanSub = n.subscribe("obstacles", 1000, scanCallback);

	ros::spin();
	
	return 0;
}