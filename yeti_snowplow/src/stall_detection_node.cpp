#include <ros/ros.h>
#include <iostream>
#include <queue>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/PointCloud.h>

using namespace std;
class StallDetection
{
    private:
    ros::NodeHandle n;
	ros::Publisher stallVelocityPub;
    ros::Publisher navigationPIDStatusPub;
	ros::Subscriber robotCurrentVelocitySub;
    ros::Subscriber expectedVelocitySub;
    int queueSize;
    queue<geometry_msgs::Pose2D> robotVelocityHistory;
    bool disableNavigationPID;

    //Initialize tolerance thresholds and time saving for Slip detection
    double noMovementTolerance;// 10 cm
    double noRotationTolerance; //5 degrees
    //DateTime lastStuckTime; //save last stuck time
    //TimeSpan reverseDuration = new TimeSpan(0, 0, 0, 0, 800); // 0.5 Seconds
    float reverseSpeed;
    public:
    StallDetection()
    {
        robotCurrentVelocitySub = n.subscribe("/localization/robot_location", 100, &StallDetection::robotPositionCallback, this);
        expectedVelocitySub = n.subscribe("/localization/velocity", 100, &StallDetection::expectedRobotVelocityCallback, this);
		stallVelocityPub  = n.advertise<geometry_msgs::Twist>("/stall/velocity", 1000);
        navigationPIDStatusPub = n.advertise<geometry_msgs::Twist>("/navigation/disable", 1000)
        n.param("robot_velocity_queue_size", queueSize, 100);
        n.param("slip_detection_noMovementTolerance",noMovementTolerance, 0.1);// 10 cm
        n.param("slip_detection_noRotationTolerance" ,noRotationTolerance, 5.0); //5 degrees
        n.param("slip_detection_reverseSpeed", reverseSpeed, -0.5);

        robotHistory.resize(queueSize);
    } 

    void addVelocityToHistory(geometry_msgs::Pose2D robotCurrentVelocity)
    {
        if(robotVelocityHistory.size() > queueSize)
        {
            robotVelocityHistory.pop();
        }
        robotVelocityHistory.push(robotCurrentVelocity);

        if(robotVelocityHistory.size() >= queueSize)
        {
            activateStallDetection();
        }

    }
    void findMinElement()
    {

    }
    void findMaxElement()
    {

    }
    
    void activateStallDetection()
    {
        auto minX = std::min_element( robotVelocityHistory.begin(), robotVelocityHistory.end(),
                             []( const geometry_msgs::Twist &a, const geometry_msgs::Twist &b )
                             {
                                 return a.x < b.x;
                             } ); 
        auto maxX = std::max_element( robotVelocityHistory.begin(), robotVelocityHistory.end(),
                             []( const geometry_msgs::Twist &a, const geometry_msgs::Twist &b )
                             {
                                 return a.x < b.x;
                             } ); 
        auto minY = std::min_element( robotVelocityHistory.begin(), robotVelocityHistory.end(),
                             []( const geometry_msgs::Twist &a, const geometry_msgs::Twist &b )
                             {
                                 return a.y < b.y;
                             } ); 
        auto maxY = std::max_element( robotVelocityHistory.begin(), robotVelocityHistory.end(),
                             []( const geometry_msgs::Twist &a, const geometry_msgs::Twist &b )
                             {
                                 return a.y < b.y;
                             } ); 
        auto minHeading = std::min_element( robotVelocityHistory.begin(), robotVelocityHistory.end(),
                             []( const geometry_msgs::Twist &a, const geometry_msgs::Twist &b )
                             {
                                 return a.theta < b.theta;
                             } ); 
        auto maxHeading = std::max_element( robotVelocityHistory.begin(), robotVelocityHistory.end(),
                             []( const geometry_msgs::Twist &a, const geometry_msgs::Twist &b )
                             {
                                 return a.theta < b.theta;
                             } ); 

        if ((pow(maxX - minX, 2) + pow(maxY - minY, 2) < noMovementTolerance) 
                            && maxHeading - minHeading < noRotationTolerance && TargetLocation.location.id > 1 && !movingObstacles)
                {
                    lastStuckTime = DateTime.Now;//save the time which slipping was detected 
                    YetiHistory.Clear();// clear the QUeue so that stall detection cannot occur again until the queue is full
                }
    }

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "stall_detection_node");
	ObstacleReaction obstacleReaction;
	
	while(ros::ok())
	{
		
	}
	
	
	return 0;
}
