#include <ros/ros.h>
#include <iostream>
#include <deque>
#include <queue>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/PointCloud.h>
#include "boost/date_time/posix_time/posix_time.hpp"
using namespace boost::posix_time;
using namespace std;

class StallDetection
{
    private:
    ros::NodeHandle n;
	ros::Publisher stallVelocityPub;
    ros::Publisher navigationPIDStatusPub;
	ros::Subscriber robotCurrentVelocitySub;
    ros::Subscriber expectedVelocitySub;
    ros::Subscriber actualVelocitySub;
    int queueSize;
    deque<geometry_msgs::Pose2D> robotVelocityHistory;
    bool disableNavigationPID;
    ptime lastStuckTime;
    
    //Initialize tolerance thresholds and time saving for Slip detection
    double noMovementTolerance;// 10 cm
    double noRotationTolerance; //5 degrees 
    //DateTime lastStuckTime; //save last stuck time
    //TimeSpan reverseDuration = new TimeSpan(0, 0, 0, 0, 800); // 0.5 Seconds
    float reverseSpeed;
    //Private Functions
    double findMinElement(string returnType, deque<geometry_msgs::Pose2D> history)
    {
        if(returnType == "x")
        {
            std::deque<int>::iterator it = std::min_element( robotVelocityHistory.begin(), robotVelocityHistory.end(),
                             []( const geometry_msgs::Twist &a, const geometry_msgs::Twist &b )
                             {
                                 return a.x < b.x;
                             } ); 
                return *it.x;
        }
        else if (returnType == "y")
        {
            std::deque<int>::iterator it = std::min_element( robotVelocityHistory.begin(), robotVelocityHistory.end(),
                             []( const geometry_msgs::Twist &a, const geometry_msgs::Twist &b )
                             {
                                 return a.y < b.y;
                             } );
                return *it.y;
        }
        else if(retrunType = "theta")
        {
            std::deque<int>::iterator it = std::max_element( robotVelocityHistory.begin(), robotVelocityHistory.end(),
                             []( const geometry_msgs::Twist &a, const geometry_msgs::Twist &b )
                             {
                                 return a.theta < b.theta;
                             } );
                return *it.theta;
        }
        else
            return 0.0;

    }
    double findMaxElement(string returnType)
    {
        if(returnType == "x")
        {
            std::deque<int>::iterator it = std::max_element( robotVelocityHistory.begin(), robotVelocityHistory.end(),
                             []( const geometry_msgs::Twist &a, const geometry_msgs::Twist &b )
                             {
                                 return a.x < b.x;
                             } ); 
                return *it.x;
        }
        else if (returnType == "y")
        {
            std::deque<int>::iterator it = std::max_element( robotVelocityHistory.begin(), robotVelocityHistory.end(),
                             []( const geometry_msgs::Twist &a, const geometry_msgs::Twist &b )
                             {
                                 return a.y < b.y;
                             } );
                return *it.y;
        }
        else if(retrunType = "theta")
        {
            std::deque<int>::iterator it = std::max_element( robotVelocityHistory.begin(), robotVelocityHistory.end(),
                             []( const geometry_msgs::Twist &a, const geometry_msgs::Twist &b )
                             {
                                 return a.theta < b.theta;
                             } );
                return *it.theta;
        }
        else
            return 0.0;
    }

    void robotPositionCallback(const geometry_msgs::Pose2D::ConstPtr& robotLocation)
    {
        addVelocityToHistory(*robotLocation);
    }

    void expectedRobotVelocityCallback(const geometry_msgs::Twist::ConstPtr& scannedData)
    {

    }
    public:
    StallDetection()
    {
        robotCurrentVelocitySub = n.subscribe("/localization/robot_location", 100, &StallDetection::robotPositionCallback, this);
        actualVelocitySub = n.subscribe("/localization/velocity", 100, &StallDetection::expectedRobotVelocityCallback, this);
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
    
    void activateStallDetection()
    {
        auto minX = findMinElement("x");
        auto maxX = findMaxElement("x");
        auto minY = findMinElement("y");
        auto maxY = findMaxElement("y";)
        auto minHeading = findMinElement("theta");
        auto maxHeading = findMaxElement("theta");

        if ((pow(maxX - minX, 2) + pow(maxY - minY, 2) < noMovementTolerance) 
                            && maxHeading - minHeading < noRotationTolerance && TargetLocation.location.id > 1 && !movingObstacles)
                {
                    lastStuckTime = second_clock::local_time();//save the time which slipping was detected 
                    YetiHistory.clear();// clear the QUeue so that stall detection cannot occur again until the queue is full
                }
    }

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "stall_detection");
	ObstacleReaction obstacleReaction;
	
	while(ros::ok())
	{
		
	}
	
	
	return 0;
}
