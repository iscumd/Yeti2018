#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <yeti_snowplow/robot_position.h>
#include <yeti_snowplow/obstacle.h>
#include <yeti_snowplow/obstacles.h>
  
using namespace std;

//Global Variables
ros::Publisher obstaclePub;

static double maxRadius = 10.0;
static double MM2M  = 0.001;
static int M2MM = 1000;
static double nonSeparationThresh = 200;
static int highThresh = 50;
static int lowThresh = 4;
static int forgiveCount = 3;
static int linkedCount;
static int sumOfPoints;
static int obsSizeNum;
static bool isAlreadyLinking = false;
static bool isThereAnObstacle = false;
static double sumOfHeadings = 0.0;
static double obstacleStopThreshold;
static double movingObstacleSize = 0.45;
static double movingObstacleThresh = 0.07;
static vector<geometry_msgs::Point32> lidarData;
vector<yeti_snowplow::lidar_point> lmsData;
vector<yeti_snowplow::obstacle> obstacles;

void clearState()
{
    sumOfPoints = 0;
    obsSizeNum = 0;
    linkedCount = 0;
    isThereAnObstacle = false;
    isAlreadyLinking = false;
    isThereAnObstacle = false;
}

void clearObstacles()
{
    obstacles.clear();
}

void linkPoint(double currPointDist,double currentTheta, double twoPointsDist)
{
    linkedCount += 1;
    sumOfPoints += currPointDist;
    obsSizeNum += twoPointsDist;
    sumOfHeadings += currentTheta;
}

double distanceCalculator(yeti_snowplow::lidar_point lidarPoint1, yeti_snowplow::lidar_point lidarPoint2)
{
    return sqrt(pow((lidarPoint2.x - lidarPoint1.x), 2)- pow((lidarPoint2.y - lidarPoint1.y), 2));
}

double distanceFromRobot(double lidarX, double lidarY)
{
    return sqrt(pow((lidarX - 0), 2)- pow((lidarY - 0), 2));
}
void convertPointCloudToClass()
{
    for(int i =0; i < lidarData.size(); i++)
    {
        yeti_snowplow::lidar_point lidar_point;
        lidar_point.x = lidarData[i].x;
        lidar_point.y = lidarData[i].y;
        lidar_point.theta = atan(lidar_point.y/lidar_point.x);
        if((lidar_point.x / cos(lidar_point.theta)) == (lidar_point.y / sin(lidar_point.theta)))
        lidar_point.distanceFromRobot = lidar_point.x / cos(lidar_point.theta);
        
        lmsData.push_back(lidar_point);
    }
}
void addAndAnalyzeObstacle(int lastLinkedIndex, yeti_snowplow::obstacle& obstacle)
{
    double index = (lastLinkedIndex - linkedCount) / 2;
    double mag = sumOfPoints / linkedCount;
    double avgTheta = sumOfHeadings / linkedCount;
    bool isOutsideTheField = false;
    // double robotPositionX = 0.0;
    // double robotPositionY = 0.0;
    //double theta = atan(robotPositionY / robotPositionX);
    obstacle.x = mag * cos(avgTheta);
    obstacle.y = mag * sin(avgTheta);

    if (mag < maxRadius || linkedCount > highThresh || linkedCount < lowThresh)
    {
        //Obstacle is not needed
        clearState();
    }
    else
    {
        //figure out if the object is within the plowing field or not
        if (obstacle.x > 4.75 || obstacle.x < -1.750 || obstacle.y > 11.75 || obstacle.y < -2.75)//check if obstacle is outside of Triple Ifield
        //if (obstacle.x > 1.75 || obstacle.x < -1.750 || obstacle.y > 11.75 || obstacle.y < -2.75)//check if obstacle is outside of Single Ifield
        { isOutsideTheField = true; }
        if(!isOutsideTheField)
        {
            obstacle.startPoint = lmsData[obstacle.objStartIndex];
            obstacle.endPoint = lmsData[obstacle.objEndIndex];
            
            obstacle.distance = mag;
            obstacle.obsRoughSize = obsSizeNum;
            obstacle.obsLineSize = distanceCalculator(obstacle.startPoint, obstacle.endPoint); 

            if(mag < 5)
            {
                if(abs(obstacle.obsLineSize - movingObstacleSize) < movingObstacleThresh)
                {
                    obstacle.isAMovingObstacle = true;
                    ROS_INFO("Moving Obstacle detected");
                }
                else
                {
                    obstacle.isAMovingObstacle = true;
                    ROS_INFO("Dynamic Obstacle detected");
                }
            }

            obstacles.push_back(obstacle);
        }
    }
    clearState();
}

void findObstacles()
{
    //Detect Obstacle
    clearObstacles();
    clearState();

    int j = 0;// forgive count variable
    for (int i = 360; i < lmsData.size() - 361; ++i)
    {
        yeti_snowplow::lidar_point currentPoint = lmsData[i];
        bool isPointLinked = false;
        yeti_snowplow::obstacle obstacle;
        if(currentPoint.distanceFromRobot < maxRadius)
        {
            for(j = 1; j <= forgiveCount; j++ )
            {
                yeti_snowplow::lidar_point nextPoint = lmsData[i + 1];
                double pointsDistance = distanceCalculator(currentPoint, nextPoint);
                if(pointsDistance < nonSeparationThresh * j * MM2M)
                {
                    linkPoint(currentPoint.distanceFromRobot, currentPoint.theta, pointsDistance);
                    isPointLinked = true;
                    if(!isAlreadyLinking)
                    { 
                        obstacle.objStartIndex = i;
                        isAlreadyLinking = true;
                    }
                    break;
                }
            }
        }
        if(isPointLinked == false)
        {
            if(isAlreadyLinking)
            {
                obstacle.objEndIndex = i;
                addAndAnalyzeObstacle(i, obstacle);
            }
            isAlreadyLinking = false;
            clearState();
        }
        else
        {
            i = i + j - 1;
            if(i > lmsData.size() - 361)
            {
                addAndAnalyzeObstacle(i, obstacle);
                clearState();
            }
        }
        
    }

    return;
}
void localizationCallback(const yeti_snowplow::robot_position::ConstPtr& position) 

{
    /* This fires every time a new robot position is published */
    //Not used for now

    return;
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
    convertPointCloudToClass();
    findObstacles();
}


// input: robot position relative to field
// input: array of lidar scan data
// output: array of obstacles (x,y,r(size), moving(boolean))

int main(int argc, char **argv){
	ros::init(argc, argv, "obstacle_detection");

	ros::NodeHandle n;

    ros::Subscriber localizationSub = n.subscribe("robot_location", 1, localizationCallback);

    ros::Subscriber scanSub = n.subscribe("scan", 1, scanCallback);

    ros::Publisher obstaclePub;//ROS obstacle publisher

    obstaclePub = n.advertise<yeti_snowplow::obstacles>("obstacles", 100);

    while(ros::ok())
    {
        //Publish all obstacles
        yeti_snowplow::obstacles msg;
        msg.obstacles = obstacles;
        obstaclePub.publish(msg);
    }
	ros::spin();
	
	return 0;
}
