#include <iostream>
#include <math.h>
#include "ros/ros.h"
#include <vector>
using namespace std;

int main()
{
    
}
// # define M_PI           3.14159265358979323846  /* pi */
// static class Buffer
// {
// private:
//     static double combinedBufferWidth = 0.55; //0.425; // 425;
//     static double combinedBufferLength = 2.5; //2000.0; // the distance in which obstacle avoidance begins if an obstacle is within this distance. \
//     //Also defines the required distance away any obstacle must be for the robot to consider turning toward that direction

//     static double combinedHalfRobotWidth = 0.35; //350.0;// this is the width of the robot/2. Should algin with the outsides of the wheel.
//     static int badCount = 0;// temporary counter to hold the number of LiDAR points in the way of a turn angle. used in CombinedVectorSCan

//     //Sampling Thresholds
//     const double SAMPLING_DISTANCE = 0.1; //100.0;
//     const int BADCOUNTTHRESH = 5;// maximum number of LiDAR points associated with a turn angle to be considered not sage (IE if bad count > BADCOUNTTHRESH, then the robot will hit an obstacle. 
//     const double ANGLE_SAMPLES = 18;// the number of angles to try to avoid an obstacle
//     const double SAMPLING_ANGLE = M_PI /ANGLE_SAMPLES;// figures out the angle between each sample angle
// public:
//     const double DOOM = (-135.0) * 2.0 * Math.PI / 360.0;//this is bad....
//     static vector<location_point> combinedBufPoints = new vector<location_point>();//stores the list of LiDAR points which are within the buffer of the robot.
    
//     static double adjust_angle(double angle, double circle)//this function limits the angle between (-pi & pi) or (-180 & 180)
//     {   //circle = 2pi for radians, 360 for degrees
//         // Subtract multiples of circle
//         angle -= floor(angle / circle) * circle;
//         angle -= floor(2 * angle / circle) * circle;
//         return angle;
//     }
    
//     static double distanceCalculator()
//     {

//     }
// //     //this function checks a turn angle (provided by wheel scans) to see if there is anything in the way of the desired turn angle.
// //     //It returns true if the robot can turn that direction without hittinemmg an obstacle, and returns false if there is something in
// //     //the way of the entered turn angle.
// //     static bool combinedVectorScan(location_point source, location_point destination)
// //     {
// //         double target_angle = adjust_angle(Math.Atan2(destination.X - source.X, destination.Y - source.Y), 2.0*Math.PI); //limit angle between (-pi & pi) or (-180 & 180)
// //         double target_dist = destination.DistanceBetween(source); //find distance between corner of wheel to desired target. this equals Buffer length
// //         double dist;
// //         int i = 0, j = 0;//set up two iterators
// //         double tempDist = 0.0;

// //         location_point sample_point;// create temporary point which is updated each iteration
           
// //         do// find out if there is anything in the way of the 
// //         {
// //             sample_point.X = (i * SAMPLING_DISTANCE) * Math.Sin(target_angle);//create temporary XY point along the desired target angle at varying distances
// //             sample_point.Y = (i * SAMPLING_DISTANCE) * Math.Cos(target_angle);//create temporary XY point along the desired target angle at varying distances

// //             foreach (var cpoint in combinedBufPoints)//look through each point point in the buffer LiDAR points to see if any are by the temporary point
// //             {
// //                 if (cpoint.Y < 0) //if it's behind the robot, skip it
// //                 {
// //                     continue;
// //                 }
// //                 dist = sample_point.DistanceBetween(cpoint); //figure out distance between the temporary point and this point in the buffer LiDAR points
// //                 if (dist < combinedBufferWidth) // if the dcurrent LiDAR point is close to the temporary point
// //                 {
// //                     badCount++;//increment the number of LiDAR points in the way of this target angle
// //                     if (badCount > BADCOUNTTHRESH) //if there's too many lidar points in the way then this is not a valid turn angle
// //                     {
// //                         return false;//return false to indicate that this turn angle is not good.
// //                     }//end BAD COUNT THRESH
// //                 }//END dist<combinedBufferWidth
// //             }//end For each LiDAR point
// //             i++;
// //         } while (SAMPLING_DISTANCE * i < target_dist); // while the sampling distance along the target angle is less than the target distance

// //         return true;//return true when it is possible for the robot to take this turn angle without hitting obstacles
// //     }//end CombinedVectorScan
// // };