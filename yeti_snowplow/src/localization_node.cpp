//so we can use M_PI
#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
// #include <sensor_msgs/PointCloud.h>
#include <yeti_snowplow/obstacle.h>
#include <yeti_snowplow/obstacles.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <string>
#include <sstream>
#include <vector>

using namespace std;

ros::Publisher pub;//ROS publisher

// sensor_msgs::PointCloud landmarkLocationsTxt;//holds landmark locations from text file
geometry_msgs::Pose2D previousRobotLocation;//holds previous robot location
// static private LocationPoint previousRobotLocation = new LocationPoint();

//find the landmark locations, with respect to where Yeti was. 
// void scanLandmarks(sensor_msgs::PointCloud landmarkLocsTXT, sensor_msgs::PointCloud* scan_pt_cloud, geometry_msgs::Pose2D prev_robot_location)
// {
// 	for( int i=0; i < scan_pt_cloud->points.size(); i++)
// 	{
// 		//loop through points
// 	}
// }

//Used to parse strings. because C++ doesn't have built in string splitting http://stackoverflow.com/a/236803
// void split(const std::string &s, char delim, std::vector<std::string> &elems) 
// {
//     std::stringstream ss;
//     ss.str(s);
//     std::string item;
//     while (std::getline(ss, item, delim)) {
//         elems.push_back(item);
//     }
// }

// std::vector<std::string> split(const std::string &s, char delim) 
// {
//     std::vector<std::string> elems;
//     split(s, delim, elems);
//     return elems;
// }

//gater landmarks from text file. Text file is in ROS PARAM.
// sensor_msgs::PointCloud importLandMarks(string filename)
// {
// 	//setup necassry string parsing and file parameters
// 	string str;
// 	ifstream file;
// 	file.open(filename.c_str());
// 	ROS_INFO("Read file line: %s", str.c_str());

// 	int numLandmarks = 0;
// 	while(getline(file, str))//check to see how many landmarks there are
// 	{
// 		numLandmarks++;
// 	}
// 	ROS_INFO("There are %d landmarks. They are: ", numLandmarks);
// 	file.close();

// 	sensor_msgs::PointCloud importedLandmarkLocations;//allocate space for landmark points
// 	int landmarkNum=0;// initialize iterator
// 	file.open(filename.c_str());//reopen file

// 	while(getline(file, str))//loop through file and save landmark locations. 
// 	{
// 		//ROS_INFO("Read file line: %s", str.c_str());
// 		vector<string> lineFields = split(str, ' '); //x y direction PID speed
// 		//ROS_INFO("Line %d has %ld many fields.", landmarkNum, lineFields.size());
		
// 		if(lineFields.size() == 2) //ignore if too short or starts with "// "
// 		{ 
// 			geometry_msgs::Point32 landmarkPoint;
// 			landmarkPoint.x=atof(lineFields[0].c_str());
// 			landmarkPoint.y=atof(lineFields[1].c_str());
// 			// importedLandmarkLocations.points[landmarkNum].x = atof(lineFields[0].c_str());
// 			// importedLandmarkLocations.points[landmarkNum].y = atof(lineFields[1].c_str());
// 			ROS_INFO("Landmark %d: \tX: %f\tY:%f",landmarkNum, landmarkPoint.x, landmarkPoint.y);
// 			landmarkNum++;

// 			importedLandmarkLocations.points.push_back(landmarkPoint);
// 		}
// 	}
// 	file.close();

// 	return landmarkLocationsTxt;
// }

geometry_msgs::Pose2D DetermineRobotLocation_old(vector<yeti_snowplow::obstacle> CLM, geometry_msgs::Pose2D robotLocation, double tolerance, float Lspeed, float Rspeed, float minSpeed)
{
	const int JMAX = 15;

	const double mu = 0.1; // (Only read)

	double maxx = 0.25;
	double minx = 0.0001;
	double maxy = 0.25;
	double miny = 0.0001;
	double maxt = 5.0 * (M_PI / 180.0);
	double mint = 0.1 * (M_PI / 180.0);

	bool updateh = false;
	double H [3][3]; // Hessian of 2nd Derivatives
	double H_lm [3][3];
	double H_inv [3][3];  // Matrix Inverse of H_lm
	double x_err = 0;  // Gradient E [0]
	double y_err = 0;  // Gradient E [1] 
	double t_err = 0;  // Gradient E [2]
	double ex;  // Landmark error in X
	double ey;  // Landmark error in Y
	double dx;  // Robot Delta X
	double dy;  // Robot Delta Y 
	double dt;  // Robot Delta Theta
	double det;  // Determinant of H_lm

	//copy last location
	geometry_msgs::Pose2D thisRobotLocation;
	thisRobotLocation.x = robotLocation.x;
	thisRobotLocation.y = robotLocation.y;
	thisRobotLocation.theta = robotLocation.theta;
	
	double errsqrd = 0.0;  // E
	double derr = 0.0;
	double lasterrsqrd = 99999.0;//was 0.0
	double lambda = 10.0;

	/*Set Hessian To Zero*/
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			H[i][j] = H_lm[i][j] = H_inv[i][j] = 0;
		}
	}

	/*Initalize Diag*/
	for (int j = 0; j < JMAX; j++)
	{
		errsqrd = 0;
		derr = 0;

		if (j == 0)
		{
			updateh = true;

			//foreach (var currentLandmark in CLM)
			//{
			//    ex = currentLandmark.x - thisRobotLocation.x - currentLandmark.distance * sin(thisRobotLocation.theta + currentLandmark.theta);  // Formula 4
			//    ey = currentLandmark.y - thisRobotLocation.y - currentLandmark.distance * cos(thisRobotLocation.theta + currentLandmark.theta);  // Formula 5
			lasterrsqrd = 99999.0; //Math.Pow(ex, 2) + Math.Pow(ey, 2);  // Formula 3 (Inside Summation)
			//}
		}

		if (updateh)
		{
			for (int i = 0; i < 3; i++)
				for (int k = 0; k < 3; k++)
					H[i][k] = 0.0;

			H[0][0] = H[1][1] = H[2][2] = mu * CLM.size();  // Formula 21
			x_err = mu * CLM.size() * (previousRobotLocation.x - thisRobotLocation.x);
			y_err = mu * CLM.size() * (previousRobotLocation.y - thisRobotLocation.y);
			t_err = mu * CLM.size() * (previousRobotLocation.theta - thisRobotLocation.theta);

			for (auto&& currentLandmark : CLM)
			{
				/*First Derivative*/
				ex = currentLandmark.x - thisRobotLocation.x - 
					currentLandmark.distance * sin(thisRobotLocation.theta + currentLandmark.heading);  // Formula 4
				ey = currentLandmark.y - thisRobotLocation.y 
					- currentLandmark.distance * cos(thisRobotLocation.theta + currentLandmark.heading);  // Formula 5

				// currentLandmark.correctedX = currentLandmark.x - ex;
				// currentLandmark.correctedY = currentLandmark.y - ey;
				currentLandmark.x = currentLandmark.x - ex;
				currentLandmark.y = currentLandmark.y - ey;

				derr += abs(ex) +abs( ey);
				errsqrd += pow(ex, 2) + pow(ey, 2);  // Formula 3
				x_err += ex;
				y_err += ey;
				t_err += ex * (currentLandmark.distance * cos(thisRobotLocation.theta + currentLandmark.heading)) - ey * (currentLandmark.distance * sin(thisRobotLocation.theta + currentLandmark.heading));

				H[0][0] += 1;  // Formula 9.2, Formula 21
				H[0][1] += 0;  // Formula 10.2, Formula 21
				H[0][2] += (currentLandmark.distance * cos(thisRobotLocation.theta + currentLandmark.heading));  // Formula 11.2, Formula 22
				H[1][0] += 0;  // Formula 12.2, Formula 21
				H[1][1] += 1;  // Formula 13.2,, Formula 21
				H[1][2] += -(currentLandmark.distance * sin(thisRobotLocation.theta + currentLandmark.heading));  // Formula 14.2, Formula 23
				H[2][0] += (currentLandmark.distance * cos(thisRobotLocation.theta + currentLandmark.heading));  // Formula 15.2, Formula 22
				H[2][1] += -(currentLandmark.distance * sin(thisRobotLocation.theta + currentLandmark.heading));  // Formula 16.2, Formula 23
				H[2][2] += pow(currentLandmark.distance, 2);  // Formula 17.2, Formula 24
			}
		}

		for (int i = 0; i < 3; i++)
		{
			for (int k = 0; k < 3; k++)
			{
				H_lm[i][k] = H[i][k];

				if (i == k)
				{
					H_lm[i][k] += lambda * 1.0;
				}
			}
		}

		det = H_lm[0][0] * (H_lm[2][2] * H_lm[1][1] - H_lm[2][1] * H_lm[1][2]) -
			H_lm[1][0] * (H_lm[2][2] * H_lm[0][1] - H_lm[2][1] * H_lm[0][2]) +
			H_lm[2][0] * (H_lm[1][2] * H_lm[0][1] - H_lm[1][1] * H_lm[0][2]);


		// Find inverse of H_lm
		H_inv[0][0] = (H_lm[2][2] * H_lm[1][1] - H_lm[2][1] * H_lm[1][2]) / det;
		H_inv[0][1] = -(H_lm[2][2] * H_lm[0][1] - H_lm[2][1] * H_lm[0][2]) / det;
		H_inv[0][2] = (H_lm[1][2] * H_lm[0][1] - H_lm[1][1] * H_lm[0][2]) / det;
		H_inv[1][0] = -(H_lm[2][2] * H_lm[1][0] - H_lm[2][0] * H_lm[1][2]) / det;
		H_inv[1][1] = (H_lm[2][2] * H_lm[0][0] - H_lm[2][0] * H_lm[0][2]) / det;
		H_inv[1][2] = -(H_lm[1][2] * H_lm[0][0] - H_lm[1][0] * H_lm[0][2]) / det;
		H_inv[2][0] = (H_lm[2][1] * H_lm[1][0] - H_lm[2][0] * H_lm[1][1]) / det;
		H_inv[2][1] = -(H_lm[2][1] * H_lm[0][0] - H_lm[2][0] * H_lm[0][1]) / det;
		H_inv[2][2] = (H_lm[1][1] * H_lm[0][0] - H_lm[1][0] * H_lm[0][1]) / det;


		/* Update Here*/
		/* Hessian Inverse times Gradiant*/
		dx = (H_inv[0][0] * x_err) + (H_inv[0][1] * y_err) + (H_inv[0][2] * t_err);  // Formula 25 (After plus)
		dy = (H_inv[1][0] * x_err) + (H_inv[1][1] * y_err) + (H_inv[1][2] * t_err);  // Formula 25 (After plus)
		dt = (H_inv[2][0] * x_err) + (H_inv[2][1] * y_err) + (H_inv[2][2] * t_err);  // Formula 25 (After plus)


		if (abs(derr) < tolerance)
		{
			j = JMAX;  // break;
		}

		if (errsqrd < lasterrsqrd)
		{
			updateh = true;
			lambda /= 10;
			lasterrsqrd = errsqrd;
			thisRobotLocation.x += dx;  // Formula 25
			thisRobotLocation.y += dy;  // Formula 25
			thisRobotLocation.theta += dt;  // Formula 25
		}
		else
		{
			updateh = false;
			lambda *= 10;
		}

		//Console.WriteLine("x=%f\ty=%f\tt=%f\td=%02f\r", thisRobotLocation.x, thisRobotLocation.y, thisRobotLocation.theta, derr);
	}

	previousRobotLocation.x = thisRobotLocation.x;
	previousRobotLocation.y = thisRobotLocation.y;
	previousRobotLocation.theta = thisRobotLocation.theta; 

	return thisRobotLocation; // Skip the rest of stuff

	// geometry_msgs::Pose2D newRobotLocation = new geometry_msgs::Pose2D();
	// newRobotLocation.x = robotLocation.x;
	// newRobotLocation.x = robotLocation.y;
	// newRobotLocation.x = robotLocation.theta;
	// if (abs(Lspeed) > minSpeed && abs(Rspeed) > minSpeed)
	// {
	// 	double deltaX = thisRobotLocation.x - previousRobotLocation.x;
	// 	double deltaY = thisRobotLocation.y - previousRobotLocation.y;
	// 	double deltaHeading = thisRobotLocation.theta - previousRobotLocation.theta;
	// 	if (abs(thisRobotLocation.x - previousRobotLocation.x) < maxx && abs(thisRobotLocation.y - previousRobotLocation.y) < maxy && abs(thisRobotLocation.theta - previousRobotLocation.theta) < maxt)
	// 	{
	// 		if (abs(deltaX) > minx)
	// 		{
	// 	////        ////*rx = previousRobotLocation.x = thisRobotLocation.x; 
	// 			previousRobotLocation.x = thisRobotLocation.x; 
	// 			newRobotLocation.x = thisRobotLocation.x;
	// 		}


	// 		if (abs(deltaY) > miny)
	// 		{

	// 			////        ////*ry = previousRobotLocation.y = thisRobotLocation.y;
	// 			previousRobotLocation.y = thisRobotLocation.y;
	// 			newRobotLocation.y = thisRobotLocation.y;
	// 		}


	// 		if (abs(deltaHeading) > mint)
	// 		{
	// 	////        ////*rt = previousRobotLocation.theta = ADJUST_RADIANS(thisRobotLocation.theta);

	// 			double adjustedTT = ((thisRobotLocation.theta + Math.PI) % (2 * Math.PI)) - Math.PI; // Adjusting to fit between -pi and pi.
	// 			newRobotLocation.theta = adjustedTT;
	// 			previousRobotLocation.theta = adjustedTT;

	// 		}

	// 		return newRobotLocation;
	// 	}

	// }




	// return robotLocation; 
}

//finds landmarks, finds robot locations from landmark location, then publishes robot location
// void localizeCallBack (const sensor_msgs::PointCloud::ConstPtr& cloud_in)
// {
// 		int i = 0;
// 		//ROS_INFO_STREAM(cloud_in->points[i]);
// 		for(i=0 ; i <  cloud_in->channels.size(); i++)
// 		{
// 			ROS_INFO_STREAM("size of cloud is " << cloud_in->points.size());
// 			//ROS_INFO_STREAM(cloud_in->channels[i].name);
// 			ROS_INFO_STREAM("Point ["<<i<< "] - X: " << cloud_in->points[i].x << "\t Y: " << cloud_in->points[i].x);
// 		}
// }

void obstacleCallback (const yeti_snowplow::obstacles::ConstPtr& obstacles){
	//determine robot location or something
}

main (int argc, char** argv)
{
	ros::init (argc, argv, "localization");
	ros::NodeHandle nh;
	ROS_INFO_STREAM("Finding Landmarks, and Robot Position");

	//import landmark locations from text file
	// std::string landmarkLocationFile;
	// if (ros::param::get("landmarkLocationFile", landmarkLocationFile))
	// {
	// 	ROS_INFO("Using landmarkLocationFile %s", landmarkLocationFile.c_str());
	// }
	// landmarkLocationsTxt = importLandMarks(landmarkLocationFile);
	
	// Create a ROS subscriber for the pointcloud published by laser geometry
 	// ros::Subscriber scanSub;
	// scanSub = nh.subscribe("laser_scan_point_cloud", 1, localizeCallBack);
	ros::Subscriber obstacleSub = nh.subscribe("obstacles", 1, obstacleCallback);

 	// Create a ROS publisher for the output point cloud
	//  pub = nh.advertise<sensor_msgs::PointCloud>("robot_location", 1);
	pub = nh.advertise<geometry_msgs::Pose2D>("yeti/robot_location", 1);

	ros::spin();
	return 0;
}