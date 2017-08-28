#include<opencv2/core/core.hpp>
#include"opencv2/imgproc.hpp"
#include<opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include<iostream>
#include "blob.h"
#include <chrono>
#include <iomanip>

#pragma region Trackbar Functions
void Rect_Area_Max_Thresh(int, void*)
{
}
void Rect_Area_Min_Thresh(int, void*)
{
}
void Rect_Width_Max_Thresh(int, void*)
{
}
void Rect_Width_Min_Thresh(int, void*)
{
}
void Rect_Height_Max_Thresh(int, void*)
{
}
void Rect_Height_Min_Thresh(int, void*)
{
}
void Low_B_Thresh(int, void *)
{
}
void Low_G_Thresh(int, void *)
{
}
void Low_R_Thresh(int, void *)
{
}
void High_B_Thresh(int, void *)
{
}
void High_G_Thresh(int, void *)
{
}
void High_R_Thresh(int, void *)
{
}
#pragma endregion

const cv::Scalar COLOR_1 = cv::Scalar(255.0, 255.0, 255.0);
const cv::Scalar COLOR_2 = cv::Scalar(255.0, 0.0, 0.0);
const cv::Scalar COLOR_3 = cv::Scalar(0.0, 200.0, 0.0);
int maximum_Area = 1500, minimum_Area = 1000, area_Slider = 3000;
int maximum_Width = 10, minimum_Width = 0, maximum_Height = 10, minimum_Height = 0, sides_Slider = 100;
int Low_B = 0, Low_G = 0, Low_R = 100, High_B = 79, High_G = 65, High_R = 255;
int cam, recording;
cv::Mat input_From_Cam, Red_Blob;
cv::Mat structuring_Element_3x3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

int main(void)
{
	int window_Width = 640, window_Height = 480;
	std::cout << " camera select 0-internal, 1-external" << std::endl;
	std::cin >> cam;
	std::cout << "record? select 0-no recording, 1-recording" << std::endl;
	std::cin >> recording;

	cv::VideoWriter record_Video("Yeti_Run.avi", CV_FOURCC('D', 'I', 'V', 'X'), 60, cv::Size(window_Width, window_Height));
	cv::namedWindow("Colors", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Blob params", cv::WINDOW_AUTOSIZE);
	cv::VideoCapture webcam(cam);
	webcam.set(CV_CAP_PROP_FRAME_WIDTH, window_Width);
	webcam.set(CV_CAP_PROP_FRAME_HEIGHT, window_Height);

	//////////////////BLOBS///////////////////////////////////////////
	cv::createTrackbar("Area Max", "Blob params", &maximum_Area, area_Slider, Rect_Area_Max_Thresh);
	cv::createTrackbar("Area Min", "Blob params", &minimum_Area, area_Slider, Rect_Area_Min_Thresh);
	cv::createTrackbar("Width Max", "Blob params", &maximum_Width, sides_Slider, Rect_Width_Max_Thresh);
	cv::createTrackbar("Width Min", "Blob params", &minimum_Width, sides_Slider, Rect_Width_Min_Thresh);
	cv::createTrackbar("Height Max", "Blob params", &maximum_Height, sides_Slider, Rect_Height_Max_Thresh);
	cv::createTrackbar("Height Min", "Blob params", &minimum_Height, sides_Slider, Rect_Height_Min_Thresh);

	/////////////////COLORS///////////////////////////////////////////////
	cv::createTrackbar("Low Blue", "Colors", &Low_B, 255, Low_B_Thresh);
	cv::createTrackbar("High Blue", "Colors", &High_B, 255, High_B_Thresh);
	cv::createTrackbar("Low Green", "Colors", &Low_G, 255, Low_G_Thresh);
	cv::createTrackbar("High Green", "Colors", &High_G, 255, High_G_Thresh);
	cv::createTrackbar("Low Red", "Colors", &Low_R, 255, Low_R_Thresh);
	cv::createTrackbar("High Red", "Colors", &High_R, 255, High_R_Thresh);

	while ((char)cv::waitKey(16) != 'q') {
		webcam >> input_From_Cam;
		if (input_From_Cam.empty())
		{
			break;
		}
		if (recording)
		{
			record_Video.write(input_From_Cam);
		}

		std::vector<Blob> blobs;
		cv::inRange(input_From_Cam, cv::Scalar(Low_B, Low_G, Low_R), cv::Scalar(High_B, High_G, High_R), Red_Blob);
		cv::dilate(Red_Blob, Red_Blob, structuring_Element_3x3);
		cv::imshow("red", Red_Blob);
		std::vector<std::vector<cv::Point>> contours;
		cv::findContours(Red_Blob, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		cv::drawContours(Red_Blob, contours, -1, COLOR_1, -1);

		std::vector<std::vector<cv::Point> > convex_Hulls(contours.size());

		for (unsigned int i = 0; i < contours.size(); i++)
		{
			cv::convexHull(contours[i], convex_Hulls[i]);
		}

		for (auto &convexHull : convex_Hulls)
		{
			Blob possibleBlob(convexHull);

			if (possibleBlob.bounding_Rect.area() < maximum_Area && possibleBlob.bounding_Rect.area() > minimum_Area &&
				possibleBlob.dbl_Aspect_Ratio >= 0.5 && possibleBlob.dbl_Aspect_Ratio <= 1.2 &&
				possibleBlob.bounding_Rect.width < maximum_Width && possibleBlob.bounding_Rect.width > minimum_Width &&
				possibleBlob.bounding_Rect.height < maximum_Height && possibleBlob.bounding_Rect.height > minimum_Height &&
				possibleBlob.dbl_Diagonal_Size > 10)
			{
				blobs.push_back(possibleBlob);
				std::cout << std::setprecision(2) << possibleBlob.bounding_Rect.area() << " " << possibleBlob.dbl_Aspect_Ratio << " "
					<< possibleBlob.bounding_Rect.width << " " << possibleBlob.bounding_Rect.height << " " << possibleBlob.dbl_Diagonal_Size << std::endl;
			}
		}

		convex_Hulls.clear();

		for (auto &blob : blobs)
		{
			convex_Hulls.push_back(blob.contour);
			cv::drawContours(Red_Blob, convex_Hulls, -1, COLOR_1, -1);  // for each blob found
			cv::rectangle(input_From_Cam, blob.bounding_Rect, COLOR_2, 2);		 // draw a box around the blob
			cv::circle(input_From_Cam, blob.center_Position, 3, COLOR_3, -1);	 // draw a filled-in circle at the center
		}
		cv::imshow("Camera", input_From_Cam);
	}
	return(0);

}


