#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <stdlib.h>
#include<iostream>
#include "Blob.h"
#include <chrono>

#pragma region Trackbar function prototypes
void low_B_thresh_trackbar(int, void*);
void low_G_thresh_trackbar(int, void*);
void low_R_thresh_trackbar(int, void*);
void high_B_thresh_trackbar(int, void*);
void high_G_thresh_trackbar(int, void*);
void high_R_thresh_trackbar(int, void*);
void rectangleArea_thresh_trackbar(int, void*);
void rectangle_Width_thresh_trackbar(int, void*);
#pragma endregion
#pragma region Global Variables
const cv::Scalar COLOR_1 = cv::Scalar(0.0, 0.0, 0.0);
const cv::Scalar COLOR_2 = cv::Scalar(255.0, 255.0, 255.0);
const cv::Scalar COLOR_3 = cv::Scalar(255.0, 0.0, 0.0);
const cv::Scalar COLOR_4 = cv::Scalar(0.0, 200.0, 0.0);
const cv::Scalar COLOR_5 = cv::Scalar(0.0, 0.0, 255.0);
const cv::Scalar COLOR_6 = cv::Scalar(255.0, 127.0, 150);

int rectArea = 0, maxArea = 2000;
int rectWidth = 1, maxWidth = 20;
int low_B = 0, low_G = 0, low_R = 160;
int high_B = 79, high_G = 65, high_R = 255;

cv::Mat imgFrame1;
cv::Mat RedBlob;

cv::Mat structuringElement3x3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
cv::Mat structuringElement5x5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
cv::Mat structuringElement7x7 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
cv::Mat structuringElement9x9 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9));
#pragma endregion

int main(void) 
{
	cv::namedWindow("trackbars", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("blob params", cv::WINDOW_AUTOSIZE);
	cv::VideoCapture webcam(0);
	webcam.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	webcam.set(CV_CAP_PROP_FRAME_HEIGHT, 560);

	cv::createTrackbar("Rect area", "blob params", &rectArea, maxArea, rectangleArea_thresh_trackbar);
	cv::createTrackbar("Rect width", "blob params", &rectWidth, maxWidth, rectangle_Width_thresh_trackbar);
	cv::createTrackbar("Low Blue", "trackbars", &low_B, 255, low_B_thresh_trackbar);
	cv::createTrackbar("High Blue", "trackbars", &high_B, 255, high_B_thresh_trackbar);
	cv::createTrackbar("Low Green", "trackbars", &low_G, 255, low_G_thresh_trackbar);
	cv::createTrackbar("High Green", "trackbars", &high_G, 255, high_G_thresh_trackbar);
	cv::createTrackbar("Low Red", "trackbars", &low_R, 255, low_R_thresh_trackbar);
	cv::createTrackbar("High Red", "trackbars", &high_R, 255, high_R_thresh_trackbar);

	while ((char)cv::waitKey(36) != 'q') {
		webcam >> imgFrame1;
		if (imgFrame1.empty())
		{
			break;
		}

		std::vector<Blob> blobs;
		cv::inRange(imgFrame1, cv::Scalar(low_B, low_G, low_R), cv::Scalar(high_B, high_G, high_R), RedBlob);
		cv::imshow("red", RedBlob);

		cv::dilate(RedBlob, RedBlob, structuringElement5x5);
		cv::dilate(RedBlob, RedBlob, structuringElement3x3);

		std::vector<std::vector<cv::Point> > contours;
		cv::findContours(RedBlob, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		cv::Mat imgContours(RedBlob.size(), CV_8UC3, COLOR_1);
		cv::drawContours(imgContours, contours, -1, COLOR_2, -1);
		cv::imshow("imgContours", imgContours);

		std::vector<std::vector<cv::Point> > convexHulls(contours.size());

		for (unsigned int i = 0; i < contours.size(); i++)
		{
			cv::convexHull(contours[i], convexHulls[i]);
		}

		for (auto &convexHull : convexHulls)
		{
			Blob possibleBlob(convexHull);

			if (possibleBlob.boundingRect.area() > rectArea &&
				possibleBlob.dblAspectRatio >= 0.2 &&
				possibleBlob.dblAspectRatio <= 1.2 &&
				possibleBlob.boundingRect.width > rectWidth &&
				possibleBlob.boundingRect.height > 20 &&
				possibleBlob.dblDiagonalSize > 30.0) {
				blobs.push_back(possibleBlob);
			}
		}

		cv::Mat imgConvexHulls(RedBlob.size(), CV_8UC3, COLOR_1);

		convexHulls.clear();

		for (auto &blob : blobs)
		{
			convexHulls.push_back(blob.contour);
			cv::drawContours(imgConvexHulls, convexHulls, -1, COLOR_2, -1);  // for each blob
			cv::rectangle(imgFrame1, blob.boundingRect, COLOR_6, 2);		 // draw a box around the blob
			cv::circle(imgFrame1, blob.centerPosition, 3, COLOR_4, -1);		 // draw a filled-in circle at the center
		}

		cv::imshow("imgConvexHulls", imgConvexHulls);
		cv::imshow("imgFrame1", imgFrame1);

	}


	return(0);

}
#pragma region Trackbar callbacks
void rectangleArea_thresh_trackbar(int, void*)
{

}
void rectangle_Width_thresh_trackbar(int, void*)
{

}
void low_B_thresh_trackbar(int, void *)
{

}

void low_G_thresh_trackbar(int, void *)
{
}

void low_R_thresh_trackbar(int, void *)
{
}

void high_B_thresh_trackbar(int, void *)
{
}

void high_G_thresh_trackbar(int, void *)
{
}

void high_R_thresh_trackbar(int, void *)
{
}
#pragma endregion