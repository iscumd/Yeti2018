#pragma once
<<<<<<< HEAD
// Blob.h

#ifndef MY_BLOB
#define MY_BLOB

=======
>>>>>>> 435da048d865c0a2542740ce6cd1f731bed7b60f
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

<<<<<<< HEAD
///////////////////////////////////////////////////////////////////////////////////////////////////
class Blob {
public:
	// member variables ///////////////////////////////////////////////////////////////////////////
	std::vector<cv::Point> contour;

	cv::Rect boundingRect;

	cv::Point centerPosition;

	double dblDiagonalSize;

	double dblAspectRatio;
	

	// function prototypes ////////////////////////////////////////////////////////////////////////
	Blob(std::vector<cv::Point> _contour);

};

#endif    // MY_BLOB
=======
class Blob
{
public:
	std::vector<cv::Point> contour;
	cv::Rect boundingRect;
	cv::Point centerPosition;
	double dblDiagonalSize;
	double dblAspectRatio;
	Blob(std::vector<cv::Point> _contour);
};

>>>>>>> 435da048d865c0a2542740ce6cd1f731bed7b60f
