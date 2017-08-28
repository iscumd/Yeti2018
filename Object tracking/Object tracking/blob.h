#pragma once
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

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

