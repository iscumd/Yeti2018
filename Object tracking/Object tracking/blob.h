#pragma once
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

class Blob
{
public:
	std::vector<cv::Point> contour;
	cv::Rect bounding_Rect;
	cv::Point center_Position;
	double dbl_Diagonal_Size;
	double dbl_Aspect_Ratio;
	Blob(std::vector<cv::Point> _contour);
};

