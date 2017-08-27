#include "Blob.h"

Blob::Blob(std::vector<cv::Point> _contour) 
{
	contour = _contour;
	bounding_Rect = cv::boundingRect(contour);
	center_Position.x = (2 * bounding_Rect.x + bounding_Rect.width) / 2;
	center_Position.y = (2 * bounding_Rect.y + bounding_Rect.height) / 2;
	dbl_Diagonal_Size = sqrt(pow(bounding_Rect.width, 2) + pow(bounding_Rect.height, 2));
	dbl_Aspect_Ratio = (float)bounding_Rect.width / (float)bounding_Rect.height;

}