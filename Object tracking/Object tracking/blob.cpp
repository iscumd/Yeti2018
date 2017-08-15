#include "Blob.h"

Blob::Blob(std::vector<cv::Point> _contour) 
{
	contour = _contour;
	boundingRect = cv::boundingRect(contour);
	centerPosition.x = (2 * boundingRect.x + boundingRect.width) / 2;
	centerPosition.y = (2 * boundingRect.y + boundingRect.height) / 2;
	dblDiagonalSize = sqrt(pow(boundingRect.width, 2) + pow(boundingRect.height, 2));
	dblAspectRatio = (float)boundingRect.width / (float)boundingRect.height;
}