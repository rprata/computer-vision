#include "Points.h"

TPoints Points::getRealPoints()
{
	return realPoints;
}

TPoints Points::getImagePoints()
{
	return imagePoints;
}

void Points::setRealPoints(std::pair <float, float> p1, std::pair <float, float> p2, std::pair <float, float> p3, std::pair <float, float> p4)
{
	realPoints.p1 = p1;
	realPoints.p2 = p2;
	realPoints.p3 = p3;
	realPoints.p4 = p4;
}

void Points::setImagePoints(std::pair <float, float> p1, std::pair <float, float> p2, std::pair <float, float> p3, std::pair <float, float> p4)
{
	imagePoints.p1 = p1;
	imagePoints.p2 = p2;
	imagePoints.p3 = p3;
	imagePoints.p4 = p4;
}