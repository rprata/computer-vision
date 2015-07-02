#ifndef __POINTS_H__
#define __POINTS_H__

#include <utility>

typedef struct _TPoints
{
	std::pair <float , float> p1;
	std::pair <float , float> p2;
	std::pair <float , float> p3;
	std::pair <float , float> p4;
} TPoints;

class Points
{

private:
	Points() {};

	TPoints realPoints;
	TPoints imagePoints;

public:
	static Points& getInstance()
    {
    	static Points instance;
        return instance;
    }

    Points(Points const&);
    void operator=(Points const&);

	TPoints getRealPoints();
	TPoints getImagePoints();
	void setRealPoints(std::pair <float, float> p1, std::pair <float, float> p2, std::pair <float, float> p3, std::pair <float, float> p4);
	void setImagePoints(std::pair <float, float> p1, std::pair <float, float> p2, std::pair <float, float> p3, std::pair <float, float> p4);
};

#endif