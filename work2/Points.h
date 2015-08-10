#ifndef __POINTS_H__
#define __POINTS_H__

#include <utility>
#include <eigen3/Eigen/Dense>

using Eigen::Vector3d;

typedef struct _Line
{
    double x;
    double y;
    double z;

} Line;

class Points
{

private:
	Points() {};
	Line  r1, r2, r3, r4;

public:
	static Points& getInstance()
    {
    	static Points instance;
        return instance;
    }

    Points(Points const&);
    void operator=(Points const&);

    Line getR1();
    Line getR2();
    Line getR3();
    Line getR4();

	void setR1(std::pair <double, double> p1, std::pair <double, double> p2);
	void setR2(std::pair <double, double> p1, std::pair <double, double> p2);
	void setR3(std::pair <double, double> p1, std::pair <double, double> p2);
	void setR4(std::pair <double, double> p1, std::pair <double, double> p2);

};

#endif