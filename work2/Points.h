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



typedef struct _Point
{
    double x;
    double y;
    double w;

} Point;

class Points
{

private:
	Points() {};
	Line  r1, r2, r3, r4;
    Point p1, p2, p3, p4;

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

    Point getP1();
    Point getP2();
    Point getP3();
    Point getP4();

	void setR1(std::pair <double, double> p1, std::pair <double, double> p2);
	void setR2(std::pair <double, double> p1, std::pair <double, double> p2);
	void setR3(std::pair <double, double> p1, std::pair <double, double> p2);
	void setR4(std::pair <double, double> p1, std::pair <double, double> p2);

    void setP1(std::pair <double, double> p);
    void setP2(std::pair <double, double> p);
    void setP3(std::pair <double, double> p);
    void setP4(std::pair <double, double> p);
};

#endif