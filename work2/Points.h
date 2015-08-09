#ifndef __POINTS_H__
#define __POINTS_H__

#include <utility>

typedef std::pair <double , double> Line;
typedef std::pair <double , double> Point;

class Points
{

private:
	Points() {};
	Line  r1, r2, r3, r4;
    Point m_p1, m_p2, m_p3, m_p4, m_p5, m_p6, m_p7, m_p8;

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

    double * getR1eq();
    double * getR2eq();
    double * getR3eq();
    double * getR4eq();


	void setR1(std::pair <double, double> p1, std::pair <double, double> p2);
	void setR2(std::pair <double, double> p1, std::pair <double, double> p2);
	void setR3(std::pair <double, double> p1, std::pair <double, double> p2);
	void setR4(std::pair <double, double> p1, std::pair <double, double> p2);

};

#endif