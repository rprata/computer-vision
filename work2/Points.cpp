#include "Points.h"

#include <iostream>
Line Points::getR1()
{
	return r1;
}

Line Points::getR2()
{
	return r2;
}

Line Points::getR3()
{
	return r3;
}

Line Points::getR4()
{
	return r4;
}

void Points::setR1(std::pair <double, double> p1, std::pair <double, double> p2)
{
	Vector3d l1, l2;
	l1 << p1.first, p1.second, 1;
	l2 << p2.first, p2.second, 1;
	Vector3d res = l1.cross(l2);
	r1.x = res(0);
	r1.y = res(1);
	r1.z = res(2);
}

void Points::setR2(std::pair <double, double> p1, std::pair <double, double> p2)
{
	Vector3d l1, l2;
	l1 << p1.first, p1.second, 1;
	l2 << p2.first, p2.second, 1;
	Vector3d res = l1.cross(l2);
	r2.x = res(0);
	r2.y = res(1);
	r2.z = res(2);
}

void Points::setR3(std::pair <double, double> p1, std::pair <double, double> p2)
{
	Vector3d l1, l2;
	l1 << p1.first, p1.second, 1;
	l2 << p2.first, p2.second, 1;
	Vector3d res = l1.cross(l2);
	r3.x = res(0);
	r3.y = res(1);
	r3.z = res(2);
}

void Points::setR4(std::pair <double, double> p1, std::pair <double, double> p2)
{
	Vector3d l1, l2;
	l1 << p1.first, p1.second, 1;
	l2 << p2.first, p2.second, 1;
	Vector3d res = l1.cross(l2);
	r4.x = res(0);
	r4.y = res(1);
	r4.z = res(2);
}