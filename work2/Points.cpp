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

double * Points::getR1eq() 
{
	double vec[3];
	vec[0] = m_p1.second - m_p2.second;
	vec[1] = m_p2.first - m_p1.first;
	vec[2] = m_p1.first*m_p2.second - m_p2.first*m_p1.second;
	std::cout << vec[0] << std::endl;
	std::cout << vec[1] << std::endl;
	std::cout << vec[2] << std::endl;
	return vec;
}

double * Points::getR2eq() 
{
	double vec[3];
	vec[0] = m_p3.second - m_p4.second;
	vec[1] = m_p4.first - m_p3.first;
	vec[2] = m_p3.first*m_p4.second - m_p4.first*m_p3.second;
	std::cout << vec[0] << std::endl;
	std::cout << vec[1] << std::endl;
	std::cout << vec[2] << std::endl;
	return vec;
}

double * Points::getR3eq() 
{
	double vec[3];
	vec[0] = m_p5.second - m_p6.second;
	vec[1] = m_p6.first - m_p5.first;
	vec[2] = m_p5.first*m_p6.second - m_p6.first*m_p5.second;
	std::cout << vec[0] << std::endl;
	std::cout << vec[1] << std::endl;
	std::cout << vec[2] << std::endl;
	std::cout << "0000000" << std::endl;
	return vec;
}

double * Points::getR4eq()
{
	double vec[3];
	vec[0] = m_p7.second - m_p8.second;
	vec[1] = m_p8.first - m_p7.first;
	vec[2] = m_p7.first*m_p8.second - m_p8.first*m_p7.second;
	std::cout << vec[0] << std::endl;
	std::cout << vec[1] << std::endl;
	std::cout << vec[2] << std::endl;
	std::cout << "0000000" << std::endl;
	return vec;
}

void Points::setR1(std::pair <double, double> p1, std::pair <double, double> p2)
{
	m_p1 = p1;
	m_p2 = p2;
	r1.first = p1.first - p2.first;
	r1.second = p1.second - p2.second;
}

void Points::setR2(std::pair <double, double> p1, std::pair <double, double> p2)
{
	m_p3 = p1;
	m_p4 = p2;
	r2.first = p1.first - p2.first;
	r2.second = p1.second - p2.second;
}

void Points::setR3(std::pair <double, double> p1, std::pair <double, double> p2)
{
	m_p5 = p1;
	m_p6 = p2;
	r3.first = p1.first - p2.first;
	r3.second = p1.second - p2.second;
}

void Points::setR4(std::pair <double, double> p1, std::pair <double, double> p2)
{
	m_p7 = p1;
	m_p8 = p2;
	r4.first = p1.first - p2.first;
	r4.second = p1.second - p2.second;
}