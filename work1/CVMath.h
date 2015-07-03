#ifndef __CVMATH_H__
#define __CVMATH_H__

#include <eigen3/Eigen/Dense>
#include "Points.h"

using Eigen::MatrixXf;
using Eigen::VectorXf;

class CVMath
{

private:
	float h11, h12, h13,
	 	  h21, h22, h23,
	  	  h31, h32;

	float xi1, yi1;
	float xi2, yi2;
	float xi3, yi3;
	float xi4, yi4;
	
	float xf1, yf1;
	float xf2, yf2;
	float xf3, yf3;
	float xf4, yf4;

	MatrixXf A;
	VectorXf X;
	VectorXf B;

	MatrixXf H;
	MatrixXf H_INV;

public:
	void setupMatrix();
	void solveEquation();
	void invertMatrixH();
	void generateImageArray(unsigned char * imgArray, unsigned char * pixmapInput, int * width, int * height, int originalWidth, int originalHeight);

};

#endif	