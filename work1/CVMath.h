#ifndef __CVMATH_H__
#define __CVMATH_H__

#include <eigen3/Eigen/Dense>
#include "Points.h"
#include "QTWindow.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class CVMath
{

private:
	double h11, h12, h13,
	 	  h21, h22, h23,
	  	  h31, h32;

	double xi1, yi1;
	double xi2, yi2;
	double xi3, yi3;
	double xi4, yi4;
	
	double xf1, yf1;
	double xf2, yf2;
	double xf3, yf3;
	double xf4, yf4;

	MatrixXd A;
	VectorXd X;
	VectorXd B;

	MatrixXd H;
	MatrixXd H_INV;

public:
	void setupMatrix();
	void solveEquation();
	void invertMatrixH();
	unsigned char * generateImageArray(unsigned char * pixmapInput, int * width, int * height, int originalWidth, int originalHeight);
	unsigned char * generateImageArrayBilinearInterpolation(unsigned char * pixmapInput, int * width, int * height, int originalWidth, int originalHeight);
	unsigned char * generateCropImageArrayBilinearInterpolation(unsigned char * pixmapInput, int * width, int * height, int originalWidth, int originalHeight);

};

#endif	