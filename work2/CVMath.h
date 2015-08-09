#ifndef __CVMATH_H__
#define __CVMATH_H__

#include <eigen3/Eigen/Dense>
#include "Points.h"
#include "QTWindow.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

class CVMath
{

private:
	double h11, h12, h13,
	 	  h21, h22, h23,
	  	  h31, h32;

	Vector3d l0;
	Vector3d l1;
	Vector3d l2;
	Vector3d l3;

	Vector3d x0;
	Vector3d x1;

	Vector3d l;

	MatrixXd Hp;
	MatrixXd Hp_INV;

public:
	void setupMatrix();
	void generateHp();
	void invertMatrixHp();
	unsigned char * generateImageArrayParallelLines(unsigned char * pixmapInput, int * width, int * height, int originalWidth, int originalHeight);
	
};

#endif	