#include "CVMath.h"
#include <iostream>

void CVMath::setupMatrix()
{
	TPoints imagePoints = Points::getInstance().getImagePoints();
	xi1 = imagePoints.p1.first;
	yi1 = imagePoints.p1.second;
	xi2 = imagePoints.p2.first;
	yi2 = imagePoints.p2.second;
	xi3 = imagePoints.p3.first;
	yi3 = imagePoints.p3.second;
	xi4 = imagePoints.p4.first;
	yi4 = imagePoints.p4.second;
	
	TPoints realPoints = Points::getInstance().getRealPoints();
	xf1 = realPoints.p1.first;
	yf1 = realPoints.p1.second;
	xf2 = realPoints.p2.first;
	yf2 = realPoints.p2.second;
	xf3 = realPoints.p3.first;
	yf3 = realPoints.p3.second;
	xf4 = realPoints.p4.first;
	yf4 = realPoints.p4.second;

	A = MatrixXf (8, 8);
	A << xi1, yi1, 1, 0, 0, 0, -xi1*xf1, -yi1*xf1,
		 0, 0, 0, xi1, yi1, 1, -xi1*yf1, -yi1*yf1,
		 xi2, yi2, 1, 0, 0, 0, -xi2*xf2, -yi2*xf2,
		 0, 0, 0, xi2, yi2, 1, -xi2*yf2, -yi2*yf2,
		 xi3, yi3, 1, 0, 0, 0, -xi3*xf3, -yi3*xf3,
		 0, 0, 0, xi3, yi3, 1, -xi3*yf3, -yi3*yf3,
		 xi4, yi4, 1, 0, 0, 0, -xi4*xf4, -yi4*xf4,
		 0, 0, 0, xi4, yi4, 1, -xi4*yf4, -yi4*yf4;

	B = VectorXf(8);
	B << xf1, yf1, xf2, yf2, xf3, yf3, xf4, yf4;
}

void CVMath::solveEquation()
{
	X = A.fullPivLu().solve(B);
	std::cout << "The solution is:\n" << X << std::endl;
	h11 = X(0);
	h12 = X(1);
	h13 = X(2);
	h21 = X(3);
	h22 = X(4);
	h23 = X(5);
	h31 = X(6);
	h32 = X(7);

	H = MatrixXf(3, 3);
	H << h11, h12, h13,
	     h21, h22, h23,
	     h31, h32, 1;

}

void CVMath::InvertMatrixH()
{
	std::cout << "Inverse Matrix H is:\n" << H.inverse() << std::endl;
	VectorXf in(3);
	in << 337, 562, 1;
	VectorXf out = H*in;
	std::cout << "Inverse Matrix out is:\n" << out/out(2) << std::endl;
}