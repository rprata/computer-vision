#include "CVMath.h"
#include <iostream>

void CVMath::setupMatrix()
{
	
	TPoints realPoints = Points::getInstance().getRealPoints();
	xi1 = realPoints.p1.first;
	yi1 = realPoints.p1.second;
	xi2 = realPoints.p2.first;
	yi2 = realPoints.p2.second;
	xi3 = realPoints.p3.first;
	yi3 = realPoints.p3.second;
	xi4 = realPoints.p4.first;
	yi4 = realPoints.p4.second;

	TPoints imagePoints = Points::getInstance().getImagePoints();
	xf1 = imagePoints.p1.first;
	yf1 = imagePoints.p1.second;
	xf2 = imagePoints.p2.first;
	yf2 = imagePoints.p2.second;
	xf3 = imagePoints.p3.first;
	yf3 = imagePoints.p3.second;
	xf4 = imagePoints.p4.first;
	yf4 = imagePoints.p4.second;

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

void CVMath::invertMatrixH()
{
	H_INV = MatrixXf(3, 3);
	H_INV = H.inverse();
}

void CVMath::generateImageArray(unsigned char * imgArray, unsigned char * pixmapInput, int * width, int * height, int originalWidth, int originalHeight)
{
	//primeiro, vamos avaliar o tamanho da imagem através dos 4 cantos
	VectorXf in(3);
	VectorXf out;
	
	int minX = 0, minY = 0, maxX = 0, maxY = 0;

	in << 0, 0, 1;
	out = H_INV * in;
	out /= out(2);

	minX = maxX = out(0);
	minY = maxY = out(1);

	in << 0, 600, 1;
	out = H_INV * in;
	out /= out(2);
		
	if (out(0) < minX)
		minX = out(0);
	if (out(0) > maxX)
		maxX = out(0);

	if (out(1) < minY)
		minY = out(1);
	if (out(1) > maxY)
		maxY = out(1);

	in << 800, 600, 1;
	out = H_INV * in;
	out /= out(2);
	
	if (out(0) < minX)
		minX = out(0);
	if (out(0) > maxX)
		maxX = out(0);

	if (out(1) < minY)
		minY = out(1);
	if (out(1) > maxY)
		maxY = out(1);

	in << 800, 0, 1;
	out = H_INV * in;
	out /= out(2);
	
	if (out(0) < minX)
		minX = out(0);
	if (out(0) > maxX)
		maxX = out(0);

	if (out(1) < minY)
		minY = out(1);
	if (out(1) > maxY)
		maxY = out(1);

	// calculando os valores de largura e altura da nova imagem
	*width = maxX - minX;
	*height = maxY - minY;

	//gerando nova imagem;
	for (int i = 0; i < originalHeight; i++)
	{
		for (int j = 0; j < originalWidth; j++)
		{
			in << j, i, 1;
			out = H_INV * in;
			out /= out(2);

			int x = (maxX - out(0))*originalWidth/(*width);
			int y = (maxY - out(1))*originalHeight/(*height);

			imgArray[3*(originalWidth*y + x)]  = pixmapInput[3*(originalWidth*i + j)];
			imgArray[3*(originalWidth*y + x) + 1]  = pixmapInput[3*(originalWidth*i + j) + 1];
			imgArray[3*(originalWidth*y + x) + 2]  = pixmapInput[3*(originalWidth*i + j) + 2];
		}
	}
}