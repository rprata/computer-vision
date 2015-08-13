#include "CVMath.h"
#include <iostream>
#include <cmath>

#define OUTPUT_HEIGHT 458

void CVMath::setupMatrixM1()
{
	Line r1 = Points::getInstance().getR1();
	Line r2 = Points::getInstance().getR2();
	Line r3 = Points::getInstance().getR3();
	Line r4 = Points::getInstance().getR4();
	
	l0 << r1.x, r1.y, r1.z;
	l1 << r2.x, r2.y, r2.z;
	l2 << r3.x, r3.y, r3.z;
	l3 << r4.x, r4.y, r4.z;

	x0 = l0.cross(l1);
	x0 << x0(0)/x0(2), x0(1)/x0(2), 1;
	x1 = l2.cross(l3);
	x1 << x1(0)/x1(2), x1(1)/x1(2), 1;

	l =  x0.cross(x1);

	Hp = MatrixXd(3, 3);
	Hp << 1, 0, 0,
	     0, 1, 0,
	     l(0), l(1), l(2);
	std::cout << Hp << std::endl;
	Vector3d l0;
	Vector3d l1;
	Hp_INV = MatrixXd(3, 3);
	Hp_INV = Hp.inverse().eval();
	std::cout << Hp_INV << std::endl;
}

void CVMath::setupMatrixM2()
{
	Line r1 = Points::getInstance().getR1();
	Line r2 = Points::getInstance().getR2();
	Line r3 = Points::getInstance().getR3();
	Line r4 = Points::getInstance().getR4();
	
	l0 << r1.x, r1.y, r1.z;
	m0 << r2.x, r2.y, r2.z;
	l1 << r3.x, r3.y, r3.z;
	m1 << r4.x, r4.y, r4.z;

	MatrixXd LT_M = MatrixXd(2, 3);
	LT_M << l0(0)*m0(0), l0(0)*m0(1) + l0(1)*m0(0), l0(1)*m0(1),
			l1(0)*m1(0), l1(0)*m1(1) + l1(1)*m1(0), l1(1)*m1(1);

	MatrixXd ZERO = MatrixXd(2, 1);
	ZERO << 0, 0;

	MatrixXd H_HT = LT_M.ldlt().solve(ZERO);
	std::cout << H_HT << std::endl;

	Hp = MatrixXd(3, 3);
	Hp << H_HT(0), H_HT(1), 0,
		H_HT(1), 1, 0,
		0, 0, 0;

	Hp_INV = MatrixXd(3, 3);
	Hp_INV = Hp.inverse().eval();

}

unsigned char *  CVMath::generateImageArrayParallelLines(unsigned char * pixmapInput, int * width, int * height, int originalWidth, int originalHeight)
{
	//primeiro, vamos avaliar o tamanho da imagem através dos 4 cantos

	VectorXd in(3);
	VectorXd out;
	double minX = 0.0, minY = 0.0, maxX = 0.0, maxY = 0.0;

	in << 0, 0, 1;
	out = Hp * in;
	out /= out(2);

	minX = maxX = out(0);
	minY = maxY = out(1);

	in << 0, originalHeight, 1;
	out = Hp * in;
	out /= out(2);
		
	if (out(0) < minX)
		minX = out(0);
	if (out(0) > maxX)
		maxX = out(0);

	if (out(1) < minY)
		minY = out(1);
	if (out(1) > maxY)
		maxY = out(1);

	in << originalWidth, originalHeight, 1;
	out = Hp * in;
	out /= out(2);
	
	if (out(0) < minX)
		minX = out(0);
	if (out(0) > maxX)
		maxX = out(0);

	if (out(1) < minY)
		minY = out(1);
	if (out(1) > maxY)
		maxY = out(1);

	in << originalWidth, 0, 1;
	out = Hp * in;
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
	*height = OUTPUT_HEIGHT;
	*width = (maxX - minX)*(*height)/(maxY - minY);

	unsigned char * imgArray = new unsigned char[3 * (*height) * (*width)];

	double ratio = (maxX - minX)/(*width);
	//gerando nova imagem;
	for (int i = 0; i < *height; i++)
	{
		for (int j = 0; j < *width; j++)
		{
			in << minX + j*ratio, minY + i*ratio, 1;
			out = Hp_INV * in;
			out /= out(2);
			
			int x = out(0);
			int y = out(1);

			if ((x >= 0) && (y >= 0) && (x < originalWidth) && (y < originalHeight))
			{	
				imgArray[3*((*width)*i + j)] = pixmapInput[3*(originalWidth*y + x)];
				imgArray[3*((*width)*i + j) + 1] = pixmapInput[3*(originalWidth*y + x) + 1];
				imgArray[3*((*width)*i + j) + 2] = pixmapInput[3*(originalWidth*y + x) + 2];
			}
		}
	}

	return imgArray;
}


unsigned char *  CVMath::generateImageArrayOrthoLines(unsigned char * pixmapInput, int * width, int * height, int originalWidth, int originalHeight)
{
	//primeiro, vamos avaliar o tamanho da imagem através dos 4 cantos

	VectorXd in(3);
	VectorXd out;
	double minX = 0.0, minY = 0.0, maxX = 0.0, maxY = 0.0;

	in << 0, 0, 1;
	out = Hp * in;
	out /= out(2);

	minX = maxX = out(0);
	minY = maxY = out(1);

	in << 0, originalHeight, 1;
	out = Hp * in;
	out /= out(2);
		
	if (out(0) < minX)
		minX = out(0);
	if (out(0) > maxX)
		maxX = out(0);

	if (out(1) < minY)
		minY = out(1);
	if (out(1) > maxY)
		maxY = out(1);

	in << originalWidth, originalHeight, 1;
	out = Hp * in;
	out /= out(2);
	
	if (out(0) < minX)
		minX = out(0);
	if (out(0) > maxX)
		maxX = out(0);

	if (out(1) < minY)
		minY = out(1);
	if (out(1) > maxY)
		maxY = out(1);

	in << originalWidth, 0, 1;
	out = Hp * in;
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
	*height = OUTPUT_HEIGHT;
	*width = (maxX - minX)*(*height)/(maxY - minY);

	unsigned char * imgArray = new unsigned char[3 * (*height) * (*width)];

	double ratio = (maxX - minX)/(*width);
	//gerando nova imagem;
	for (int i = 0; i < *height; i++)
	{
		for (int j = 0; j < *width; j++)
		{
			in << minX + j*ratio, minY + i*ratio, 1;
			out = Hp_INV * in;
			out /= out(2);
			
			int x = out(0);
			int y = out(1);

			if ((x >= 0) && (y >= 0) && (x < originalWidth) && (y < originalHeight))
			{	
				imgArray[3*((*width)*i + j)] = pixmapInput[3*(originalWidth*y + x)];
				imgArray[3*((*width)*i + j) + 1] = pixmapInput[3*(originalWidth*y + x) + 1];
				imgArray[3*((*width)*i + j) + 2] = pixmapInput[3*(originalWidth*y + x) + 2];
			}
		}
	}

	return imgArray;
}