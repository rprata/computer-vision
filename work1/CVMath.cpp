#include "CVMath.h"
#include <iostream>
#include <cmath>

#define OUTPUT_HEIGHT 950

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

	A = MatrixXd (8, 8);
	A << xi1, yi1, 1, 0, 0, 0, -xi1*xf1, -yi1*xf1,
		 0, 0, 0, xi1, yi1, 1, -xi1*yf1, -yi1*yf1,
		 xi2, yi2, 1, 0, 0, 0, -xi2*xf2, -yi2*xf2,
		 0, 0, 0, xi2, yi2, 1, -xi2*yf2, -yi2*yf2,
		 xi3, yi3, 1, 0, 0, 0, -xi3*xf3, -yi3*xf3,
		 0, 0, 0, xi3, yi3, 1, -xi3*yf3, -yi3*yf3,
		 xi4, yi4, 1, 0, 0, 0, -xi4*xf4, -yi4*xf4,
		 0, 0, 0, xi4, yi4, 1, -xi4*yf4, -yi4*yf4;

	B = VectorXd(8);
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

	H = MatrixXd(3, 3);
	H << h11, h12, h13,
	     h21, h22, h23,
	     h31, h32, 1;

}

void CVMath::invertMatrixH()
{
	H_INV = MatrixXd(3, 3);
	H_INV = H.inverse();
}

unsigned char *  CVMath::generateImageArray(unsigned char * pixmapInput, int * width, int * height, int originalWidth, int originalHeight)
{
	//primeiro, vamos avaliar o tamanho da imagem através dos 4 cantos
	VectorXd in(3);
	VectorXd out;
	
	double minX = 0.0, minY = 0.0, maxX = 0.0, maxY = 0.0;

	in << 0, 0, 1;
	out = H_INV * in;
	out /= out(2);

	minX = maxX = out(0);
	minY = maxY = out(1);

	in << 0, originalHeight, 1;
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

	in << originalWidth, originalHeight, 1;
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

	in << originalWidth, 0, 1;
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
			out = H * in;
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

unsigned char * CVMath::generateImageArrayBilinearInterpolation(unsigned char * pixmapInput, int * width, int * height, int originalWidth, int originalHeight)
{
	//primeiro, vamos avaliar o tamanho da imagem através dos 4 cantos
	VectorXd in(3);
	VectorXd out;
	
	double minX = 0.0, minY = 0.0, maxX = 0.0, maxY = 0.0;

	in << 0, 0, 1;
	out = H_INV * in;
	out /= out(2);

	minX = maxX = out(0);
	minY = maxY = out(1);

	in << 0, originalHeight, 1;
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

	in << originalWidth, originalHeight, 1;
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

	in << originalWidth, 0, 1;
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
	*height = OUTPUT_HEIGHT;
	*width = (maxX - minX)*((double)*height)/(maxY - minY);

	unsigned char * imgArray = new unsigned char[3 * (*height) * (*width)];

	double ratio = (maxX - minX)/(double)(*width);
	//gerando nova imagem;
	for (int i = 1; i < *height - 1; i++)
	{
		for (int j = 1; j < *width -1; j++)
		{
			in << minX + j*ratio, minY + i*ratio, 1;
			out = H * in;
			out /= out(2);

			double x = out(0);
			double y = out(1);
			
			double iInf;
			double iSup;
			if ((x >= 0) && (y >= 0) && (x < originalWidth) && (y < originalHeight))
			{	
				//bilinear interpolation
				unsigned char rInf = pixmapInput[3*((int) (y - 1)*originalWidth + (int) x)];
				unsigned char rSup = pixmapInput[3*((int) (y - 1)*originalWidth + (int) x)];

				iInf = ((double) rInf*(ceil(x) - x))/(ceil(x) - floor(x)) + ((double) rInf*(x - floor(x))/(ceil(x) - floor(x)));
				iSup = ((double) rSup*(ceil(x) - x))/(ceil(x) - floor(x)) + ((double) rSup*(x - floor(x))/(ceil(x) - floor(x)));

				unsigned char r = (((double) iInf*(ceil(y) - y))/(ceil(y) - floor(y))) + ((double) iSup*(y - floor(y))/(ceil(y) - floor(y)));

				unsigned char gInf = pixmapInput[3*((int) (y - 1)*originalWidth + (int) x) + 1];
				unsigned char gSup = pixmapInput[3*((int) (y - 1)*originalWidth + (int) x) + 1];

				iInf = ((double) gInf*(ceil(x) - x))/(ceil(x) - floor(x)) + ((double) gInf*(x - floor(x))/(ceil(x) - floor(x)));
				iSup = ((double) gSup*(ceil(x) - x))/(ceil(x) - floor(x)) + ((double) gSup*(x - floor(x))/(ceil(x) - floor(x)));

				unsigned char g = (((double) iInf*(ceil(y) - y))/(ceil(y) - floor(y))) + ((double) iSup*(y - floor(y))/(ceil(y) - floor(y)));

				unsigned char bInf = pixmapInput[3*((int) (y - 1)*originalWidth + (int) x) + 2];
				unsigned char bSup = pixmapInput[3*((int) (y + 1)*originalWidth + (int) x) + 2];

				iInf = ((double) bInf*(ceil(x) - x))/(ceil(x) - floor(x)) + ((double) bInf*(x - floor(x))/(ceil(x) - floor(x)));
				iSup = ((double) bSup*(ceil(x) - x))/(ceil(x) - floor(x)) + ((double) bSup*(x - floor(x))/(ceil(x) - floor(x)));

				unsigned char b = (((double) iInf*(ceil(y) - y))/(ceil(y) - floor(y))) + ((double) iSup*(y - floor(y))/(ceil(y) - floor(y)));


				imgArray[3*((*width)*i + j)] = r;
				imgArray[3*((*width)*i + j) + 1] = g;
				imgArray[3*((*width)*i + j) + 2] = b;
			}
		}
	}

	return imgArray;
}


unsigned char * CVMath::generateCropImageArrayBilinearInterpolation(unsigned char * pixmapInput, int * width, int * height, int originalWidth, int originalHeight)
{
	//primeiro, vamos avaliar o tamanho da imagem através dos 4 cantos
	VectorXd in(3);
	VectorXd out;
	
	double minX = 0.0, minY = 0.0, maxX = 0.0, maxY = 0.0;

	in << 0, 0, 1;
	out = H_INV * in;
	out /= out(2);

	minX = maxX = out(0);
	minY = maxY = out(1);

	in << 0, originalHeight, 1;
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

	in << originalWidth, originalHeight, 1;
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

	in << originalWidth, 0, 1;
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
	*height = OUTPUT_HEIGHT;
	*width = (maxX - minX)*((double)*height)/(maxY - minY);

	unsigned char * imgArray = new unsigned char[3 * (*height) * (*width)];

	double ratio = (maxX - minX)/(double)(*width);
	//gerando nova imagem;
	for (int i = 1; i < *height - 1; i++)
	{
		for (int j = 1; j < *width -1; j++)
		{
			in << minX + j*ratio, minY + i*ratio, 1;
			out = H * in;
			out /= out(2);

			double x = out(0);
			double y = out(1);
			
			double iInf;
			double iSup;
			if ((x >= 0) && (y >= 0) && (x < originalWidth) && (y < originalHeight))
			{	

				if (j >= minX && i >= minY && j <= maxX && i <= maxY)
				{
					//bilinear interpolation
					unsigned char rInf = pixmapInput[3*((int) (y - 1)*originalWidth + (int) x)];
					unsigned char rSup = pixmapInput[3*((int) (y - 1)*originalWidth + (int) x)];

					iInf = ((double) rInf*(ceil(x) - x))/(ceil(x) - floor(x)) + ((double) rInf*(x - floor(x))/(ceil(x) - floor(x)));
					iSup = ((double) rSup*(ceil(x) - x))/(ceil(x) - floor(x)) + ((double) rSup*(x - floor(x))/(ceil(x) - floor(x)));

					unsigned char r = (((double) iInf*(ceil(y) - y))/(ceil(y) - floor(y))) + ((double) iSup*(y - floor(y))/(ceil(y) - floor(y)));

					unsigned char gInf = pixmapInput[3*((int) (y - 1)*originalWidth + (int) x) + 1];
					unsigned char gSup = pixmapInput[3*((int) (y - 1)*originalWidth + (int) x) + 1];

					iInf = ((double) gInf*(ceil(x) - x))/(ceil(x) - floor(x)) + ((double) gInf*(x - floor(x))/(ceil(x) - floor(x)));
					iSup = ((double) gSup*(ceil(x) - x))/(ceil(x) - floor(x)) + ((double) gSup*(x - floor(x))/(ceil(x) - floor(x)));

					unsigned char g = (((double) iInf*(ceil(y) - y))/(ceil(y) - floor(y))) + ((double) iSup*(y - floor(y))/(ceil(y) - floor(y)));

					unsigned char bInf = pixmapInput[3*((int) (y - 1)*originalWidth + (int) x) + 2];
					unsigned char bSup = pixmapInput[3*((int) (y + 1)*originalWidth + (int) x) + 2];

					iInf = ((double) bInf*(ceil(x) - x))/(ceil(x) - floor(x)) + ((double) bInf*(x - floor(x))/(ceil(x) - floor(x)));
					iSup = ((double) bSup*(ceil(x) - x))/(ceil(x) - floor(x)) + ((double) bSup*(x - floor(x))/(ceil(x) - floor(x)));
					
					unsigned char b = (((double) iInf*(ceil(y) - y))/(ceil(y) - floor(y))) + ((double) iSup*(y - floor(y))/(ceil(y) - floor(y)));


					imgArray[3*((*width)*i + j)] = r;
					imgArray[3*((*width)*i + j) + 1] = g;
					imgArray[3*((*width)*i + j) + 2] = b;
				}
			}
		}
	}

	return imgArray;
}