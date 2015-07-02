#include "QTWindow.h"
#include <iostream>
#include <cstdio>
#include "CVMath.h"

static int counter = 0;

std::pair<float, float> p1, p2, p3, p4;

void QTWindow::mousePressEvent (QMouseEvent * event)
{
	if(event->button() == Qt::LeftButton)
	{
		// std::cout << event->x() << " " << event->y() << std::endl;
		// int pixPos = event->y() * imageInputWidth + event->x();
		// printf("%d\n", pixPos);
		// printf("Color: %d %d %d\n", pixmapInput[3 * pixPos], pixmapInput[3 * pixPos + 1], pixmapInput[3 * pixPos + 2]);
		CVMath cvm;
		switch (counter) 
		{
			case 0:
				std::cout << "P1\n";
				p1 = std::make_pair((float) event->x(), (float)event->y());
				counter++;
				break;
			case 1:
				std::cout << "P2\n";
				p2 = std::make_pair((float) event->x(), (float)event->y());
				counter++;
				break;
			case 2:
				std::cout << "P3\n";
				p3 = std::make_pair((float) event->x(), (float)event->y());
				counter++;
				break;
			case 3:
				std::cout << "P4\n";
				p4 = std::make_pair((float) event->x(), (float)event->y());
				std::cout << "Computando Pontos: \n";
				Points::getInstance().setImagePoints(p1, p2, p3, p4);
				cvm.setupMatrix();
				cvm.solveEquation();
				cvm.InvertMatrixH();
				counter = 0;
				break;
			default:
				break;

		}
	} 
} 

void QTWindow::setupWindow(QTWindow * window, const char * title, const char * imagePath)
{
	ilInit();
	ilLoadImage((const ILstring)imagePath); 

	imageInputWidth  = (int) ilGetInteger(IL_IMAGE_WIDTH);
	imageInputHeight = (int) ilGetInteger(IL_IMAGE_HEIGHT);

	pixmapInput = new BYTE[3 * imageInputWidth * imageInputWidth];

	ilCopyPixels(0, 0, 0, imageInputWidth, imageInputWidth, 1, IL_RGB, IL_UNSIGNED_BYTE, pixmapInput);

	window->setWindowTitle(QString::fromUtf8(title));
    window->resize(imageInputWidth, imageInputHeight);
	QPixmap pm(imagePath);
	window->setPixmap(pm);
	window->show();
}