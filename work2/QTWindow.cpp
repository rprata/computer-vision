#include "QTWindow.h"
#include <iostream>
#include <cstdio>
#include "CVMath.h"

static int counter = 0;

std::pair<double, double> p1, p2, p3, p4, p5, p6, p7, p8;

CVMath cvm;

void QTWindow::mousePressEvent (QMouseEvent * event)
{
	if(event->button() == Qt::LeftButton)
	{
		std::cout << event->x() << " "  << event->y() << std::endl;
		switch (counter) 
		{
			case 0:
				p1 = std::make_pair((double) event->x(), (double)event->y());
				counter++;
				break;
			case 1:
				p2 = std::make_pair((double) event->x(), (double)event->y());
				Points::getInstance().setR1(p1, p2);
				counter++;
				break;
			case 2:
				p3 = std::make_pair((double) event->x(), (double)event->y());
				counter++;
				break;
			case 3:
				p4 = std::make_pair((double) event->x(), (double)event->y());
				Points::getInstance().setR2(p3, p4);
				counter++;		
				break;
			case 4:
				p5 = std::make_pair((double) event->x(), (double)event->y());
				counter++;
				break;
			case 5:
				p6 = std::make_pair((double) event->x(), (double)event->y());
				Points::getInstance().setR3(p5, p6);
				counter++;			
				break;
			case 6:
				p7 = std::make_pair((double) event->x(), (double)event->y());
				counter++;			
				break;
			case 7:
				p8 = std::make_pair((double) event->x(), (double)event->y());
				Points::getInstance().setR4(p7, p8);
				counter++;				
				BYTE  * outputArrayImage;
				int width;
				int height;

				if (method == 0)
				{
					std::cout << "show time! retificacao afim" << std::endl;			
					cvm.setupMatrixM1();
					outputArrayImage = cvm.generateImageArrayParallelLines(pixmapInput, &width, &height, imageInputWidth, imageInputHeight);
					saveImage("../imgs/work2/retificada-retas-paralelas.jpg", outputArrayImage, width, height);
				}
				else 
				{
					std::cout << "show time! retificacao similaridade" << std::endl;			
					cvm.setupMatrixM2();
					outputArrayImage = cvm.generateImageArrayOrthoLines(pixmapInput, &width, &height, imageInputWidth, imageInputHeight);
					saveImage("../imgs/work2/retificada-ortho.jpg", outputArrayImage, width, height);
				}
				exit(0);
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

	pixmapInput = new BYTE[3 * imageInputWidth * imageInputHeight];

	ilCopyPixels(0, 0, 0, imageInputWidth, imageInputWidth, 1, IL_RGB, IL_UNSIGNED_BYTE, pixmapInput);

	window->setWindowTitle(QString::fromUtf8(title));
    window->resize(imageInputWidth, imageInputHeight);
	QPixmap pm(imagePath);
	window->setPixmap(pm);
	window->show();
}

void QTWindow::saveImage(const char * outputFilename, BYTE * imgData, int width, int height)
{
	ILuint imageID = ilGenImage();
	ilBindImage(imageID);
	ilTexImage(width, height, 1, 3, IL_RGB, IL_UNSIGNED_BYTE, imgData);
  	ilEnable(IL_FILE_OVERWRITE);
	ilSave(IL_JPG, outputFilename);
}

void QTWindow::setMethod(int m)
{
	method = m;
}