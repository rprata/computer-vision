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
		CVMath cvm;
		switch (counter) 
		{
			case 0:
				p1 = std::make_pair((float) event->x(), (float)event->y());
				counter++;
				break;
			case 1:
				p2 = std::make_pair((float) event->x(), (float)event->y());
				counter++;
				break;
			case 2:
				p3 = std::make_pair((float) event->x(), (float)event->y());
				counter++;
				break;
			case 3:
				p4 = std::make_pair((float) event->x(), (float)event->y());
				Points::getInstance().setImagePoints(p1, p2, p3, p4);
				cvm.setupMatrix();
				cvm.solveEquation();
				cvm.invertMatrixH();
				
				int width, height;
				BYTE  * outputArrayImage;
				
				outputArrayImage = cvm.generateImageArray(pixmapInput, &width, &height, imageInputWidth, imageInputHeight);
				saveImage("../imgs/work1/retificada.jpg", outputArrayImage, width, height);
				
				std::cout << "Imagem retificada salva em: ../imgs/work1/retificada.jpg" << std::endl;

				BYTE  * outputArrayImageInterpolation;
				outputArrayImageInterpolation = cvm.generateImageArrayBilinearInterpolation(pixmapInput, &width, &height, imageInputWidth, imageInputHeight);
				saveImage("../imgs/work1/retificada-com-interpolacao-bilinear.jpg", outputArrayImageInterpolation, width, height);

				std::cout << "Imagem com interpolacao bilinear salva em: ../imgs/work1/retificada-com-interpolacao-bilinear.jpg" << std::endl;



				BYTE  * outputArrayImageCrop;
				outputArrayImageCrop = cvm.generateCropImageArrayBilinearInterpolation(pixmapInput, &width, &height, imageInputWidth, imageInputHeight);
				saveImage("../imgs/work1/crop-com-interpolacao-bilinear.jpg", outputArrayImageCrop, width, height);

				std::cout << "Regiao de interesse com interpolacao bilinear salva em: ../imgs/work1/crop-com-interpolacao-bilinear.jpg" << std::endl;

				counter = 0;

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

	pixmapInput = new BYTE[3 * imageInputWidth * imageInputWidth];

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
