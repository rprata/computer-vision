#include "QTWindow.h"
#include <iostream>

void QTWindow::mousePressEvent (QMouseEvent * event)
{
	if(event->button() == Qt::LeftButton)
	{
		std::cout << event->x() << " " << event->y() << std::endl;
	} 
} 

void QTWindow::setupWindow(QTWindow * window, const char * title, const char * imagePath)
{
	ilInit();
	ilLoadImage((const ILstring)imagePath); 

	window->setWindowTitle(QString::fromUtf8(title));
    window->resize((int) ilGetInteger(IL_IMAGE_WIDTH), (int) ilGetInteger(IL_IMAGE_HEIGHT));
	QPixmap pm(imagePath);
	window->setPixmap(pm);
	window->show();
}