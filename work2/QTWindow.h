#ifndef __QTWINDOW_H__
#define __QTWINDOW_H__

#include <QtGui>
#include <QMouseEvent>
#include <IL/il.h>
#include <bitset>

#include "Points.h"

typedef unsigned char BYTE;

class QTWindow: public QLabel
{
private:   
  int imageInputWidth;
  int imageInputHeight;
  int method;
  BYTE * pixmapInput;

public:    
	QTWindow(){};
	~QTWindow(){};

	void mousePressEvent (QMouseEvent * event); 
	void setupWindow(QTWindow * window, const char * title, const char * picturePath);
	void saveImage(const char * outputFilename, BYTE * imgData, int width, int height);
	void setMethod(int m);

};

#endif