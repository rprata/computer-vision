#ifndef __QTWINDOW_H__
#define __QTWINDOW_H__

#include <QtGui>
#include <QMouseEvent>
#include <IL/il.h>

class QTWindow: public QLabel
{
private:   
  QMessageBox* msgBox;

public:    
	QTWindow(){};
	~QTWindow(){};

	void mousePressEvent (QMouseEvent * event); 
	void setupWindow(QTWindow * window, const char * title, const char * picturePath);

};

#endif