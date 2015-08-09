#include "QTWindow.h"
#include <iostream>

int main(int argc, char ** argv)
{
	QApplication app(argc, argv);

  	if (argc != 2)
  	{
  		std::cout << "usage: "<<  argv[0] << " <imagePath>" << std::endl;
  		return -1;
  	}

	QTWindow * window = new QTWindow();
 	window->setupWindow(window, argv[0], argv[1]);
	
	app.exec();
	return 0;
}