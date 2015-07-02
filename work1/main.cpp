#include "QTWindow.h"
#include <iostream>

//81.9x61.3

int main(int argc, char ** argv)
{
  	QApplication app(argc, argv);

  	if (argc != 4)
  	{
  		std::cout << "usage: "<<  argv[0] << " <imagePath> <realWidth> <realHeight>" << std::endl;
  		return -1;
  	}

 	float width = atof(argv[2]);
 	float height = atof(argv[3]);

 	Points::getInstance().setRealPoints(std::make_pair(0.0, 0.0), std::make_pair(0.0, height), std::make_pair(width, height), std::make_pair(width, 0.0));

  	QTWindow * window = new QTWindow();
  	window->setupWindow(window, argv[0], argv[1]);

  	app.exec();
  	return 0;
}