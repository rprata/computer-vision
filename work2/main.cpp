#include "QTWindow.h"
#include <iostream>

int main(int argc, char ** argv)
{
	QApplication app(argc, argv);

  	if (argc != 3)
  	{
  		std::cout << "usage: "<<  argv[0] << " <imagePath> <metodo retas paralelas (0) | orthogonais (1)>" << std::endl;
  		return -1;
  	}

	QTWindow * window = new QTWindow();
	window->setMethod(atoi(argv[2]));
 	window->setupWindow(window, argv[0], argv[1]);
	
	app.exec();
	return 0;
}