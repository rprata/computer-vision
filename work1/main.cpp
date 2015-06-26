#include <QTWindow.h>

int main(int argc, char ** argv)
{
  	QApplication app(argc, argv);

  	QTWindow * window = new QTWindow();
  	window->setupWindow(window, argv[0], argv[1]);
  	app.exec();
  	return 0;
}