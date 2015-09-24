#include <iostream>
#include "Utils.h"
#include "Image.h"

int main(int argc, char ** argv)
{
  	if (argc != 3)
  	{
  		std::cout << "usage: "<<  argv[0] << " <img1> <img2>" << std::endl;
  		return -1;
  	}

  	Utils::getInstance()->generateMatchingPoints(argv[1], argv[2]);
  	// Utils::getInstance()->printVectorPairPoints();
  	Utils::getInstance()->calculateDLT();
  	// Utils::getInstance()->printMatrixH();
  	Image img(make_pair(argv[1], Matrix<double, 3, 3>::Identity()), make_pair(argv[2], Utils::getInstance()->getMatrixH()));
  	img.DrawPanoramicPicture("output.jpg");
	return 0;
}