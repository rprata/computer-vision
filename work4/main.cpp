#include <iostream>
#include "Utils.h"

int main(int argc, char ** argv)
{
  	if (argc != 1)
  	{
  		std::cout << "usage: "<<  argv[0] << std::endl;
  		return -1;
  	}

  	Utils::getInstance()->generateMatchingPoints("../imgs/work4/thai-lion/DSC_0176.JPG", "../imgs/work4/thai-lion/DSC_0179.JPG");
    Utils::getInstance()->ransac(1000, 0.05, 8);

    Utils::getInstance()->generateE();
    Utils::getInstance()->generateP();
  	return 0;
}