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
  	// Utils::getInstance()->calculateDLT();
  	// Utils::getInstance()->printMatrixH();

    Matrix3d Hf = Utils::getInstance()->ransac(200, 0.05, 4);
    Matrix3d H;

    H << (double)Hf(0,0), (double)Hf(0,1), (double)Hf(0,2),
       (double)Hf(1,0), (double)Hf(1,1), (double)Hf(1,2),
       (double)Hf(2,0), (double)Hf(2,1), (double)Hf(2,2);
    
    cout << H << endl;
    
    H = H.inverse().eval();

  	Image img(make_pair(argv[1], Matrix<double, 3, 3>::Identity()), make_pair(argv[2], H));

#ifdef USING_QT
  	img.DrawPanoramicPictureUsingQT("output.jpg");
#else
    img.DrawPanoramicPicture("output.jpg");
#endif
	return 0;
}