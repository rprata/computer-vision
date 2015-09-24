#ifndef __IMAGE__
#define __IMAGE__

#include <iostream>
#include <string>
#include <utility> 

#include <eigen3/Eigen/Dense>

#include <IL/il.h>

using namespace std;
using namespace Eigen;

typedef unsigned char BYTE;

typedef struct _Bound
{
	double minX;
	double maxX;
	double minY;
	double maxY;

} Bound;

class Image
{

private:
	int image1InputHeight, image1InputWidth;
	int image2InputHeight, image2InputWidth;
	BYTE * pixmapInputImage1;
	BYTE * pixmapInputImage2;
	MatrixXd H1;
	MatrixXd H2;
	Bound bound;

	int imageOutputHeight, imageOutputWidth;

public:
	Image(pair< char *, MatrixXd > arg1, pair< char *, MatrixXd > arg2);
	void getBounds(void);
	void DrawPanoramicPicture(const string & outputPanoramic);
};

#endif
