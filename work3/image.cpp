#include "Image.h"

#define OUTPUT_HEIGHT 480

Image::Image(pair<char * , MatrixXd > arg1, pair< char * , MatrixXd > arg2)
{
	ilInit();
	if (!ilLoadImage((const ILstring)arg1.first)) 
	{
		cout << "error load image 1" << endl;
	} 

	image1InputWidth  = (int) ilGetInteger(IL_IMAGE_WIDTH);
	image1InputHeight = (int) ilGetInteger(IL_IMAGE_HEIGHT);

	pixmapInputImage1 = new BYTE[3 * image1InputWidth * image1InputHeight];

	ilCopyPixels(0, 0, 0, image1InputWidth, image1InputHeight, 1, IL_RGB, IL_UNSIGNED_BYTE, pixmapInputImage1);

	H1 = arg1.second;

	if(!ilLoadImage((const ILstring)arg2.first))
	{
		cout << "error load image 2 " << endl;
	} 

	image2InputWidth  = (int) ilGetInteger(IL_IMAGE_WIDTH);
	image2InputHeight = (int) ilGetInteger(IL_IMAGE_HEIGHT);

	pixmapInputImage2 = new BYTE[3 * image2InputWidth * image2InputHeight];

	ilCopyPixels(0, 0, 0, image2InputWidth, image2InputHeight, 1, IL_RGB, IL_UNSIGNED_BYTE, pixmapInputImage2);
	H2 = arg2.second;
	
	getBounds();

}

void Image::getBounds(void)
{	
	double minX = 0.0, minY = 0.0, maxX = 0.0, maxY = 0.0;
	VectorXd in(3);
	VectorXd out;

	// first image
	in << 0, 0, 1;
	out = H1 * in;

	minX = maxX = out(0);
	minY = maxY = out(1);

	in << 0, image1InputHeight, 1;
	out = H1 * in;
	out /= out(2);	
	
	if (out(0) < minX)
		minX = out(0);
	if (out(0) > maxX)
		maxX = out(0);

	if (out(1) < minY)
		minY = out(1);
	if (out(1) > maxY)
		maxY = out(1);

	in << image1InputWidth, image1InputHeight, 1;
	out = H1 * in;
	out /= out(2);

	if (out(0) < minX)
		minX = out(0);
	if (out(0) > maxX)
		maxX = out(0);

	if (out(1) < minY)
		minY = out(1);
	if (out(1) > maxY)
		maxY = out(1);

	in << image1InputWidth, 0, 1;
	out = H1 * in;
	out /= out(2);
	
	if (out(0) < minX)
		minX = out(0);
	if (out(0) > maxX)
		maxX = out(0);

	if (out(1) < minY)
		minY = out(1);
	if (out(1) > maxY)
		maxY = out(1);

	// second image
	in << 0, 0, 1;
	out = H2 * in;
	out /= out(2);
	cout << out << endl;

	if (out(0) < minX)
		minX = out(0);
	if (out(0) > maxX)
		maxX = out(0);

	if (out(1) < minY)
		minY = out(1);
	if (out(1) > maxY)
		maxY = out(1);

	in << 0, image2InputHeight, 1;
	out = H2 * in;
	out /= out(2);
	cout << out << endl;

	if (out(0) < minX)
		minX = out(0);
	if (out(0) > maxX)
		maxX = out(0);

	if (out(1) < minY)
		minY = out(1);
	if (out(1) > maxY)
		maxY = out(1);

	in << image2InputWidth, image2InputHeight, 1;
	out = H2 * in;
	out /= out(2);
	cout << out << endl;

	if (out(0) < minX)
		minX = out(0);
	if (out(0) > maxX)
		maxX = out(0);

	if (out(1) < minY)
		minY = out(1);
	if (out(1) > maxY)
		maxY = out(1);

	in << image2InputWidth, 0, 1;
	out = H2 * in;
	out /= out(2);
	cout << out << endl;

	if (out(0) < minX)
		minX = out(0);
	if (out(0) > maxX)
		maxX = out(0);

	if (out(1) < minY)
		minY = out(1);
	if (out(1) > maxY)
		maxY = out(1);

	bound.minX = minX;
	bound.minY = minY;
	bound.maxX = maxX;
	bound.maxY = maxY;

	cout << bound.minX << " " << bound.minY << " " << bound.maxX << " " << bound.maxY << endl;
}

void Image::DrawPanoramicPicture(const string & outputPanoramic)
{
	imageOutputHeight = OUTPUT_HEIGHT;
	imageOutputWidth = (bound.maxX - bound.minX)*((double)imageOutputHeight)/(bound.maxY - bound.minY);

	double ratio = (bound.maxX - bound.minX)/(double)(imageOutputWidth);

	unsigned char * imgArray = new unsigned char[3 * (imageOutputHeight) * (imageOutputWidth)];

	VectorXd in(3);
	VectorXd out;

	//first image
	MatrixXd H1_INV = H1.inverse().eval();
	cout << H1_INV << endl;
	for (int i = 0; i < imageOutputHeight; i++)
	{
		for (int j = 0; j < imageOutputWidth; j++)
		{
			in << bound.minX + j*ratio, bound.minY + i*ratio, 1;
			out = H1_INV * in;
			out /= out(2);
			
			int x = out(0);
			int y = out(1);

			if ((x >= 0) && (y >= 0) && (x < image1InputWidth) && (y < image1InputHeight))
			{	
				imgArray[3*((imageOutputWidth)*i + j)] = pixmapInputImage1[3*(image1InputWidth*y + x)];
				imgArray[3*((imageOutputWidth)*i + j) + 1] = pixmapInputImage1[3*(image1InputWidth*y + x) + 1];
				imgArray[3*((imageOutputWidth)*i + j) + 2] = pixmapInputImage1[3*(image1InputWidth*y + x) + 2];
			}
		}
	}

	//second image
	MatrixXd H2_INV = H2.inverse().eval();
	cout << H2_INV << endl;
	for (int i = 0; i < imageOutputHeight; i++)
	{
		for (int j = 0; j < imageOutputWidth; j++)
		{
			in << bound.minX + j*ratio, bound.minY + i*ratio, 1;
			out = H2_INV * in;
			out /= out(2);
			
			int x = out(0);
			int y = out(1);

			if ((x >= 0) && (y >= 0) && (x < image2InputWidth) && (y < image2InputHeight))
			{	
				imgArray[3*((imageOutputWidth)*i + j)] = pixmapInputImage2[3*(image2InputWidth*y + x)];
				imgArray[3*((imageOutputWidth)*i + j) + 1] = pixmapInputImage2[3*(image2InputWidth*y + x) + 1];
				imgArray[3*((imageOutputWidth)*i + j) + 2] = pixmapInputImage2[3*(image2InputWidth*y + x) + 2];
			}
		}
	}

	ILuint imageID = ilGenImage();
	ilBindImage(imageID);
	ilTexImage(imageOutputWidth, imageOutputHeight, 1, 3, IL_RGB, IL_UNSIGNED_BYTE, imgArray);
  	ilEnable(IL_FILE_OVERWRITE);
	ilSave(IL_JPG, outputPanoramic.c_str());

}
