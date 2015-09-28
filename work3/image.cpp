#include "Image.h"
#include "Utils.h"

#define OUTPUT_HEIGHT 480

Image::Image(pair<char * , MatrixXd > arg1, pair< char * , MatrixXd > arg2)
{

#ifndef USING_QT
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
#else
    inputImage1 = QImage(arg1.first);
  	image1InputWidth  = inputImage1.width();
	image1InputHeight = inputImage1.height();
    H1 = arg1.second;

    inputImage2 = QImage(arg2.first);
    image2InputWidth  = inputImage2.width();
	image2InputHeight = inputImage2.height();
	H2 = arg2.second;
#endif

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
	// out /= out(2);	

	minX = maxX = out(0);
	minY = maxY = out(1);

	in << 0, image1InputHeight, 1;
	out = H1 * in;
	// out /= out(2);	
	
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
	// out /= out(2);

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
	// out /= out(2);
	
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
	// out /= out(2);

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
	// out /= out(2);

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
	// out /= out(2);

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
	// out /= out(2);

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
	for (int i = 0; i < imageOutputHeight; i++)
	{
		for (int j = 0; j < imageOutputWidth; j++)
		{
			in << bound.minX + j*ratio, bound.minY + i*ratio, 1;
			out = H2 * in;
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

void Image::DrawPanoramicPictureUsingQT(const string & outputPanoramic)
{
	imageOutputHeight = OUTPUT_HEIGHT;
	imageOutputWidth = (bound.maxX - bound.minX)*((double)imageOutputHeight)/(bound.maxY - bound.minY);

	double ratio = (bound.maxX - bound.minX)/(double)(imageOutputWidth);

	VectorXd in(3);
	VectorXd out;

	QImage newImage = QImage(imageOutputWidth,  imageOutputHeight, QImage::Format_ARGB32);

    QPainter painter;
    painter.begin(&newImage);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setBackgroundMode(Qt::TransparentMode);
    QPen pen;

    //first image
	MatrixXd H1_INV = H1.inverse().eval();
	for (int i = 0; i < imageOutputHeight; i++)
	{
		for (int j = 0; j < imageOutputWidth; j++)
		{
            in << bound.minY + j*ratio, bound.minX + i*ratio, 1;
			out = H1_INV * in;
			out /= out(2);
			
            if (out(0,0) >= 0 && out(0,0) <= inputImage1.width() - 1
                    && out(1,0) >= 0 && out(1,0) <= inputImage1.height() - 1)
            {
                // Point lies inside the input Image

                // Do interpolation
                QColor c = interpolate(inputImage1, out);
                //QRgb clrCurrent( inputImage.pixel( y(0,0), y(1,0) ) );
                pen.setColor(c);
                pen.setWidth(1);
                pen.setCapStyle(Qt::RoundCap);
                painter.setPen(pen);
                painter.drawPoint(j, i);

            }
            /*else
            {
                // Point lies outside the input Image
                QColor clrCurrent(0,0,0);
                pen.setColor(clrCurrent);
                pen.setWidth(1);
                pen.setCapStyle(Qt::RoundCap);
                painter.setPen(pen);
                painter.drawPoint(j, i);
            }*/
        }
  	}
    
    //second image
	MatrixXd H2_INV = H2.inverse().eval();
	for (int i = 0; i < imageOutputHeight; i++)
	{
		for (int j = 0; j < imageOutputWidth; j++)
		{
            in << bound.minY + j*ratio, bound.minX + i*ratio, 1;
			out = H2_INV * in;
			out /= out(2);
			
            if (out(0,0) >= 0 && out(0,0) <= inputImage2.width() - 1
                    && out(1,0) >= 0 && out(1,0) <= inputImage2.height() - 1)
            {
                // Point lies inside the input Image

                // Do interpolation
                QColor c = interpolate(inputImage2, out);
                //QRgb clrCurrent( inputImage.pixel( y(0,0), y(1,0) ) );
                pen.setColor(c);
                pen.setWidth(1);
                pen.setCapStyle(Qt::RoundCap);
                painter.setPen(pen);
                painter.drawPoint(j, i);

            }
            /*else
            {
                // Point lies outside the input Image
                QColor clrCurrent(0,0,0);
                pen.setColor(clrCurrent);
                pen.setWidth(1);
                pen.setCapStyle(Qt::RoundCap);
                painter.setPen(pen);
                painter.drawPoint(j, i);
            }*/
        }
      }

    painter.end();

    QImageWriter writer(outputPanoramic.c_str());
    writer.write(newImage);
}


QColor Image::interpolate(QImage img, MatrixXd y)
{
    double mappedX;
    double mappedY;
    int mappedXFloor, mappedXCeil, mappedYFloor, mappedYCeil;

    mappedX = y(0,0)/y(2,0);
    mappedY = y(1,0)/y(2,0);

    mappedXFloor = floor(mappedX);
    mappedXCeil = ceil(mappedX);
    mappedYFloor = floor(mappedY);
    mappedYCeil = ceil(mappedY);

    QColor leftTop (img.pixel(mappedXFloor, mappedYFloor));
    QColor rightTop (img.pixel(mappedXCeil, mappedYFloor));
    QColor leftBottom (img.pixel(mappedXFloor, mappedYCeil));
    QColor rightBottom (img.pixel(mappedXCeil, mappedYCeil));

    QColor a ( (((mappedXCeil - y(0,0)) * leftTop.red()) + ( (y(0,0) - mappedXFloor) * rightTop.red())),
               (((mappedXCeil - y(0,0)) * leftTop.green()) + ( (y(0,0) - mappedXFloor) * rightTop.green())),
               (((mappedXCeil - y(0,0)) * leftTop.blue()) + ( (y(0,0) - mappedXFloor) * rightTop.blue()))
               );
    QColor b ( (((mappedXCeil - y(0,0)) * leftBottom.red()) + ( (y(0,0) - mappedXFloor) * rightBottom.red())),
               (((mappedXCeil - y(0,0)) * leftBottom.green()) + ( (y(0,0) - mappedXFloor) * rightBottom.green())),
               (((mappedXCeil - y(0,0)) * leftBottom.blue()) + ( (y(0,0) - mappedXFloor) * rightBottom.blue()))
               );

    QColor c ( ((mappedYCeil - y(1,0)) * a.red() + ( (y(1,0) - mappedYFloor) * b.red() )),
               ((mappedYCeil - y(1,0)) * a.green() + ( (y(1,0) - mappedYFloor) * b.green() )),
               ((mappedYCeil - y(1,0)) * a.blue() + ( (y(1,0) - mappedYFloor) * b.blue() ))
               );

    return c;

}
