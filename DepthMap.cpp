#include "DepthMap.h"
#include <strsafe.h>
#include <iostream>
#include <string>
#include <fstream>

using namespace std;
using namespace cv;

int DepthMap::imageNumber = 0;

DepthMap::DepthMap(Aquisition *aquisition)
{
	// create heap storage for depth pixel data in RGBX format
	//m_depthRGBX = new BYTE[cDepthWidth*cDepthHeight*cBytesPerPixel];
	this->aquisition = *aquisition;
	paletteForDepth = imread("C:\\Users\\Gheorghe\\Desktop\\palette.bmp", CV_LOAD_IMAGE_COLOR);
	input = (unsigned char *)(this->paletteForDepth.data);

	paletteSize = 0;
}

int DepthMap::saveAFrame(BYTE * imageArray) {
	
		int pressedKey;
	
	
		Mat depthImage = Mat(Size(640, 480), CV_8UC4, imageArray).clone();
		Mat flipImage;
		flip(depthImage, flipImage, 1);

		Size size (640, 480);
		cvNamedWindow("sandBox", CV_WINDOW_NORMAL );
		//cvSetWindowProperty("sandBox", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
		Mat dst;

		resize(flipImage, dst, size);
		IplImage imagine = dst;
		cvShowImage("sandBox", &imagine);
		pressedKey = waitKey(0);	
		if(pressedKey == 49) {
			
			cvSaveImage("test.png", &imagine);
		}
		return pressedKey;
	
}

void DepthMap::ColorTheImage(BYTE * imageArray)
{

	//cout<<this->paletteForDepth;

	//cout<<(int)input[990]<<" "<<(int)input[991]<<" "<<(int)input[992]<<endl;
	for(int i = 0; i < aquisition.cDepthWidth * aquisition.cDepthHeight * aquisition.cBytesPerPixel; i+=4)
	{
		int grayLevel = (int)imageArray[i];
		int index = 3 * 4 * grayLevel;

		imageArray[i]   = this->input[index];
		imageArray[i+1] = this->input[index + 1];
		imageArray[i+2] = this->input[index + 2];

		/*imageArray[i]   = this->palette[grayLevel].blue;
		imageArray[i+1] = this->palette[grayLevel].green;
		imageArray[i+2] = this->palette[grayLevel].red;*/

		/*if(imageArray[i] > 255 || imageArray[i+1] >255 || imageArray[i+2] >255){*/
		/*cout<<(int)imageArray[i]<< " "<<(int)input[index]<<" index"<<index<<" grayLevel"<<grayLevel<<endl;
		cout<<(int)imageArray[i+1]<< " "<<(int)input[index+1]<<endl;
		cout<<(int)imageArray[i+2]<< " "<<(int)input[index+2]<<endl;
		*/
	}



	Mat depthImage = Mat(Size(640, 480), CV_8UC4, imageArray).clone();
	Mat flipImage;
	flip(depthImage, flipImage, 1);

	Size size (640, 480);
	cvNamedWindow("sandBox", CV_WINDOW_NORMAL );
	cvSetWindowProperty("sandBox", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	Mat dst;

	line(flipImage, Point(15, 20), Point(15, 980), Scalar(250, 250, 250), 2, 8);
	line(flipImage, Point(5, 400), Point(450, 400), Scalar(250, 250, 250), 2, 8);
	resize(flipImage, dst, size);
	IplImage imagine = dst;
	cvShowImage("sandBox", &imagine);
	waitKey(10);	
}

void DepthMap::onMouse( int event, int x, int y, int, void* )
{
	cout<<"in callback \n";
    if( event != CV_EVENT_LBUTTONDOWN )
            return;

    Point pt = Point(x,y);
    std::cout<<"x="<<pt.x<<"\t y="<<pt.y<<"\n";

}

void DepthMap::createPalette(color palette[256], int &paletteSize)
{

	color colors[13];
	colors[0].blue = 48;
	colors[0].green = 1;
	colors[0].red = 1;

	colors[1].blue = 245;
	colors[1].green = 24;
	colors[1].red = 29;

	colors[2].blue = 254;
	colors[2].green = 255;
	colors[2].red = 0;

	colors[3].blue = 56;
	colors[3].green = 100;
	colors[3].red = 0;

	colors[4].blue = 0;
	colors[4].green = 127;
	colors[4].red =	63;

	colors[5].blue = 72;
	colors[5].green = 153;
	colors[5].red = 3;

	colors[6].blue = 86;
	colors[6].green = 178;
	colors[6].red = 54;

	colors[7].blue = 119;
	colors[7].green = 204;
	colors[7].red = 116;

	colors[8].blue = 161;
	colors[8].green = 229;
	colors[8].red = 173;

	colors[9].blue = 209;
	colors[9].green = 255;
	colors[9].red = 229;

	colors[10].blue = 107;
	colors[10].green = 192;
	colors[10].red = 186;

	colors[11].blue = 35;
	colors[11].green = 90;
	colors[11].red = 184;

	colors[12].blue = 0;
	colors[12].green = 42;
	colors[12].red = 105;

	for(int i = 1; i < 13; i++)
	{
		color A = colors[i-1];
		color B = colors[i];

		int colorWidth = 256 / 13;

		for(int j = 0; j < colorWidth; j++)
		{
			float p = j / (float)colorWidth;

			this->palette[this->paletteSize].red = A.red * (1-p) + B.red * p;
			this->palette[this->paletteSize].green = A.green * (1-p) + B.green * p;
			this->palette[this->paletteSize].blue = A.blue * (1-p) + B.blue * p;

			this->paletteSize++;
		}
	}

	for(int i = this->paletteSize; i < 256; i++)
	{
		this->palette[i].red = this->palette[this->paletteSize - 1].red;
		this->palette[i].green = this->palette[this->paletteSize - 1].green;
		this->palette[i].blue = this->palette[this->paletteSize - 1].blue;

		this->paletteSize++;
	}
}