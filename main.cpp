#include <iostream>
#include <stdint.h>
#include <list>
#include <opencv2\opencv.hpp>
#include "DepthMap.h"
#include "Calibration.h"
#include "Aquisition.h"


using namespace std;
using namespace cv;

enum StepDirection
  {
    None,
    Up,
    Left,
    Down,
    Right
 };

bool isColored(int x, int y, Mat image) {
	if(x >= image.size().width || x < 0 || y >= image.size().height || y < 0) {
		return false;
	}
	
	int pixel = (int)image.at<uchar>(y, x);
	if(pixel == 0)
		return true;

	return false;
}

Point firstPixelToAdd(Mat image) {
	Size size = image.size();
	int width = size.width;
	int height = size.height;

	for(int y = 0; y < height; y++) {
		for(int x = 0; x < width; x++){
			int pixel = (int)image.at<uchar>(y,x);
			if(pixel == 0) {
				return Point(x, y);
			}
		}
	}

	return NULL;
}

StepDirection nextDirection = Up;
StepDirection direction = None;

void step(int x, int y, Mat image) {

	bool upLeft = isColored(x-1,y-1, image);
	bool upRight = isColored(x, y-1, image);
	bool downLeft = isColored(x-1, y, image);
	bool downRight = isColored(x, y, image);

	direction = nextDirection;

	char state = 0;
	if(upLeft)
		state |= 1;
	if(upRight)
		state |= 2;
	if(downLeft)
		state |= 4;
	if(downRight)
		state |= 8;

	switch(state) {
		case 1: 
			nextDirection = Up;
			break;
		case 2:
			nextDirection = Right;
			break;
		case 3:
			nextDirection = Right;
			break;
		case 4:
			nextDirection = Left;
			break;
		case 5:
			nextDirection = Up;
			break;
		case 6:
			if(direction == Up) {
				nextDirection = Left;
			} else {
				nextDirection = Right;
			}
			break;
		case 7:
			nextDirection = Right;
			break;
		case 8:
			nextDirection = Down;
			break;
		case 9:
			if(direction == Right) {
				nextDirection = Up;
			} else {
				nextDirection = Down;
			}
			break;
		case 10:
			nextDirection = Down;
			break;
		case 11:
			nextDirection = Down;
			break;
		case 12:
			nextDirection = Left;
			break;
		case 13:
			nextDirection = Up;
			break;
		case 14:
			nextDirection = Left;
			break;
		default:
			nextDirection = None;
			break;
	}
}

list<Point> marchingSquares(Point point, Mat image) {

	int startX = point.x;
	int startY = point.y;

	int x = startX;
	int y = startY;
	Size size = image.size();
	int width = size.width;
	int height = size.height;
	list<Point> points;

	do {

		step(x, y, image);
		if(x >= 0 && x < width && y >= 0 && y < height)
			points.push_back(Point(x,y));

		switch(nextDirection) {
			case Up:
				y--;
				break;
			case Left:
				x--;
				break;
			case Right:
				x++;
				break;
			case Down:
				y++;
				break;
			default:
				break;
		}
	} while(x != startX || y != startY);
	
		return points;
}

void contourFunction(Mat image,  int treshold) {
	
	int height = image.size().height;
	int width = image.size().width;

	for(int y = 1; y < height - 1; y++) 
	{
		for(int x = 1; x < width - 1; x++)
		{
			if((int)image.at<uchar>(y, x) >= treshold)
			{
				if((int)image.at<uchar>(y + 1, x - 1) < treshold)
					image.at<uchar>(y + 1, x - 1) = 0;

				if((int)image.at<uchar>(y + 1, x) < treshold)
					image.at<uchar>(y + 1, x) = 0;

				if((int)image.at<uchar>(y + 1, x + 1) < treshold)
					image.at<uchar>(y + 1, x + 1) = 0;

				if((int)image.at<uchar>(y, x + 1) < treshold)
					image.at<uchar>(y, x + 1) = 0;

				if((int)image.at<uchar>(y - 1, x + 1) < treshold)
					image.at<uchar>(y - 1, x + 1) = 0;

				if((int)image.at<uchar>(y - 1, x) < treshold)
					image.at<uchar>(y - 1, x) = 0;

				if((int)image.at<uchar>(y - 1, x - 1) < treshold)
					image.at<uchar>(y - 1, x - 1) = 0;

				if((int)image.at<uchar>(y, x - 1) < treshold)
					image.at<uchar>(y, x - 1) = 0;
			}
		}
	}
}

int main(){

	/*Mat img = imread("C:\\Users\\Gheorghe\\Desktop\\depth.png", IMREAD_GRAYSCALE);
	
	Mat dst;
	int treshold = 1;
	Mat test = Mat::zeros(img.size().height, img.size().width, CV_8U);
	test = Scalar(255, 255, 255);
	threshold(img, dst, treshold, 255, THRESH_BINARY);
	while(treshold < 255) {
		contourFunction(img, treshold);

		treshold ++;
	}
	
	imwrite("C:/Users/Gheorghe/Desktop/test_contour.bmp", test);
	imshow("test1", dst);
	
	imshow("test", img);
	waitKey(0);
	
		//cout<<dst;
	//cout<<(int)startPoint[0]<<" "<<(int)startPoint[1];
	
	
	/*while(treshold != 255) {
		threshold(img, dst, treshold, 255, THRESH_BINARY);
		imshow("test", dst);
		waitKey(100);
		treshold++;
	}*/


	
	Aquisition *aq = new Aquisition();
	const int eventCount = 1;
    HANDLE hEvents[eventCount];

	hEvents[0] = aq->m_hNextDepthFrameEvent;
	HRESULT hr = aq->CreateFirstConnected();

	//MsgWaitForMultipleObjects(eventCount, hEvents, FALSE, INFINITE, QS_ALLINPUT);


	//
	//DepthMap *dm = new DepthMap(aq);
	//dm->createPalette(dm->palette, dm->paletteSize);
	

	//while(true) {
	//	dm->ColorTheImage(dm->aquisition.Update());
	//}

	Calibration *c = new Calibration(aq);

	//c->getMouseCoordinates();

	/*c->savePointsForAxes();
	c->readConfig();
	c->getFrames();*/
	


	//c->showCircles();
	
	c->getCoef();
	//c->solvePnP();

	//c->solveSVD();
	
	c->loadCalibration();
	c->doTransformationOfImage();


	
	
	return 0;
}