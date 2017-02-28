#pragma once

#include <sstream>
#include <random>
#include "DepthMap.h"

using namespace std;
using namespace cv;

class Calibration {

public:
	static double FX;
	static double CX;
	static double FY;
	static double CY; 
	static int THRESH_MIN;
	static int THRESH_MAX;

	Aquisition aquisition;
	static int nrFrame;
	static int nrClick;
	static int x1;
	static int x2;
	static int y1;
	static int y2;
	static CvMat *solutieQ;
	static Mat homographyMatrix;

	static int lowThreshold;

	static Mat thresh;

	int config[36][6];

	vector<pair<unsigned int, unsigned int>> vecIntersectionPoints;
	vector<Point3f> vecOfDetectedCoordinates;

	 

	Calibration(Aquisition *aquisition);
	Calibration();
	~Calibration();
	void createAxis(int x1, int y1, int x2, int y2, int x3, int y3, Mat *image);
	void createAxis(int x, int y, Mat *image);

	int showAxes(USHORT * imageArray);
	void getFrames();
	void showCircles();
	void readConfig();
	static void CallBackFunc(int event, int x, int y, int flags, void* userdata);
	void getMouseCoordinates();
	void setROI(Mat &img, int x1, int x2, int y1, int y2);
	void savePointsForAxes();
	void detectCircle();
	void getCoef();
	Point2d * transformPoint(Point3d kinect);
	void doTransformationOfImage();
	void drawGrid(Mat image);

	int detectCircleForImage(Mat eightBitsImage, Mat sixteenBitsImage);
	void sendImageAtClick(Mat* depthImage);

	static void CannyThreshold(int, void *);

	void scalingImage();
	void loadCalibration();
};

