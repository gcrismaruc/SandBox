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
	static const int NR_POINTS;

	Aquisition aquisition;
	static int nrFrame;
	static int nrClick;
	static int x1;
	static int x2;
	static int y1;
	static int y2;
	static CvMat *solutieQ;
	static Mat homographyMatrix;
	static Mat projection;

	static float currentXCenter;
	static float currentYCenter;
	static float currentZCenter;

	static double zFar;
	static double zNear;


	static int lowThreshold;

	static Mat thresh;

	int config[36][6];

	vector<pair<unsigned int, unsigned int>> vecIntersectionPoints;
	vector<Point3f> vecOfDetectedCoordinates;
	vector<Point2f> vecOfDetectedPixelCoordinates;
	 

	int detectCircelsForDisplay(Mat eightBitsImage, Mat sixteenBitsImage);
	int getFramesAfterCalibration(USHORT * imageArray);
	void updateCenterOfCurrentCircle();

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
	Point2f * transformPoint(Point3f kinect);
	void doTransformationOfImage();


	int detectCircleForImage(Mat eightBitsImage, Mat sixteenBitsImage);
	void sendImageAtClick(Mat* depthImage);

	static void CannyThreshold(int, void *);

	void solvePnP();
	void solveSVD();

	void scalingImage();
	void loadCalibration();

	void zhangEstimation();
};

