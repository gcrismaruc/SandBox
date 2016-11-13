#pragma once

#include <sstream>
#include "DepthMap.h"

using namespace std;
using namespace cv;

class Calibration {

public:
	Aquisition aquisition;
	static int nrFrame;
	static int nrClick;
	static int x1;
	static int x2;
	static int y1;
	static int y2;
	static CvMat *solutieQ;

	int config[12][6];

	Calibration(Aquisition *aquisition);
	Calibration();
	~Calibration();
	void createAxis(int x1, int y1, int x2, int y2, int x3, int y3, Mat *image);
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

};

