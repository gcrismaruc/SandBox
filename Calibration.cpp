#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "Calibration.h"
#include <strsafe.h>
#include <iostream>
#include <string>
#include <iomanip>
#include <fstream>
#include <math.h>
#include <NuiSensor.h>
#include <NuiImageCamera.h>
#include <NuiApi.h>
#include <NuiSkeleton.h>
#include "opencv2\core\types.hpp"


int Calibration::nrFrame = 0;
int Calibration::nrClick = 0;
int Calibration::x1 = 0;
int Calibration::y1 = 0;
int Calibration::x2 = 640;
int Calibration::y2 = 480;
const int Calibration::NR_POINTS = 60;
int Calibration::THRESH_MAX = 1255;
int Calibration::THRESH_MIN = 935;
int Calibration::WIDTH = 640;
int Calibration::HEIGHT = 480;


double Calibration::zFar = 0.0;
double Calibration::zNear = 0.0;


float Calibration::currentYCenter = 0.0;
float Calibration::currentXCenter = 0.0;
float Calibration::currentZCenter = 0.0;

Mat Calibration::thresh = Mat(Size(640, 480), CV_8UC1);
int Calibration::lowThreshold = 100;

CvMat *Calibration::solutieQ = cvCreateMat(12, 1, CV_64FC1);
Mat Calibration::homographyMatrix = Mat(3, 4, CV_32FC1);
Mat Calibration::projection = Mat(4, 4, CV_32FC1);


Calibration::Calibration(Aquisition *aquisition) {
	this->aquisition = *aquisition;
}

Calibration::Calibration(){

}

Calibration::~Calibration(){

}

void Calibration::createAxis(int x1, int y1, int x2, int y2, int x3, int y3, Mat *image) {
	line(*image, Point(x1, y1), Point(x2, y2), Scalar(32000, 32000, 32000), 2, 8);
	line(*image, Point(x2, y2), Point(x3, y3), Scalar(32000, 32000, 32000), 2, 8);
}


void Calibration::createAxis(int x, int y, Mat *image) {
	line(*image, Point(x - 10, y), Point(x + 10, y), Scalar((unsigned short)-1), 1, 8);
	line(*image, Point(x, y - 10), Point(x, y + 10), Scalar((unsigned short)-1), 1, 8);
}

int Calibration::showAxes(USHORT * imageArray) {

	Mat depthImage = Mat(Size(640, 480), CV_16UC1, imageArray);
	Mat projectedImage = Mat(Size(640, 480), CV_16UC1, Scalar(0, 0, 0));
	this->createAxis(vecIntersectionPoints[nrFrame].first, vecIntersectionPoints[nrFrame].second, &projectedImage);


	int newWidth = abs(x2 - x1);
	int newHeight = abs(y2 - y1);

	cvNamedWindow("axis", CV_WINDOW_NORMAL);
	cvSetWindowProperty("axis", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

	imshow("axis", projectedImage);
	int pressedKey = waitKey(10);

	int checkDetectedCircle = -1;

	if (pressedKey == 49){
		Mat depthImageForCircle = Mat(Size(640, 480), CV_8UC1);
		depthImage.clone().convertTo(depthImageForCircle, CV_8UC1, 1.0 / 255.0);

		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(100);
		const string imageName = "calibration\\calibration" + to_string(nrFrame) + ".png";

		cout << imageName << endl;
		imwrite(imageName, depthImage, compression_params);

		checkDetectedCircle = detectCircleForImage(depthImageForCircle, depthImage);

		if (checkDetectedCircle == 0) {
			nrFrame++;
		}

		depthImageForCircle.release();
	}

	
	projectedImage.release();
	depthImage.release();

	return (checkDetectedCircle == -1) ? checkDetectedCircle : pressedKey;
}

int Calibration::getFramesAfterCalibration(USHORT * imageArray){
	Mat depthImage = Mat(Size(640, 480), CV_16UC1, imageArray);

	int checkDetectedCircle = -1;

	Mat depthImageForCircle = Mat(Size(640, 480), CV_8UC1);
	depthImage.clone().convertTo(depthImageForCircle, CV_8UC1, 1.0 / 255.0);

	checkDetectedCircle = detectCircelsForDisplay(depthImageForCircle, depthImage);

	if (checkDetectedCircle == 0) {
	}
	
	depthImage.release();
	depthImageForCircle.release();

	return (checkDetectedCircle == -1) ? checkDetectedCircle : 3;
}

void Calibration::updateCenterOfCurrentCircle() {
	int a = 0;

	while (a != 3){
		a = this->getFramesAfterCalibration(this->aquisition.Update());
	}	
}

void Calibration::getFrames(){

	ofstream xyp("coordonateXYPuncteDetectate.txt");
	int a = 0;
	int count = 0;

	while (count != NR_POINTS) {
		cout << "aici\n";
		while (a != 49){
			a = this->showAxes(this->aquisition.Update());
		}
		count++;
		a = 0;
	}

	cout << "End of getFrames\n";
	cout << "Size of vector : " << vecOfDetectedCoordinates.size() << endl;
	ofstream fcout("coordonateDetectate.txt");

	for (vector<Point3f>::iterator it = vecOfDetectedCoordinates.begin(); it != vecOfDetectedCoordinates.end(); it++)
	{
		fcout << it->x << " " << it->y << " " << it->z << endl;
	}

	for (auto point : vecOfDetectedPixelCoordinates){
		xyp << point.x << "  " << point.y << endl;
	}

	fcout.close();
	xyp.close();
	cvDestroyWindow("axis");

}

void Calibration::CannyThreshold(int, void *) {
	Canny(thresh, thresh, lowThreshold, lowThreshold * 3, 3);

	Mat dst;
	thresh.copyTo(dst, thresh);
	imshow("Edge Map", dst);
	waitKey(10);
}

int Calibration::detectCircelsForDisplay(Mat eightBitsImage, Mat sixteenBitsImage) {
	int retValue = 0;
	Mat src_gray;
	thresh = eightBitsImage.clone();

	int nrPixels = 0;
	int nr_x = 0;
	int nr_y = 0;

	Mat labels, stats, centroids;
	int noComp;

	for (int y = 0; y < 480; y++) {
		for (int x = 0; x < 640; x++) {
			if (!(sixteenBitsImage.at<ushort>(Point(x, y)) >= THRESH_MIN && sixteenBitsImage.at<ushort>(Point(x, y)) <= THRESH_MAX))
			{
				thresh.at<uchar>(Point(x, y)) = 255;
			}
			else {
				thresh.at<uchar>(Point(x, y)) = 0;
			}
		}
	}

	Mat lower_red_hue_range;
	Mat upper_red_hue_range;

	medianBlur(thresh, thresh, 3);
	GaussianBlur(thresh, thresh, Size(9, 9), 3, 3);

	vector<cv::Vec3f> circles;

	int thresholdMaxForHough = 4;
	HoughCircles(thresh, circles, HOUGH_GRADIENT, 2, thresh.rows / 8, thresholdMaxForHough, 60, 10, 100);
	int x, y, z;
	if (circles.size() == 0) {
		retValue = -1;
	}

	z = 0;
	x = 0;
	y = 0;

	if (circles.size() == 1){
		//for (size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
		size_t current_circle = 0;
		cv::Point center(std::round(circles[current_circle][0]), std::round(circles[current_circle][1]));
		int radius = std::round(circles[current_circle][2]);

		x = circles[current_circle][0];
		y = circles[current_circle][1];
		z = (int)sixteenBitsImage.at<ushort>(Point(x, y));

		cv::circle(thresh, center, radius, cv::Scalar(255, 0, 0), 5);
		cv::circle(thresh, center, radius, Scalar(1322, 0, 0), 3, 8, 0);

		//}
	}


	imshow("teststs", thresh);
	waitKey(1);
	//iau coordonate in mm

	Vector4 worldCoordinates;
	worldCoordinates = NuiTransformDepthImageToSkeleton((long)x, (long)y, z << 3, NUI_IMAGE_RESOLUTION_640x480);

	if (x == 48 || y == 48 || worldCoordinates.x == 0 || worldCoordinates.y == 0){
		retValue = -1;
	}
	
	float delta = 0.0017505;
	
	//salvez coordinate in milimeters
	if (retValue == 0){
		currentXCenter = (x * z * delta - 320 * z * delta) / 10;
		currentYCenter = (-y * z * delta + 240 * z * delta) / 10;
		currentZCenter = z / 10;
		cout << "x = " << currentXCenter << "y = " << currentYCenter << "z = " << currentZCenter << endl;

	}
	return retValue;
}

//detectez cercul si salvez coordonatele in fisier
int Calibration::detectCircleForImage(Mat eightBitsImage, Mat sixteenBitsImage) {

	int retValue = 0;
	Mat src_gray;
	thresh = eightBitsImage.clone();

	int nrPixels = 0;
	int nr_x = 0;
	int nr_y = 0;

	Mat labels, stats, centroids;
	int noComp;

	for (int y = 0; y < 480; y++) {
		for (int x = 0; x < 640; x++) {
			if (!(sixteenBitsImage.at<ushort>(Point(x, y)) > THRESH_MIN && sixteenBitsImage.at<ushort>(Point(x, y)) < THRESH_MAX))
			{
				thresh.at<uchar>(Point(x, y)) = 255;
			}
			else {
				thresh.at<uchar>(Point(x, y)) = 0;
			}
		}
	}

	Mat lower_red_hue_range;
	Mat upper_red_hue_range;

	medianBlur(thresh, thresh, 3);
	GaussianBlur(thresh, thresh, Size(9, 9), 3, 3);

	vector<cv::Vec3f> circles;

	int thresholdMaxForHough = 4;
	HoughCircles(thresh, circles, HOUGH_GRADIENT, 2, thresh.rows / 8, thresholdMaxForHough, 60, 10, 100);
	int x = 0;
	int y = 0;
	int z = 0;
	if (circles.size() == 0) {

		retValue = -1;
	}
	if (circles.size() == 1){
		//for (size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
		size_t current_circle = 0;
		cv::Point center(std::round(circles[current_circle][0]), std::round(circles[current_circle][1]));
		int radius = std::round(circles[current_circle][2]);

		x = circles[current_circle][0];
		y = circles[current_circle][1];
		z = (int)sixteenBitsImage.at<ushort>(Point(x, y));

		cv::circle(thresh, center, radius, cv::Scalar(125, 12, 21), 5);
		cv::circle(thresh, center, radius, Scalar(255, 0, 0), 3, 8, 0);

		//}
	}


	imshow("teststs", thresh);
	waitKey(10);

	//Get world coordiantes for center
	Vector4 worldCoordinates = NuiTransformDepthImageToSkeleton((long)x, (long)y, z << 3, NUI_IMAGE_RESOLUTION_640x480);

	//coordinates in meters
	cout << x << " " << y << "real coordinates : " << (worldCoordinates.x * 1000) << " " << (worldCoordinates.y * 1000) << " " << (worldCoordinates.z * 1000) << endl;

	if (x == 48 || y == 48 || worldCoordinates.x == 0 || worldCoordinates.y == 0){
		retValue = -1;
	}

	//save coordinate in milimeters
	if (retValue == 0){
		vecOfDetectedCoordinates.push_back(Point3f((worldCoordinates.x), (worldCoordinates.y), (worldCoordinates.z)));
		vecOfDetectedPixelCoordinates.push_back(Point2f(x, y));
	}
	return retValue;
}

void Calibration::showCircles() {

	ofstream file("coordonateDetectate.txt");

	int nrFrame = 0;

	while (nrFrame != 36){
		const string imageName = "calibration\\calibration" + to_string(nrFrame) + ".png";;

		Mat src = imread(imageName, CV_LOAD_IMAGE_UNCHANGED | CV_LOAD_IMAGE_ANYDEPTH);
		Mat src_gray;
		Mat tresh = src.clone();

		cout << "\n\t nr.channels = " << src.channels();

		int nrPixels = 0;
		int nr_x = 0;
		int nr_y = 0;
		for (int y = 0; y < 480; y++){
			for (int x = 0; x < 640; x++)
			{

				if (src.at<ushort>(Point(x, y)) != 0 && !(src.at<ushort>(Point(x, y)) > 1090 && src.at<ushort>(Point(x, y)) < 1230))
				{
					nrPixels++;
					nr_x += x;
					nr_y += y;

					src.at<ushort>(Point(x, y)) = 54000;
				}
				else {
					src.at<ushort>(Point(x, y)) = 0;
				}
			}
			cout << endl;
		}
		imshow("tresh", src);


		int x = nr_x / nrPixels;
		int y = nr_y / nrPixels;
		int z = (int)tresh.at<ushort>(Point(x, y));
		Point center(x, y);
		src.at<ushort>(Point(x, y)) = 0;
		file << x << " " << y << " " << z << "\n";
		imshow("test", src);
		waitKey(0);
		nrFrame++;
	}

	file.close();

}

//citesc coordonatele proiectate
void Calibration::readConfig() {

	ifstream file("coordonateProiector.txt");

	while (!file.eof()){
		unsigned int x, y;
		file >> x >> y;
		vecIntersectionPoints.push_back(make_pair(x, y));
	}

	file.close();
}

void Calibration::sendImageAtClick(Mat* depthImage) {

	Mat depthImageForCircle = Mat(Size(640, 480), CV_8UC1);
	depthImage->convertTo(depthImageForCircle, CV_8UC1, 1.0 / 255.0);
	vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	compression_params.push_back(9);
	const string imageName = "calibration\\calibration" + to_string(nrFrame) + ".png";

	cout << imageName << endl;
	imshow("8bitsTest", depthImageForCircle);
	detectCircleForImage(depthImageForCircle, *depthImage);
	nrFrame++;
}

void Calibration::CallBackFunc(int event, int x, int y, int flags, void* userdata){
	if (flags == (EVENT_FLAG_LBUTTON))
	{
		nrClick++;
		if (nrClick == 1){
			x1 = x;
			y1 = y;
		}
		else {
			x2 = x;
			y2 = y;
		}
		cout << "Left mouse button is clicked while pressing CTRL key - position (" << x << ", " << y << ")" << endl;
	}
}

void Calibration::getMouseCoordinates(){

	ofstream f("roi.txt");
	while (nrClick != 2) {
		Mat img = Mat(Size(640, 480), CV_16UC1, this->aquisition.Update()).clone();

		for (int y = 0; y < 480; y++){
			for (int x = 0; x < 640; x++){
				img.at<ushort>(Point(x, y)) += 5000;
			}
		}
		//flip(img, img, 1);
		namedWindow("GetCoordinates", CV_WINDOW_NORMAL);
		cvSetWindowProperty("GetCoordinates", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
		setMouseCallback("GetCoordinates", CallBackFunc, NULL);
		imshow("GetCoordinates", img);
		waitKey(10);
	}

	cvDestroyWindow("GetCoordinates");
	f << x1 << " " << y1 << endl;
	f << x2 << " " << y2 << endl;
	f.close();
}

void Calibration::setROI(Mat &img, int x1, int x2, int y1, int y2){
	Size size = img.size();
	int width = size.width;
	int height = size.height;

	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			if (x < x1 || y < y1 || y > y2 || x > x2)
			{
				img.at<ushort>(Point(x, y)) = 0;
			}
		}
	}
}

//salvez coordonatele pentru proiector in fisier, dupa ce la generez random
void Calibration::genereazaPuncteAleatoare(){

	ofstream g("coordonateProiector.txt");
	ofstream f("coordinates.txt");


	random_device rd;     // only used once to initialise (seed) engine
	mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
	uniform_int_distribution<int> x_rnd(30, x2 - 70);
	uniform_int_distribution<int> y_rnd(30, y2 - 70);


	for (int i = 0; i < NR_POINTS; i++){
		int x = x_rnd(rng);
		int y = y_rnd(rng);

		f << x << " " << y << endl;
		g << x << " " << y << endl;
		cout << x << " " << y << endl;
	}

	f.close();
	g.close();
}


void Calibration::genereazaPuncteLaPozitiiSpecificate() {
	ofstream g("coordonateProiector.txt");
	ofstream f("coordinates.txt");

	int xPas = (550 * 3) / NR_POINTS;
	int yPas = 380 / 2;

	for (int y = 30; y <= 480; y += yPas) {
		for (int x = 45; x <= 580; x += xPas) {

			f << x << " " << y << endl;
			g << x << " " << y << endl;

			cout << x << " " << y << endl;
		}
			
	}

	f.close();
	g.close();
}

void Calibration::detectCircle(){

	Mat image = imread("calibration\\calibration1.png", 0);
	Mat dst;

	INuiCoordinateMapper *mapper;

	this->aquisition.m_pNuiSensor->NuiGetCoordinateMapper(&mapper);

	cvNamedWindow("circle", CV_WINDOW_NORMAL);
	for (int y = 0; y < 480; y++)
	for (int x = 1; x < 640; x++)
		image.at<ushort>(y, x) += 5000;

	imshow("circle", image);
	waitKey(10);
}

void Calibration::zhangEstimation(){
	ofstream logFile("FileLogZhangEstiomation.txt");

	Mat A = Mat(2 * NR_POINTS, 12, CV_64FC1);
	double x, y, z;

	std::vector<Point2d> puncteProiector;
	std::vector<Point3d> puncteKinect;

	//citesc coordonatele proiectate({x,y})
	ifstream proiectorFile("coordonateProiector.txt");
	for (int i = 0; i < NR_POINTS; i++){
		proiectorFile >> x >> y;
		puncteProiector.push_back(Point2d(x, y));
	}

	//citesc coordonatele detectate(centrul cercurilor {x, y, z})
	ifstream pp("coordonateXYPuncteDetectate.txt");
	ifstream kinectFile("coordonateDetectate.txt");
	for (int i = 0; i < NR_POINTS; i++){
		kinectFile >> x >> y >> z;
		//pp >> x >> y;
		z *= -1;
		x *= 1;
		y *= 1;
		puncteKinect.push_back(Point3d(x, y, z));
	}

	pp.close();
	proiectorFile.close();
	kinectFile.close();


	//creare matrice A pentru sistem
	int index = 0;
	for (int i = 0; i < 2 * NR_POINTS; i += 2) {

		A.at<double>(i, 0) = puncteKinect.at(index).x;
		A.at<double>(i, 1) = puncteKinect.at(index).y;
		A.at<double>(i, 2) = puncteKinect.at(index).z;
		A.at<double>(i, 3) = 1;
		A.at<double>(i, 4) = 0;
		A.at<double>(i, 5) = 0;
		A.at<double>(i, 6) = 0;
		A.at<double>(i, 7) = 0;
		A.at<double>(i, 8) = -puncteProiector.at(index).x * puncteKinect.at(index).x;
		A.at<double>(i, 9) = -puncteProiector.at(index).x * puncteKinect.at(index).y;
		A.at<double>(i, 10) = -puncteProiector.at(index).x * puncteKinect.at(index).z;
		A.at<double>(i, 11) = -puncteProiector.at(index).x;

		A.at<double>(i + 1, 0) = 0;
		A.at<double>(i + 1, 1) = 0;
		A.at<double>(i + 1, 2) = 0;
		A.at<double>(i + 1, 3) = 0;
		A.at<double>(i + 1, 4) = puncteKinect.at(index).x;
		A.at<double>(i + 1, 5) = puncteKinect.at(index).y;
		A.at<double>(i + 1, 6) = puncteKinect.at(index).z;
		A.at<double>(i + 1, 7) = 1;
		A.at<double>(i + 1, 8) = -puncteProiector.at(index).y * puncteKinect.at(index).x;
		A.at<double>(i + 1, 9) = -puncteProiector.at(index).y * puncteKinect.at(index).y;
		A.at<double>(i + 1, 10) = -puncteProiector.at(index).y * puncteKinect.at(index).z;
		A.at<double>(i + 1, 11) = -puncteProiector.at(index).y;

		index++;
	}

	logFile << "\n--------------------------Matricea A----------------------------------\n";
	for (int i = 0; i < 2 * NR_POINTS; i++){
		for (int j = 0; j < 12; j++){
			logFile << A.at<double>(i, j) << " ";
		}
		logFile << endl;
		//logFile << ";";
	}

	logFile << "\n------------------------Coordonate proiector----------------------------------\n";
	for (auto pointProiector : puncteProiector){
		logFile << pointProiector.x << " " << pointProiector.y << " \n";
	}

	logFile << "\n------------------------Coordonate Kinect----------------------------------\n";
	for (auto pointKinect : puncteKinect){
		logFile << pointKinect.x << " " << pointKinect.y << "  " << pointKinect.z << " \n";
	}

	Mat solutieValori;
	Mat solutieValoriVector;
	Mat ATranspusa;

	transpose(A, ATranspusa);
	Mat eigenInput = ATranspusa * A;

	Mat eigenInputdouble;
	eigenInput.convertTo(eigenInputdouble, CV_64FC1);

	eigen(eigenInputdouble, solutieValori, solutieValoriVector);

	logFile << "\n---------------------Valori singulare---------------------------------\n";
	for (int i = 0; i < 12; i++){
		logFile << solutieValori.at<double>(i, 0) << "   ";
	}

	logFile << "\n----------------------Vectori valori singulare-----------------------------\n";
	for (int i = 0; i < 12; i++){
		for (int j = 0; j < 12; j++){
			logFile << solutieValoriVector.at<double>(i, j) << "   ";
		}
		logFile << endl << endl << endl;
	}

	vector<double> vec;
	for (int i = 0; i < 12; i++){
		vec.push_back(solutieValoriVector.at<double>(i, 11));
	}

	cv::Mat H_Matrix = cv::Mat(3, 4, CV_64FC1);

	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 4; j++){
			H_Matrix.at<double>(i, j) = solutieValoriVector.at<double>(11, i * 4 + j);
			logFile << H_Matrix.at<double>(i, j) << " ";
		}
		logFile << endl;
	}



	cv::Mat B = cv::Mat(3, 3, CV_64FC1);
	cv::Mat b = cv::Mat(3, 1, CV_64FC1);

	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++) {
			B.at<double>(i, j) = H_Matrix.at<double>(i, j);
		}
	}

	b.at<double>(0, 0) = H_Matrix.at<double>(0, 3);
	b.at<double>(0, 1) = H_Matrix.at<double>(1, 3);
	b.at<double>(0, 2) = H_Matrix.at<double>(2, 3);

	logFile << "\n\n----------------------B--------------------------\n" << B;
	logFile << "\n\n---------------------b--------------------------\n" << b;

	cv::Mat Bt;
	transpose(B, Bt);
	logFile << "\n\n----------------------Bt--------------------------\n" << Bt;

	cv::Mat K = B*Bt;

	K /= K.at<double>(2, 2);

	logFile << "\n\n----------------------K--------------------------\n" << K;


	double u0 = K.at<double>(0, 2);
	double v0 = K.at<double>(1, 2);
	double ku = K.at<double>(0, 0);
	double kv = K.at<double>(1, 1);
	double kc = K.at<double>(0, 1);

	double beta = sqrt(kv - v0*v0);
	double gama = (kc - u0*v0) / beta;
	double alfa = sqrt(ku - u0*u0 - gama);

	logFile << "\n\nu0 v0 ku kv kc beta gama alfa " << endl;
	logFile << u0 << endl << v0 << endl << ku << endl << kv << endl << kc << endl << beta << endl << gama << endl << alfa << endl;

	cv::Mat A_z = cv::Mat::zeros(3, 3, CV_64FC1);
	A_z.at<double>(0, 0) = alfa;
	A_z.at<double>(0, 1) = gama;
	A_z.at<double>(0, 2) = u0;
	A_z.at<double>(1, 1) = beta;
	A_z.at<double>(1, 2) = v0;
	A_z.at<double>(2, 2) = 1;

	logFile << "\n\n----------------------A--------------------------\n" << A_z;

	cv::Mat A_inverse;
	invert(A_z, A_inverse);

	logFile << "\n\n----------------------A_inverse--------------------------\n" << A_inverse;

	cv::Mat R = A_inverse * B;
	cv::Mat t = A_inverse * b;

	logFile << "\n\n----------------------R--------------------------\n" << R;
	logFile << "\n\n----------------------t--------------------------\n" << t;

	cv::Mat viewModel = cv::Mat::zeros(4, 4, CV_64FC1);

	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++) {
			viewModel.at<double>(i, j) = R.at<double>(i, j);
		}
	}

	viewModel.at<double>(0, 3) = t.at<double>(0, 0);
	viewModel.at<double>(1, 3) = t.at<double>(1, 0);
	viewModel.at<double>(2, 3) = t.at<double>(2, 0);
	viewModel.at<double>(3, 3) = 1;

	cv::Mat glViewModel;
	transpose(viewModel, glViewModel);

	logFile << "\n\n----------------------viewModelt--------------------------\n" << viewModel;
	logFile << "\n\n----------------------glViewModel--------------------------\n" << glViewModel;

	double hFOV = 60.8;
	double vFOV = 45.8;

	double zfar = 1000;
	double znear = 0.01;

	cv::Mat projection = cv::Mat::zeros(4, 4, CV_64FC1);
	projection.at<double>(0, 0) = 1.0 / (640.0 / 480.0 * tan(hFOV / 2));
	projection.at<double>(1, 1) = 1.0 / tan(vFOV / 2);
	projection.at<double>(2, 2) = (zfar + znear) / (zfar - znear);
	projection.at<double>(2, 3) = 2 * (zfar * znear) / (zfar - znear);
	projection.at<double>(3, 2) = -1;

	logFile << "\n\n----------------------projection--------------------------\n" << projection;

	cv::Mat glProjMatrix;


	glProjMatrix = projection * glViewModel;

	logFile << "\n\n----------------------glProjMatrix--------------------------\n" << glProjMatrix;


	logFile.close();
}

//rezolvare sistem
void Calibration::getCoef(){

	ofstream logFile("FileLog.txt");

	Mat A = Mat(2 * NR_POINTS, 12, CV_64FC1);
	float x, y, z;

	std::vector<Point2d> puncteProiector;
	std::vector<Point3d> puncteKinect;

	//citesc coordonatele proiectate({x,y})
	ifstream proiectorFile("coordonateProiector.txt");
	for (int i = 0; i < NR_POINTS; i++){
		proiectorFile >> x >> y;
		puncteProiector.push_back(Point2d(x, y));
	}

	//citesc coordonatele detectate(centrul cercurilor {x, y, z})
	ifstream pp("coordonateXYPuncteDetectate.txt");
	ifstream kinectFile("coordonateDetectate.txt");
	for (int i = 0; i < NR_POINTS; i++){
		kinectFile >> x >> y >> z;
		//pp >> x >> y;
		z *= -100;
		x *= 100;
		y *= 100;
		puncteKinect.push_back(Point3d(x, y, z));
	}

	pp.close();
	proiectorFile.close();
	kinectFile.close();


	//creare matrice A pentru sistem
	int index = 0;
	for (int i = 0; i < 2 * NR_POINTS; i += 2) {

		A.at<double>(i, 0) = puncteKinect.at(index).x;
		A.at<double>(i, 1) = puncteKinect.at(index).y;
		A.at<double>(i, 2) = puncteKinect.at(index).z;
		A.at<double>(i, 3) = 1;
		A.at<double>(i, 4) = 0;
		A.at<double>(i, 5) = 0;
		A.at<double>(i, 6) = 0;
		A.at<double>(i, 7) = 0;
		A.at<double>(i, 8) = -puncteProiector.at(index).x * puncteKinect.at(index).x;
		A.at<double>(i, 9) = -puncteProiector.at(index).x * puncteKinect.at(index).y;
		A.at<double>(i, 10) = -puncteProiector.at(index).x * puncteKinect.at(index).z;
		A.at<double>(i, 11) = -puncteProiector.at(index).x;

		A.at<double>(i + 1, 0) = 0;
		A.at<double>(i + 1, 1) = 0;
		A.at<double>(i + 1, 2) = 0;
		A.at<double>(i + 1, 3) = 0;
		A.at<double>(i + 1, 4) = puncteKinect.at(index).x;
		A.at<double>(i + 1, 5) = puncteKinect.at(index).y;
		A.at<double>(i + 1, 6) = puncteKinect.at(index).z;
		A.at<double>(i + 1, 7) = 1;
		A.at<double>(i + 1, 8) = -puncteProiector.at(index).y * puncteKinect.at(index).x;
		A.at<double>(i + 1, 9) = -puncteProiector.at(index).y * puncteKinect.at(index).y;
		A.at<double>(i + 1, 10) = -puncteProiector.at(index).y * puncteKinect.at(index).z;
		A.at<double>(i + 1, 11) = -puncteProiector.at(index).y;

		index++;
	}

	logFile << "\n--------------------------Matricea A----------------------------------\n";
	for (int i = 0; i < 2 * NR_POINTS; i++){
		for (int j = 0; j < 12; j++){
			logFile << A.at<double>(i, j) << " ";
		}
		logFile << endl;
		//logFile << ";";
	}

	logFile << "\n------------------------Coordonate proiector----------------------------------\n";
	for (auto pointProiector : puncteProiector){
		logFile << pointProiector.x << " " << pointProiector.y << " \n";
	}

	logFile << "\n------------------------Coordonate Kinect----------------------------------\n";
	for (auto pointKinect : puncteKinect){
		logFile << pointKinect.x << " " << pointKinect.y << "  " << pointKinect.z << " \n";
	}

	Mat solutieValori;
	Mat solutieValoriVector;
	Mat ATranspusa;

	transpose(A, ATranspusa);
	Mat eigenInput = ATranspusa * A;

	Mat eigenInputFloat;
	eigenInput.convertTo(eigenInputFloat, CV_32FC1);

	eigen(eigenInputFloat, solutieValori, solutieValoriVector);

	logFile << "\n---------------------Valori singulare---------------------------------\n";
	for (int i = 0; i < 12; i++){
		logFile << solutieValori.at<float>(i, 0) << "   ";
	}

	logFile << "\n----------------------Vectori valori singulare-----------------------------\n";
	for (int i = 0; i < 12; i++){
		for (int j = 0; j < 12; j++){
			logFile << solutieValoriVector.at<float>(i, j) << "   ";
		}
		logFile << endl << endl << endl;
	}

	logFile << "\n----------------------Homography----------------------------------\n";
	//create homography matrix
	//matricea de transformare(rotatie+translatie)
	vector<float> vec;
	for (int i = 0; i < 12; i++){
		vec.push_back(solutieValoriVector.at<float>(i, 11));
	}

	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 4; j++){
			homographyMatrix.at<float>(i, j) = solutieValoriVector.at<float>(11, i * 4 + j);
			logFile << homographyMatrix.at<float>(i, j) << " ";
		}
		logFile << endl;
	}

	//calculate scale
	double h20 = homographyMatrix.at<float>(2, 0);
	double h21 = homographyMatrix.at<float>(2, 1);
	double h22 = homographyMatrix.at<float>(2, 2);
	double wLen = sqrt(h20*h20 + h21*h21 + h22*h22);

	cout << "\n\nwLen " << wLen << endl;
	int nrNegativeWeights = 0;
	for (auto pointKinect : puncteKinect){
		float sum = h20*pointKinect.x + h21*pointKinect.y + h22*pointKinect.z;
		sum += homographyMatrix.at<float>(2, 3);

		if (sum < 0.0)
			nrNegativeWeights++;
	}

	if (nrNegativeWeights == 0 || nrNegativeWeights == NR_POINTS){
		if (nrNegativeWeights > 0)
			wLen = -wLen;
		cout << "nrNegativeWeights = " << nrNegativeWeights << endl;
	}
	else{
		cout << "Ponderi negative\n";
	}

	cout << "Scale factor: " << wLen << endl;

	//scalare homography
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 4; j++){
			homographyMatrix.at<float>(i, j) /= wLen;
		}
	}

	//create projection matrix
	for (int i = 0; i < 2; i++){
		for (int j = 0; j < 4; j++){
			projection.at<float>(i, j) = homographyMatrix.at<float>(i, j);
		}
	}
	for (int i = 0; i < 4; i++){
		projection.at<float>(2, i) = 0;
	}
	for (int i = 0; i < 4; i++){
		projection.at<float>(3, i) = homographyMatrix.at<float>(2, i);
	}

	projection.at<float>(2, 3) = -1.0;


	logFile << "\n-----------------Projection matrix-------------------------\n";
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < 4; j++){
			logFile << projection.at<float>(i, j) << " ";
		}
		logFile << endl;
	}

	//calculate zRange
	double zRangeVector[NR_POINTS];
	int i = 0;
	for (auto pointKinect : puncteKinect){
		Mat xyzOmogen = Mat(4, 1, CV_32FC1);
		xyzOmogen.at<float>(0, 0) = pointKinect.x;
		xyzOmogen.at<float>(1, 0) = pointKinect.y;
		xyzOmogen.at<float>(2, 0) = pointKinect.z;
		xyzOmogen.at<float>(3, 0) = 1.0;

		cout << pointKinect.x << " " << pointKinect.y << " " << pointKinect.z << " " << endl;

		Mat z = projection*xyzOmogen;
		zRangeVector[i] = (z.at<float>(2, 0) / z.at<float>(3, 0));

		if (z.at<float>(3, 0) < 0) {
			nrNegativeWeights++;
		}

		//zRangeVector[i] = -pointKinect.z;
		cout << zRangeVector[i] << "  ";
		i++;
	}


	cout << endl;
	cout << "nrNegativeWeights = " << nrNegativeWeights << endl;

	float zMin = 9999, zMax = -9999;

	for (double z : zRangeVector){
		if (z < zMin)
			zMin = z;
		if (z > zMax)
			zMax = z;
	}

	double zRangeSize;
	//double interval
	zMin *= 2.0;
	zMax *= 0.5;
	zRangeSize = zMax - zMin;


	cout << "zRangeMin : " << zMin << "   zRangeMax : " << zMax << endl;
	cout << "zSize : " << zRangeSize << endl;

	Mat invViewport = Mat(4, 4, CV_32FC1);
	invViewport = Scalar(0.0);

	invViewport.at<float>(0, 0) = 2.0 / 640;
	invViewport.at<float>(0, 3) = -1.0;
	invViewport.at<float>(1, 1) = 2.0 / 480;
	invViewport.at<float>(1, 3) = -1.0;
	invViewport.at<float>(2, 2) = 2 / (zRangeSize);
	invViewport.at<float>(2, 3) = -2.0*zMin / (zRangeSize)-1.0;
	invViewport.at<float>(3, 3) = 1.0;


	projection = invViewport * projection;

	logFile << "\n-----------------invViewPort matrix-------------------------\n";
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < 4; j++){
			logFile << invViewport.at<float>(i, j) << "  ";
		}
		logFile << endl;
	}

	projection *= -1;

	logFile << "\n-----------------Full projection matrix-------------------------\n";
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < 4; j++){
			logFile << projection.at<float>(i, j) << " ";
		}
		logFile << endl;
	}

	logFile << "\n-----------------Projection matrix-------------------------\n";
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < 4; j++){
			logFile << projection.at<float>(i, j) << "f,  ";
		}
		logFile << endl;
	}


	ofstream file("calibration.txt");
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < 4; j++){
			file << projection.at<float>(i, j) << "\n";
		}
	}
	file.close();
	logFile.close();
}

//transform punct (X,Y,Z) in metri -> (x,y) in pixeli
Point2f * Calibration::transformPoint(Point3f kinect) {
	Point2f *proiector = new Point2f();


	Mat XYZ = Mat(4, 1, CV_64FC1);
	//Mat XYZ = Mat(3, 1, CV_32FC1);

	XYZ.at<float>(0, 0) = kinect.x;
	XYZ.at<float>(1, 0) = kinect.y;
	XYZ.at<float>(2, 0) = kinect.z;
	XYZ.at<float>(3, 0) = 1;
	Mat UV = projection * XYZ;

	//Mat UV = homographyMatrix * XYZ;

	float xPrim = UV.at<float>(0, 0) / UV.at<float>(2, 0);
	float yPrim = UV.at<float>(1, 0) / UV.at<float>(2, 0);

	cout << "\n\tWdasda = " << UV.at<float>(3, 0) << "\n";
	proiector->x = xPrim;
	proiector->y = yPrim;
	return proiector;
}

void Calibration::solveSVD(){

	ofstream logFile("FileLogMIT.txt");

	Mat A = Mat(2 * NR_POINTS, 12, CV_64FC1);
	double x, y, z;

	std::vector<Point2d> puncteProiector;
	std::vector<Point3d> puncteKinect;

	//citesc coordonatele proiectate({x,y})
	ifstream proiectorFile("coordonateProiector.txt");
	for (int i = 0; i < NR_POINTS; i++){
		proiectorFile >> x >> y;
		puncteProiector.push_back(Point2d(x, y));
	}

	//citesc coordonatele detectate(centrul cercurilor {x, y, z})
	ifstream pp("coordonateXYPuncteDetectate.txt");
	ifstream kinectFile("coordonateDetectate.txt");
	for (int i = 0; i < NR_POINTS; i++){
		kinectFile >> x >> y >> z;
		//pp >> x >> y;
		z *= -1;
		x *= 1;
		y *= 1;
		puncteKinect.push_back(Point3d(x, y, z));
	}

	pp.close();
	proiectorFile.close();
	kinectFile.close();


	//creare matrice A pentru sistem
	int index = 0;
	for (int i = 0; i < 2 * NR_POINTS; i += 2) {

		A.at<double>(i, 0) = puncteKinect.at(index).x;
		A.at<double>(i, 1) = puncteKinect.at(index).y;
		A.at<double>(i, 2) = puncteKinect.at(index).z;
		A.at<double>(i, 3) = 1;
		A.at<double>(i, 4) = 0;
		A.at<double>(i, 5) = 0;
		A.at<double>(i, 6) = 0;
		A.at<double>(i, 7) = 0;
		A.at<double>(i, 8) = -puncteProiector.at(index).x * puncteKinect.at(index).x;
		A.at<double>(i, 9) = -puncteProiector.at(index).x * puncteKinect.at(index).y;
		A.at<double>(i, 10) = -puncteProiector.at(index).x * puncteKinect.at(index).z;
		A.at<double>(i, 11) = -puncteProiector.at(index).x;

		A.at<double>(i + 1, 0) = 0;
		A.at<double>(i + 1, 1) = 0;
		A.at<double>(i + 1, 2) = 0;
		A.at<double>(i + 1, 3) = 0;
		A.at<double>(i + 1, 4) = puncteKinect.at(index).x;
		A.at<double>(i + 1, 5) = puncteKinect.at(index).y;
		A.at<double>(i + 1, 6) = puncteKinect.at(index).z;
		A.at<double>(i + 1, 7) = 1;
		A.at<double>(i + 1, 8) = -puncteProiector.at(index).y * puncteKinect.at(index).x;
		A.at<double>(i + 1, 9) = -puncteProiector.at(index).y * puncteKinect.at(index).y;
		A.at<double>(i + 1, 10) = -puncteProiector.at(index).y * puncteKinect.at(index).z;
		A.at<double>(i + 1, 11) = -puncteProiector.at(index).y;

		index++;
	}

	logFile << "\n--------------------------Matricea A----------------------------------\n";
	for (int i = 0; i < 2 * NR_POINTS; i++){
		for (int j = 0; j < 12; j++){
			logFile << A.at<double>(i, j) << " ";
		}
		logFile << endl;
		//logFile << ";";
	}

	logFile << "\n------------------------Coordonate proiector----------------------------------\n";
	for (auto pointProiector : puncteProiector){
		logFile << pointProiector.x << " " << pointProiector.y << " \n";
	}

	logFile << "\n------------------------Coordonate Kinect----------------------------------\n";
	for (auto pointKinect : puncteKinect){
		logFile << pointKinect.x << " " << pointKinect.y << "  " << pointKinect.z << " \n";
	}

	Mat solutieValori;
	Mat solutieValoriVector;
	Mat ATranspusa;

	transpose(A, ATranspusa);
	Mat eigenInput = ATranspusa * A;

	Mat eigenInputdouble;
	eigenInput.convertTo(eigenInputdouble, CV_64FC1);

	eigen(eigenInputdouble, solutieValori, solutieValoriVector);

	logFile << "\n---------------------Valori singulare---------------------------------\n";
	for (int i = 0; i < 12; i++){
		logFile << solutieValori.at<double>(i, 0) << "   ";
	}

	logFile << "\n----------------------Vectori valori singulare-----------------------------\n";
	for (int i = 0; i < 12; i++){
		for (int j = 0; j < 12; j++){
			logFile << solutieValoriVector.at<double>(i, j) << "   ";
		}
		logFile << endl << endl << endl;
	}

	logFile << "\n----------------------ProjMV----------------------------------\n";
	//create homography matrix
	//matricea de transformare(rotatie+translatie)
	vector<double> vec;
	for (int i = 0; i < 12; i++){
		vec.push_back(solutieValoriVector.at<double>(i, 11));
	}

	cv::Mat ProjMV = cv::Mat(3, 4, CV_64FC1);

	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 4; j++){
			ProjMV.at<double>(i, j) = solutieValoriVector.at<double>(11, i * 4 + j);
			logFile << ProjMV.at<double>(i, j) << " ";
		}
		logFile << endl;
	}


	//
	double zRangeVector[NR_POINTS];
	double gfFarPlane = -9999, gfNearPlane = 9999;
	int i = 0;
	for (auto pointKinect : puncteKinect){
		cout << pointKinect.x << " " << pointKinect.y << " " << pointKinect.z << " " << endl;

		zRangeVector[i] = pointKinect.z;
		cout << zRangeVector[i] << "  ";
		i++;
	}
	cout << endl;


	for (double z : zRangeVector){
		if (z < gfNearPlane)
			gfNearPlane = z;
		if (z > gfFarPlane)
			gfFarPlane = z;
	}



	//calculate scale
	double h20 = ProjMV.at<double>(2, 0);
	double h21 = ProjMV.at<double>(2, 1);
	double h22 = ProjMV.at<double>(2, 2);
	double scale = sqrt(h20*h20 + h21*h21 + h22*h22);

	cout << "\n\n\tscale = " << scale << endl;
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 4; j++){
			ProjMV.at<double>(i, j) /= scale;
		}
	}

	/*if (ProjMV.at<double>(2, 3) > 0) {

		for (int i = 0; i < 3; i++){
		for (int j = 0; j < 4; j++){
		ProjMV.at<double>(i, j) *= -1;
		}
		}
		}*/


	logFile << "\n---------------Scaled ProjMV--------------------\n";

	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 4; j++){
			logFile << ProjMV.at<double>(i, j) << " ";
		}
		logFile << endl;
	}

	Mat Q1 = Mat(3, 1, CV_64FC1);
	Mat Q2 = Mat(3, 1, CV_64FC1);
	Mat Q3 = Mat(3, 1, CV_64FC1);

	for (int i = 0; i < 3; i++) {
		Q1.at<double>(i, 0) = ProjMV.at<double>(0, i);
		Q2.at<double>(i, 0) = ProjMV.at<double>(1, i);
		Q3.at<double>(i, 0) = ProjMV.at<double>(2, i);

		cout << ProjMV.at<double>(0, i) << " " << ProjMV.at<double>(1, i) << " " << ProjMV.at<double>(2, i) << endl;
	}

	logFile << "Q1 = " << Q1 << endl;
	logFile << "Q2 = " << Q2 << endl;
	logFile << "Q3 = " << Q3 << endl;
	logFile << "-----------------------------------------------------------------\n\n";

	double q14 = ProjMV.at<double>(0, 3);
	double q24 = ProjMV.at<double>(1, 3);
	double q34 = ProjMV.at<double>(2, 3);

	double tz = q34;
	double tzeps = 1;

	if (tz > 0){
		tzeps *= -1;
	}

	tz = tzeps * q34;

	Mat r1 = Mat(3, 1, CV_64FC1);
	Mat r2 = Mat(3, 1, CV_64FC1);
	Mat r3 = Mat(3, 1, CV_64FC1);

	Mat Q1t, Q2t, Q3t;
	transpose(Q1, Q1t);
	transpose(Q2, Q2t);
	transpose(Q3, Q3t);


	r3 = tzeps * Q3t;
	Mat u0Mat = Q1t * Q3;
	Mat v0Mat = Q2t * Q3;

	/*Mat aMat = Q1.cross(Q3);
	Mat bMat = Q2.cross(Q3);*/

	Mat aMat = Q1t * Q1;
	Mat bMat = Q2t * Q2;

	logFile << "aMat = " << bMat << endl;

	double u0 = u0Mat.at<double>(0, 0);
	double v0 = v0Mat.at<double>(0, 0);

	double a = sqrt(aMat.at<double>(0, 0) - u0 * u0);
	double b = sqrt(bMat.at<double>(0, 0) - v0 * v0);

	logFile << "----------------------------u0 v0 a b------------------------\n";

	logFile << "u0 = " << u0 << "\nv0 = " << v0 << "\na = " << a << "\nb = " << b << endl;

	r1 = tzeps * ((u0 * Q3t) - Q1t) / a;
	r2 = tzeps * ((v0 * Q3t) - Q2t) / b;

	double tx = tzeps * (q14 - u0 * tz) / a;
	double ty = tzeps * (q24 - v0 * tz) / b;

	// create Rotation Matrix and Translation Vector
	Mat RotMatrix = Mat(3, 3, CV_64FC1);

	for (int i = 0; i < 3; i++) {
		RotMatrix.at<double>(0, i) = r1.at<double>(0, i);
		RotMatrix.at<double>(1, i) = r2.at<double>(0, i);
		RotMatrix.at<double>(2, i) = r3.at<double>(0, i);
	}
	logFile << "-------------------------RotMatrix------------------------------------\n";
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			logFile << RotMatrix.at<double>(i, j) << "  ";
		}
		logFile << endl;
	}


	/*Mat RT;
	transpose(RotMatrix, RT);

	Mat solutieValoriRot;
	Mat solutieValoriVectorRot;
	Mat u, vt, w;
	SVDecomp(RotMatrix, w, u, vt, 0);

	logFile << " u matrix \n" << u;
	logFile << " \nw matrix \n" << w;
	logFile << " \nvt matrix \n" << vt;

	Mat V;
	transpose(vt, V);

	Mat rotationSingular = u * V;

	logFile << "Rotation Singular Decomposition \n" << rotationSingular;*/

	Mat t = Mat(3, 1, CV_64FC1);
	t.at<double>(0, 0) = tx;
	t.at<double>(1, 0) = ty;
	t.at<double>(2, 0) = tz;
	logFile << "\n-------------------------t------------------------------------\n";
	logFile << t.at<double>(0, 0) << "  " << t.at<double>(1, 0) << "  " << t.at<double>(2, 0) << "\n";


	// Step 5: Expand found matrices to 4 x 4 matrices
	// Projection
	Mat IntMat = Mat(4, 4, CV_64FC1);
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < 4; j++){
			IntMat.at<double>(i, j) = 0.0f;
		}
	}

	IntMat.at<double>(0, 0) = -a;
	IntMat.at<double>(0, 2) = -u0;
	IntMat.at<double>(1, 1) = -b;
	IntMat.at<double>(1, 2) = -v0;
	IntMat.at<double>(2, 2) = -(gfFarPlane + gfNearPlane) / (gfFarPlane - gfNearPlane);
	IntMat.at<double>(2, 3) = -2 * gfFarPlane * gfNearPlane / (gfFarPlane - gfNearPlane);
	IntMat.at<double>(3, 2) = -1.0f;

	logFile << "\n-------------------------IntMat ProjMat------------------------------------\n";

	logFile << IntMat << endl;


	// Rotation & Translation
	Mat ExtMat = Mat(4, 4, CV_64FC1);

	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			ExtMat.at<double>(i, j) = RotMatrix.at<double>(i, j);
			//ExtMat.at<double>(i, j) = rotationSingular.at<double>(i, j);
		}
	}

	ExtMat.at<double>(0, 3) = t.at<double>(0, 0);
	ExtMat.at<double>(1, 3) = t.at<double>(1, 0);
	ExtMat.at<double>(2, 3) = t.at<double>(2, 0);
	ExtMat.at<double>(3, 3) = 1.0;


	ExtMat.at<double>(3, 0) = 0.0f;
	ExtMat.at<double>(3, 1) = 0.0f;
	ExtMat.at<double>(3, 2) = 0.0f;


	Mat viewMatTransposed;
	transpose(ExtMat, viewMatTransposed);

	double hFOV = 60.8;
	double vFOV = 45.8;

	double zfar = 100;
	double znear = 0.1; // focal length

	double width = 1024;
	double height = 768;
	double FOV = 2 * atan(1.0f * height / width);

	//double FOV = 180;
	double aspect = 4.0 / 3.0;

	cv::Mat projection = cv::Mat::zeros(4, 4, CV_64FC1);
	projection.at<double>(0, 0) = 1.0 / (aspect * tan(FOV / 2));
	projection.at<double>(1, 1) = 1.0 / tan(FOV / 2);
	projection.at<double>(2, 2) = -(zfar + znear) / (zfar - znear);
	projection.at<double>(2, 3) = -2 * (zfar * znear) / (zfar - znear);
	projection.at<double>(3, 2) = -1;

	/*double height = 768;
	double width = 1024;
	double l = -u0 / a * znear;
	double r = (width - u0) / a * znear;
	double buttom = -v0 / b * znear;
	double top = (height - v0) / b * znear;

	projection.at<double>(0, 0) = 2.0 * znear / (r - l);
	projection.at<double>(0, 2) = (r + l) / (r - l);
	projection.at<double>(1, 1) = 2.0 * znear / (top - buttom);
	projection.at<double>(1, 2) = (top + buttom) / (top - buttom);
	projection.at<double>(2, 2) = (zfar + znear) / (zfar - znear);
	projection.at<double>(2, 3) = 2 * zfar * znear / (zfar - znear);
	projection.at<double>(3, 2) = -1; */



	logFile << "projection :\n" << projection << endl;

	cv::Mat glProjMatrix;
	transpose(projection, glProjMatrix);

	Mat mvp = glProjMatrix * viewMatTransposed;



	logFile << "\n-------------------------ExtMat ModelView------------------------------------\n";
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < 4; j++){
			logFile << ExtMat.at<double>(i, j) << "f,  ";
		}
		logFile << endl;
	}

	logFile << "\n-------------------------ExtMat ModelView transposed------------------------------------\n";
	logFile << viewMatTransposed;

	logFile << "\mvp\n" << mvp;

	logFile.close();
}

//realizarea transformarii tuturor pixelilor din imagine
void Calibration::doTransformationOfImage(){
	Mat imgKinect = imread("calibration\\calibration10.png", CV_LOAD_IMAGE_UNCHANGED | CV_LOAD_IMAGE_ANYDEPTH);
	flip(imgKinect, imgKinect, 1);
	//while (true){
	//Mat imgKinect = Mat(Size(640, 480), CV_16UC1, this->aquisition.Update());
	//flip(imgKinect, imgKinect, 1);

	Mat imgProjector = Mat(Size(1024, 768), CV_16UC1, Scalar(0, 0, 0));
	Size size = imgKinect.size();
	int width = size.width;
	int height = size.height;


	ofstream f("imagine.txt");
	for (int y = 0; y < 480; y++)
	{
		for (int x = 0; x < 640; x++)
		{
			ushort z = imgKinect.at<ushort>(Point(x, y));

			Vector4 worldCoordinates = NuiTransformDepthImageToSkeleton((long)x, (long)y, z << 3, NUI_IMAGE_RESOLUTION_640x480);
			/*FLOAT fSkeletonZ = static_cast<FLOAT>(z) / 1000.0f;
			FLOAT fSkeletonX = (x - width / 2.0f) * (320.0f / width) * 3.501e-3f * fSkeletonZ;
			FLOAT fSkeletonY = -(y - height / 2.0f) * (240.0f / height) * 3.501e-3f * fSkeletonZ;
			*/

			Point3f kinect = Point3f(worldCoordinates.x, worldCoordinates.y, worldCoordinates.z);
			Point2f * projector = this->transformPoint(kinect);


			if (projector->x >= 0 && projector->x < 640 && projector->y < 480 && projector->y >= 0){
				imgProjector.at<ushort>(Point(projector->x, projector->y)) = z + z * 300;
				f << "x pixel " << x << " y pixel:" << y << " world->x: " << worldCoordinates.x * 100 << " world->y: " << worldCoordinates.y * 100 << " world->z: " << worldCoordinates.z * 100 << " projector x = " << projector->x << " prjector y = " << projector->y << endl;
				//f << "xSkeleton = " << fSkeletonX << "ySkeleton = " << fSkeletonY << "zSkeleton = " << fSkeletonZ << endl;

			}
			else{
				f << "x pixel " << x << " y pixel:" << y << " world->x: " << worldCoordinates.x * 100 << " world->y: " << worldCoordinates.y * 100 << " world->z: " << worldCoordinates.z * 100 << " projector x = " << projector->x << " prjector y = " << projector->y << endl;
			}
		}
	}

	f.close();

	for (int y = 0; y < 480; y++){
		for (int x = 0; x < 640; x++){
			imgKinect.at<ushort>(Point(x, y)) = imgKinect.at<ushort>(Point(x, y)) + imgKinect.at<ushort>(Point(x, y)) * 300;
		}
	}


	//cvNamedWindow("proiector", CV_WINDOW_NORMAL );
	//cvSetWindowProperty("proiector", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

	//Mat im_color;
	//applyColorMap(imgProjector, im_color, COLORMAP_JET);

	//flip(imgProjector, imgProjector, 1);
	imshow("proiector", imgProjector);
	imshow("kinect", imgKinect);
	waitKey(0);

	//}
}

void Calibration::solvePnP(){

	Mat A = Mat(2 * NR_POINTS, 9, CV_64FC1);
	float x, y, z;

	std::vector<Point2d> puncteProiector;
	std::vector<Point3d> puncteKinect;

	//citesc coordonatele proiectate({x,y})
	ifstream proiectorFile("coordonateProiector.txt");
	for (int i = 0; i < NR_POINTS; i++){
		proiectorFile >> x >> y;
		puncteProiector.push_back(Point2d(x, y));
	}

	//citesc coordonatele detectate(centrul cercurilor {x, y, z})
	ifstream kinectFile("coordonateDetectate.txt");
	for (int i = 0; i < NR_POINTS; i++){
		kinectFile >> x >> y >> z;
		puncteKinect.push_back(Point3d(x, y, z));
	}

	proiectorFile.close();
	kinectFile.close();

	int convertedCols = 1024;
	int convertedRows = 768;


	double FOV = 2 * atan(1.0f * convertedRows / convertedCols);


	int focal_length = convertedCols; // Approximate focal length.
	//int focal_length = (int) (1.0f *convertedRows/ 2 / tan(FOV * 3.14/180) / 2);

	Point center(convertedCols / 2, convertedRows / 2);

	Mat camera_matrix = Mat::zeros(3, 3, CV_64FC1);

	camera_matrix.at<double>(0, 0) = focal_length;
	camera_matrix.at<double>(0, 2) = center.x;
	camera_matrix.at<double>(1, 1) = focal_length;
	camera_matrix.at<double>(1, 2) = center.y;
	camera_matrix.at<double>(2, 2) = 1;

	std::cout << "Initial cameraMatrix: " << camera_matrix << std::endl;

	cv::Mat distCoeffs(5, 1, cv::DataType<double>::type);
	distCoeffs.at<double>(0) = 0;
	distCoeffs.at<double>(1) = 0;
	distCoeffs.at<double>(2) = 0;
	distCoeffs.at<double>(3) = 0;
	distCoeffs.at<double>(4) = 0;


	cv::Mat rvec(3, 1, cv::DataType<double>::type);
	cv::Mat tvec(3, 1, cv::DataType<double>::type);

	cv::solvePnP(puncteKinect, puncteProiector, camera_matrix, distCoeffs, rvec, tvec);


	ofstream file("solvepnp.txt");

	Mat rotation = Mat::zeros(3, 3, CV_64FC1);
	Rodrigues(rvec, rotation);



	file << rvec << endl << tvec << endl;

	file << rotation << endl;

	Mat viewMatrix = Mat::zeros(4, 4, CV_64FC1);
	double arr;
	for (int row = 0; row < 3; ++row) {
		int setRow = row;
		/*if (row == 0) setRow = 1;
			else  if (row == 1) setRow = 0;*/

		for (int col = 0; col < 3; ++col) {
			arr = rotation.at<double>(row, col);
			if (setRow == 1 || setRow == 2 || setRow == 0) arr *= -1;

			viewMatrix.at<double>(setRow, col) = arr;
		}
		arr = tvec.at<double>(row, 0);

		if (setRow == 1 || setRow == 2 || setRow == 0) arr *= -1;

		viewMatrix.at<double>(setRow, 3) = arr;
	}

	viewMatrix.at<double>(3, 3) = 1;
	Mat glViewMatrix;
	transpose(viewMatrix, glViewMatrix);

	file << glViewMatrix << endl;

	//double FOV = 2 * atan(1.0f * 480 / 640);

	double hFOV = 60.8;
	double vFOV = 45.8;

	double zfar = 1000;
	double znear = 0.01;

	cv::Mat projection = cv::Mat::zeros(4, 4, CV_64FC1);
	projection.at<double>(0, 0) = 1.0 / (640.0 / 480.0 * tan(FOV / 2));
	projection.at<double>(1, 1) = 1.0 / tan(FOV / 2);
	projection.at<double>(2, 2) = (zfar + znear) / (zfar - znear);
	projection.at<double>(2, 3) = 2 * (zfar * znear) / (zfar - znear);
	projection.at<double>(3, 2) = -1;

	file << projection << endl;

	cv::Mat glProjMatrix;
	transpose(projection, glProjMatrix);

	Mat mvp = viewMatrix;

	file << mvp << endl;


	file.close();
}

void Calibration::loadCalibration(){	
}
