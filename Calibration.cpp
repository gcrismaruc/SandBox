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
const int Calibration::NR_POINTS = 12;
int Calibration::THRESH_MAX = 1000;
int Calibration::THRESH_MIN = 900;
//int Calibration::THRESH_MAX = 805;
//int Calibration::THRESH_MIN = 725;
double Calibration::zFar = 0.0;
double Calibration::zNear = 0.0;

Mat Calibration::thresh = Mat(Size(640, 480), CV_8UC1);
int Calibration::lowThreshold = 100;

CvMat *Calibration::solutieQ = cvCreateMat(12, 1, CV_64FC1);
Mat Calibration::homographyMatrix =  Mat(3, 4, CV_32FC1);
//Mat Calibration::homographyMatrix = Mat(3, 3, CV_32FC1);
Mat Calibration::projection = Mat(4, 4, CV_32FC1);


double Calibration::FX = 5.4735210296154094e+002;
double Calibration::CX = 3.1075353754346150e+002;
double Calibration::FY = 5.4392478916448886e+002;
double Calibration::CY = 2.6349428097343048e+002;

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

//adaug punctele pentru proiectie
void Calibration::createAxis(int x, int y, Mat *image) {
	line(*image, Point(x - 10, y), Point(x + 10, y), Scalar((unsigned short)-1), 1, 8);
	line(*image, Point(x, y - 10), Point(x, y + 10), Scalar((unsigned short)-1), 1, 8);
}

//afisez imaginea cu axele si salvez imaginea de la Kinect cand apas tasta 1
//imaginea este salvata dar totodata detectez cercul si centrul lui
int Calibration::showAxes(USHORT * imageArray) {

	Mat depthImage = Mat(Size(640, 480), CV_16UC1, imageArray);
	Mat projectedImage = Mat(Size(640, 480), CV_16UC1, Scalar(0, 0, 0));
	this->createAxis(vecIntersectionPoints[nrFrame].first, vecIntersectionPoints[nrFrame].second, &projectedImage);


	int newWidth = abs(x2 - x1);
	int newHeight = abs(y2 - y1);
	
	cvNamedWindow("axis", CV_WINDOW_NORMAL );
	cvSetWindowProperty("axis", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

	imshow("axis", projectedImage);
	int pressedKey =  waitKey(10);

	int checkDetectedCircle = -1;

	if(pressedKey == 49){
		Mat depthImageForCircle = Mat(Size(640, 480), CV_8UC1);
		depthImage.clone().convertTo(depthImageForCircle, CV_8UC1, 1.0 / 255.0);

		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(100);
		const string imageName = "calibration\\calibration" + to_string(nrFrame) + ".png";
		
		cout<<imageName<<endl;
		imwrite(imageName, depthImage, compression_params);

		checkDetectedCircle = detectCircleForImage(depthImageForCircle, depthImage);
		
		if (checkDetectedCircle == 0) {
			nrFrame++;
		}
	}

	return (checkDetectedCircle == -1) ? checkDetectedCircle : pressedKey;
}

//achizitionez frame-uri de la Kinect si le transmit la functia de mai sus
void Calibration::getFrames(){
	
	ofstream xyp("coordonateXYPuncteDetectate.txt");
	int a = 0;
	int count = 0;

	while(count != 70) {
		cout<<"aici\n";
		while(a != 49){
			a = this->showAxes(this->aquisition.Update());
		}
		count++;
		a = 0;
	}

	cout<<"End of getFrames\n";
	cout << "Size of vector : " << vecOfDetectedCoordinates.size()<<endl;
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

//detectez cercul si salvez coordonatele in fisier
int Calibration::detectCircleForImage(Mat eightBitsImage, Mat sixteenBitsImage) {

	int retValue = 0;
		Mat src_gray;
		thresh = eightBitsImage.clone();

	    int nrPixels = 0;
		int nr_x = 0;
		int nr_y = 0;

		Mat labels,stats,centroids;
		int noComp;
		
		for (int y = 0; y < 480; y++) {
			for(int x = 0; x < 640; x++) {	
				if(!(sixteenBitsImage.at<ushort>(Point(x, y)) > THRESH_MIN && sixteenBitsImage.at<ushort>(Point(x, y)) < THRESH_MAX))
				{
					thresh.at<uchar>(Point(x, y)) = 0;
				} else {
					thresh.at<uchar>(Point(x, y)) = 255;					
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
		if (circles.size() == 1){
			//for (size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
			size_t current_circle  = 0;
			cv::Point center(std::round(circles[current_circle][0]), std::round(circles[current_circle][1]));
			int radius = std::round(circles[current_circle][2]);

			x = circles[current_circle][0];
			y = circles[current_circle][1];
			z = (int)sixteenBitsImage.at<ushort>(Point(x, y));

			cv::circle(thresh, center, radius, cv::Scalar(122, 125, 22), 5);
			cv::circle(thresh, center, radius, Scalar(0, 33, 255), 3, 8, 0);

			//}
		}


		imshow("teststs", thresh);
		waitKey(10);
		
		//Get world coordiantes for center
		Vector4 worldCoordinates =  NuiTransformDepthImageToSkeleton((long)x, (long)y, z<<3, NUI_IMAGE_RESOLUTION_640x480);
		
		//coordinates in meters
		cout << x<< " " <<y << "real coordinates : "<<(worldCoordinates.x * 1000) << " " << (worldCoordinates.y * 1000) << " " << (worldCoordinates.z * 1000) << endl;

		if (x == 48 || y == 48 || worldCoordinates.x == 0 || worldCoordinates.y == 0){
			retValue = -1;
		}

		//save coordinate in milimeters
		if (retValue == 0){
			vecOfDetectedCoordinates.push_back(Point3f((worldCoordinates.x ), (worldCoordinates.y ), (worldCoordinates.z )));
			vecOfDetectedPixelCoordinates.push_back(Point2f(x, y));
		}
		return retValue;
}

void Calibration::showCircles() {

	ofstream file("coordonateDetectate.txt"); 

	int nrFrame = 0;

	while(nrFrame != 36){
		const string imageName = "calibration\\calibration" + to_string(nrFrame) + ".png";;
		
		Mat src = imread(imageName, CV_LOAD_IMAGE_UNCHANGED | CV_LOAD_IMAGE_ANYDEPTH);
		Mat src_gray;
		Mat tresh = src.clone();
	
		cout<<"\n\t nr.channels = "<<src.channels();
		
	     int nrPixels = 0;
		 int nr_x = 0;
		 int nr_y = 0;
		for (int y = 0; y < 480; y++){
			for(int x = 0; x < 640; x++)
			{	

				if(src.at<ushort>(Point(x, y)) != 0 && !(src.at<ushort>(Point(x, y)) > 1090 && src.at<ushort>(Point(x, y)) < 1230))
					{
						nrPixels ++;
						nr_x += x;
						nr_y += y;

						src.at<ushort>(Point(x, y)) = 54000;
						
				} else {
					src.at<ushort>(Point(x, y)) = 0;					
				}
			}
			cout<<endl;
		}
		imshow("tresh", src);

	
		int x = nr_x/nrPixels;
		int y = nr_y/nrPixels;
		int z = (int)tresh.at<ushort>(Point(x, y));
		Point center(x, y);
		src.at<ushort>(Point(x, y)) = 0;
		file<<x << " " << y<<" "<<z<<"\n";
		imshow("test", src);
		waitKey(0);
	nrFrame++;
	}

	file.close();

}

//citesc coordonatele proiectate
void Calibration::readConfig() {

	ifstream file("coordonateProiector.txt");

	while(!file.eof()){
		unsigned int x, y;
		file>>x>>y;
		vecIntersectionPoints.push_back(make_pair(x, y));
	}

	file.close();
}

void Calibration::sendImageAtClick(Mat* depthImage) {

		Mat depthImageForCircle = Mat(Size(640, 480), CV_8UC1);
		depthImage->convertTo(depthImageForCircle, CV_8UC1, 1.0/255.0);
		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(9);
		const string imageName = "calibration\\calibration" + to_string(nrFrame) + ".png";
		
		cout<<imageName<<endl;
		imshow("8bitsTest", depthImageForCircle);
		//imwrite(imageName, depthImageForCircle, compression_params);
		detectCircleForImage(depthImageForCircle, *depthImage);
		nrFrame++;
}

void Calibration::CallBackFunc(int event, int x, int y, int flags, void* userdata){
	if ( flags == (EVENT_FLAG_LBUTTON) )
     {
		  nrClick++;
		  if(nrClick == 1){
			  x1 = x;
			  y1 = y;
		  } else {
			  x2 = x;
			  y2 = y;
		  }
          cout << "Left mouse button is clicked while pressing CTRL key - position (" << x << ", " << y << ")" << endl;
     }
}

void Calibration::getMouseCoordinates(){
	
	ofstream f("roi.txt");
	while(nrClick != 2) {
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

		/*if(nrClick == 1){
			cout<<"x1 = "<<x1<<" y1 = "<<y1<<" z = "<<img.at<ushort>(Point(x1, y1))<<endl; 
		}

		if(nrClick == 2){
			cout<<"x2 = "<<x2<<" y2 = "<<y2<<" z = "<<img.at<ushort>(x2, y2)<<endl;
		}*/
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

	for(int y = 0; y < height; y++)
	{
		for(int x = 0; x < width; x++)
		{
			if(x < x1  || y < y1 || y > y2 || x > x2 )
			{
				img.at<ushort>(Point(x, y)) = 0;
			}
		}
	}
}

//salvez coordonatele pentru proiector in fisier, dupa ce la generez random
void Calibration::savePointsForAxes(){
	
	ofstream g("coordonateProiector.txt");
	ofstream f("coordinates.txt");
	

	random_device rd;     // only used once to initialise (seed) engine
	mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
	uniform_int_distribution<int> x_rnd(30, x2-70);
	uniform_int_distribution<int> y_rnd(30, y2-70);


	for (int i = 0; i < 70; i++){
		int x = x_rnd(rng);
		int y = y_rnd(rng);

		f << x << " " << y << endl;
		g << x << " " << y << endl;
		cout << x << " " << y << endl;
	}

	f.close();
	g.close();
}

void Calibration::detectCircle(){
	
	Mat image = imread("calibration\\calibration1.png", 0);
	Mat dst;

	INuiCoordinateMapper *mapper;

	
	
	this->aquisition.m_pNuiSensor->NuiGetCoordinateMapper(&mapper);

	
	cvNamedWindow("circle", CV_WINDOW_NORMAL );
	//setMouseCallback("circle", CallBackFunc, NULL);
	for (int y = 0; y < 480; y++)
		for(int x = 1; x < 640; x++)
			image.at<ushort>(y,x) +=5000;
	
	imshow("circle", image);
	waitKey(10);

	/*for(int i = 1; i < 10; i++){

		threshold(image, dst, p[0], 255, THRESH_BINARY);
	
		imshow("dddd", dst);
		waitKey(0);
		ostringstream imageName;
		imageName<<"calibration\\calibration" + to_string(i) << ".jpg";
		image = imread(imageName.str().c_str());
		cvtColor( image, image, CV_BGR2GRAY );
		imshow("dddd", image);
		waitKey(0);
	}*/

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
			homographyMatrix.at<float>(i, j) = solutieValoriVector.at<float>(11, i*4+j);
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
		projection.at<float>(3, i) = homographyMatrix.at<float>(2,i);
	}

	projection.at<float>(2, 3) = -1.0;
	

	logFile << "\n-----------------Projection matrix-------------------------\n";
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < 4; j++){
			logFile << projection.at<float>(i, j) << "  ";
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
		//zRangeVector[i] = -pointKinect.z;
		cout << zRangeVector[i] << "  ";
		i++;
	}
	cout << endl;

	float zMin = 9999, zMax=-9999;

	for (double z : zRangeVector){
		if (z < zMin)
			zMin = z;
		if (z > zMax)
			zMax = z;
	}

	//cv::Range zRange = cv::Range(zMin, zMax);

	//cout << "zRangeMin : " << zRange.start << "   zRangeMax : " << zRange.end << endl;
	//cout << "zSize : " << zRange.size() << endl;

	double zRangeSize;
	//double interval
	zMin *= 2.0;
	zMax *= 0.5;
	zRangeSize = zMax - zMin;


	cout << "zRangeMin : " << zMin << "   zRangeMax : " << zMax << endl;
	cout << "zSize : " << zRangeSize << endl;

	Mat invViewport = Mat(4, 4, CV_32FC1);
	invViewport = Scalar(0.0);

	invViewport.at<float>(0, 0) = 2.0 / 1024;
	invViewport.at<float>(0, 3) = -1.0;
	invViewport.at<float>(1, 1) = 2.0 / 768;
	invViewport.at<float>(1, 3) = -1.0;
	invViewport.at<float>(2, 2) = 2.0 / (zRangeSize);
	invViewport.at<float>(2, 3) = -2.0*zMin / (zRangeSize) - 1.0;
	invViewport.at<float>(3, 3) = 1.0;

	/*invViewport.at<float>(0, 0) = 2.0 * zMin / 640;
	invViewport.at<float>(0, 2) = 1;
	invViewport.at<float>(1, 1) = 2.0 * zMin / 480;
	invViewport.at<float>(1, 2) = 1;
	invViewport.at<float>(2, 2) = 2.0 / (zRangeSize);
	invViewport.at<float>(2, 3) = -2.0*zMin / (zRangeSize)-1.0;
	invViewport.at<float>(3, 2) = -1.0;*/

	//projection = invViewport*projection;

	logFile << "\n-----------------invViewPort matrix-------------------------\n";
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < 4; j++){
			logFile << invViewport.at<float>(i, j) << "  ";
		}
		logFile << endl;
	}

	logFile << "\n-----------------Full projection matrix-------------------------\n";
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < 4; j++){
			logFile << projection.at<float>(i, j) << "  ";
		}
		logFile << endl;
	}


	//// Decompose the projection matrix into:
	//cv::Mat K(3, 3, cv::DataType<float>::type); // intrinsic parameter matrix
	//cv::Mat R(3, 3, cv::DataType<float>::type); // rotation matrix
	//cv::Mat T(4, 1, cv::DataType<float>::type); // translation vector

	//cv::decomposeProjectionMatrix(homographyMatrix, K, R, T);


	/*vector<Mat> RS;
	vector<Mat> TS;
	vector<Mat> NS;

	float fx = 368.096588;
	float fy = 368.096588;

	float cx = 261.696594;
	float cy = 202.522202;

	K = Scalar(0.0);
	K.at<float>(0, 0) = fx;
	K.at<float>(1, 1) = fy;
	K.at<float>(2, 2) = 1;
	K.at<float>(0, 2) = 640.0 / 2;
	K.at<float>(1, 2);

	Mat homography3 = Mat(3, 3, CV_32FC1);

	homography3.at<float>(0, 0) = homographyMatrix.at<float>(0, 0);
	homography3.at<float>(0, 1) = homographyMatrix.at<float>(0, 1);
	homography3.at<float>(0, 2) = homographyMatrix.at<float>(0, 3);

	homography3.at<float>(1, 0) = homographyMatrix.at<float>(1, 0);
	homography3.at<float>(1, 1) = homographyMatrix.at<float>(1, 1);
	homography3.at<float>(1, 2) = homographyMatrix.at<float>(1, 3);

	homography3.at<float>(2, 0) = homographyMatrix.at<float>(2, 0);
	homography3.at<float>(2, 1) = homographyMatrix.at<float>(2, 1);
	homography3.at<float>(2, 2) = homographyMatrix.at<float>(2, 3);

		
	cv::decomposeHomographyMat(homography3, K, RS, TS, NS);


	Mat R = RS.at(0);
	Mat T = TS.at(0);

	logFile << "\n-----------------Intrinsic parameter matrix-------------------------\n";
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			logFile << K.at<float>(i, j) << "  ";
		}
		logFile << endl;
	}

	logFile << "\n-----------------Rotation matrix-------------------------\n";
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			logFile << R.at<float>(i, j) << "  ";
		}
		logFile << endl;
	}

	logFile << "\n-----------------Translation vector-------------------------\n";
	for (int i = 0; i < 3; i++){
		logFile << T.at<float>(i, 0) << " ";
	}

	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			projection.at<float>(i,j) = R.at<float>(i, j);
		}
	}

	for (int i = 0; i < 3; i++){
		projection.at<float>(i, 3) = T.at<float>(i, 0);
	}

	for (int i = 0; i < 3; i++){
		projection.at<float>(3, i) = 0.0;
	}

	projection.at<float>(3, 3) = 1.0;
	*/

	logFile << "\n-----------------Projection matrix-------------------------\n";
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < 4; j++){
			logFile << projection.at<float>(i, j) << "  ";
		}
		logFile << endl;
	}
	

	logFile.close();
}

void Calibration::decomposeHomography(){
	double fx, fy, cx, cy;

	fx = 368.096588;
	fy = 368.096588;

	cx = 261.696594;
	cy = 202.522202;
	Mat R(3, 3, CV_32FC1);
	Mat T(3, 1, CV_32FC1);
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 4; j++){
			homographyMatrix.at<float>(i, j) = homographyMatrix.at<float>(i, j) / homographyMatrix.at<float>(2, 3);
		}
	}


	R.at<float>(0, 0) = (homographyMatrix.at<float>(0, 0) - cx*homographyMatrix.at<float>(2, 0)) / fx;
	R.at<float>(0, 1) = (homographyMatrix.at<float>(0, 1) - cx*homographyMatrix.at<float>(2, 1)) / fx;
	R.at<float>(0, 2) = (homographyMatrix.at<float>(0, 2) - cx*homographyMatrix.at<float>(2, 2)) / fx;

	R.at<float>(1, 0) = (homographyMatrix.at<float>(1, 0) - cy*homographyMatrix.at<float>(2, 0)) / fy;
	R.at<float>(1, 1) = (homographyMatrix.at<float>(1, 1) - cy*homographyMatrix.at<float>(2, 1)) / fy;
	R.at<float>(1, 2) = (homographyMatrix.at<float>(1, 2) - cy*homographyMatrix.at<float>(2, 2)) / fy;

	R.at<float>(2, 0) = homographyMatrix.at<float>(2, 0);
	R.at<float>(2, 1) = homographyMatrix.at<float>(2, 1);
	R.at<float>(2, 1) = homographyMatrix.at<float>(2, 2);

	T.at<float>(0, 0) = (homographyMatrix.at<float>(0, 3) - cx*homographyMatrix.at<float>(2, 3)) / fx;
	T.at<float>(1, 0) = (homographyMatrix.at<float>(1, 3) - cy*homographyMatrix.at<float>(2, 3)) / fy;
	T.at<float>(2, 0) = homographyMatrix.at<float>(2, 3);

	std::ofstream logFile;
	logFile.open("FileLog.txt", std::ofstream::out | std::ofstream::app);


	logFile << "\n-----------------Rotation matrix-------------------------\n";
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			logFile << R.at<float>(i, j) << "  ";
		}
		logFile << endl;
	}

	logFile << "\n-----------------Translation vector-------------------------\n";
	for (int i = 0; i < 3; i++){
		logFile << T.at<float>(i, 0) << " ";
	}

	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			projection.at<float>(i, j) = R.at<float>(i, j);
		}
	}

	for (int i = 0; i < 3; i++){
		projection.at<float>(i, 3) = T.at<float>(i, 0);
	}

	for (int i = 0; i < 3; i++){
		projection.at<float>(3, i) = 0.0;
	}

	projection.at<float>(3, 3) = 1.0;

	logFile << "\n-----------------Projection matrix-------------------------\n";
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < 4; j++){
			logFile << projection.at<float>(i, j) << "  ";
		}
		logFile << endl;
	}
	
	logFile.close();
}

//transform punct (X,Y,Z) in metri -> (x,y) in pixeli
Point2f * Calibration::transformPoint(Point3f kinect) {
	Point2f *proiector = new Point2f();

	
	Mat XYZ = Mat(4, 1, CV_32FC1);
	//Mat XYZ = Mat(3, 1, CV_32FC1);

	XYZ.at<float>(0, 0) = kinect.x;
	XYZ.at<float>(1, 0) = kinect.y;
	XYZ.at<float>(2, 0) = kinect.z;
	XYZ.at<float>(3, 0) = 1;
	Mat UV = projection * XYZ;

	//Mat UV = homographyMatrix * XYZ;

	int xPrim = round(UV.at<float>(0, 0) / UV.at<float>(3, 0));
	int yPrim = round(UV.at<float>(1, 0) / UV.at<float>(3, 0));

	
	proiector->x = xPrim;
	proiector->y = yPrim;
	return proiector;
}

void Calibration::solveSVD(){



}

//realizarea transformarii tuturor pixelilor din imagine
void Calibration::doTransformationOfImage(){
	Mat imgKinect = imread("calibration\\calibration10.png", CV_LOAD_IMAGE_UNCHANGED | CV_LOAD_IMAGE_ANYDEPTH);
	flip(imgKinect, imgKinect,1);
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

				Vector4 worldCoordinates = NuiTransformDepthImageToSkeleton((long)x, (long)y, z<<3, NUI_IMAGE_RESOLUTION_640x480);
				Point3f kinect = Point3f(x, y, -worldCoordinates.z);
				Point2f * projector = this->transformPoint(kinect);

				
				if (projector->x >= 0 && projector->x < 640 && projector->y < 480 && projector->y >= 0){
					imgProjector.at<ushort>(Point(projector->x, projector->y)) = z + z * 300;
					f << "x pixel " << x << " y pixel:" << y << " world->x: " << worldCoordinates.x << " world->y: " << worldCoordinates.y << " world->z: " << worldCoordinates.z << " projector x = " << projector->x << " prjector y = " << projector->y << endl;
				}else{
					f << "x pixel " << x << " y pixel:" << y << " world->x: " << worldCoordinates.x << " world->y: " << worldCoordinates.y << " world->z: " << worldCoordinates.z << " projector x = " << projector->x << " prjector y = " << projector->y << endl;
				}
			}
		}

		f.close();

		for (int y = 0; y < 480; y++){
			for (int x = 0; x < 640; x++){
				imgKinect.at<ushort>(Point(x, y)) = imgKinect.at<ushort>(Point(x, y)) + imgKinect.at<ushort>(Point(x, y))*300;
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

	Mat A = Mat(2*NR_POINTS, 9, CV_64FC1);
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


	//creare matrice A pentru sistem
	int index = 0;
	for (int i = 0; i < 2 * NR_POINTS; i += 2) {

		A.at<double>(i , 0) = puncteKinect.at(index).x;
		A.at<double>(i , 1) = puncteKinect.at(index).y;
		//A.at<double>(i , 2) = puncteKinect.at(index).z;
		A.at<double>(i , 2) = 1;
		A.at<double>(i , 3) = 0;
		A.at<double>(i , 4) = 0;
		A.at<double>(i , 5) = 0;
		//A.at<double>(i , 7) = 0;
		A.at<double>(i , 6) = -puncteProiector.at(index).x * puncteKinect.at(index).x;
		A.at<double>(i , 7) = -puncteProiector.at(index).x * puncteKinect.at(index).y;
		//A.at<double>(i , 8) = -puncteProiector.at(index).x * puncteKinect.at(index).z;
		A.at<double>(i , 8) = -puncteProiector.at(index).x;

		A.at<double>(i + 1, 0) = 0;
		A.at<double>(i + 1, 1) = 0;
		A.at<double>(i + 1, 2) = 0;
		//A.at<double>(i + 1, 3) = 0;
		A.at<double>(i + 1, 3) = puncteKinect.at(index).x;
		A.at<double>(i + 1, 4) = puncteKinect.at(index).y;
		//A.at<double>(i + 1, 6) = puncteKinect.at(index).z;
		A.at<double>(i + 1, 5) = 1;
		A.at<double>(i + 1, 6) = -puncteProiector.at(index).y * puncteKinect.at(index).x;
		A.at<double>(i + 1, 7) = -puncteProiector.at(index).y * puncteKinect.at(index).y;
		//A.at<double>(i + 1, ) = -puncteProiector.at(index).y * puncteKinect.at(index).z;
		A.at<double>(i + 1, 8) = -puncteProiector.at(index).y;
		
		index++;
	}

	Mat solutieValori;
	Mat solutieValoriVector;
	Mat AT;

	transpose(A, AT);
	Mat eigenInput = AT * A;

	Mat eigenInputFloat;

	eigenInput.convertTo(eigenInputFloat, CV_32FC1);

	cv::eigen(eigenInputFloat, solutieValori, solutieValoriVector);


	for (int j = 0; j < 9; j++){
		for (int i = 0; i < 9; i++){
			cout << solutieValoriVector.at<float>(i, j) << "   ";
			//cvmSet(solutieQ, i, 0, solutieValoriVector.at<float>(i, 11));
		}
		cout << endl << endl << endl;
	}

	ofstream fis("homography.txt");
	cout << "Homography----------------------------------\n";
	//create homography matrix
	//matricea de transformare(rotatie+translatie)
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			homographyMatrix.at<float>(i, j) = solutieValoriVector.at<float>( i * 3 + j, 8);
			fis << homographyMatrix.at<float>(i, j) << " ";
		}
		fis << endl;
	}

	fis.close();
	/*float q11, q12, q13, q14, q21, q22, q23, q24, q31, q32, q33, q34;

	q13 = homographyMatrix.at<float>(2, 0);
	q23 = homographyMatrix.at<float>(2, 1);
	q33 = homographyMatrix.at<float>(2, 2);

	double scale = sqrt(q13*q13 + q23*q23 + q33*q33);//??SumSquare???

	homographyMatrix /= scale;
	
	q14 = homographyMatrix.at<float>(0, 3);
	q24 = homographyMatrix.at<float>(1, 3);
	q34 = homographyMatrix.at<float>(2, 3);

	double tz = q34;
	double tzeps = 1;
	if (tz > 0) tzeps = -1;
	tz = tzeps * q34;

	//vectori coloana
	Mat q1;
	Mat q2;
	Mat q3;


	q1.push_back(homographyMatrix.at<float>(0, 0));
	q1.push_back(homographyMatrix.at<float>(0, 1));
	q1.push_back(homographyMatrix.at<float>(0, 2));

	q2.push_back(homographyMatrix.at<float>(1, 0));
	q2.push_back(homographyMatrix.at<float>(1, 1));
	q2.push_back(homographyMatrix.at<float>(1, 2));

	q3.push_back(homographyMatrix.at<float>(2, 0));
	q3.push_back(homographyMatrix.at<float>(2, 1));
	q3.push_back(homographyMatrix.at<float>(2, 2));

	Mat q1t;
	Mat q2t;
	Mat q3t;
	transpose(q1, q1t);
	transpose(q2, q2t);
	transpose(q3, q3t);

	Mat r1, r2, r3;

	Mat m1 = q1t * q3;
	Mat m2 = q2t * q3;

	float u0 = m1.at<float>(0, 0);
	float v0 = m2.at<float>(0, 0);

	double a = norm(q1) * norm(q3);
	double b = norm(q2) * norm(q3);


	r3 = tzeps * q3;
	r1 = tzeps * (q1 - (u0 * q3)) / a;
	r2 = tzeps * (q2 - (v0 * q3)) / b;

	double tx = tzeps * (q14 - u0 * tz) / a;
	double ty = tzeps * (q24 - v0 * tz) / b;

	Mat t;
	t.push_back(tx);
	t.push_back(ty);
	t.push_back(tz);

	
	//Mat proj = Mat(4, 4, CV_32F);

	/*projection.at<float>(0, 0) = r1.at<float>(0, 0);
	projection.at<float>(1, 1) = r1.at<float>(0, 1);
	projection.at<float>(2, 2) = r1.at<float>(0, 2);
	projection.at<float>(3, 3) = 0.0;

	projection.at<float>(0, 1) = r2.at<float>(0, 0);
	projection.at<float>(1, 1) = r2.at<float>(0, 1);
	projection.at<float>(2, 1) = r2.at<float>(0, 2);
	projection.at<float>(3, 1) = 0.0;

	projection.at<float>(0, 2) = r3.at<float>(0, 0);
	projection.at<float>(1, 2) = r3.at<float>(0, 1);
	projection.at<float>(2, 2) = r3.at<float>(0, 2);
	projection.at<float>(3, 2) = 0.0;

	projection.at<float>(0, 0) = r1.at<float>(0, 0);
	projection.at<float>(1, 1) = r1.at<float>(1, 0);
	projection.at<float>(2, 2) = r1.at<float>(2, 0);
	projection.at<float>(3, 3) = 0.0;

	projection.at<float>(0, 1) = r2.at<float>(0, 0);
	projection.at<float>(1, 1) = r2.at<float>(1, 0);
	projection.at<float>(2, 1) = r2.at<float>(2, 0);
	projection.at<float>(3, 1) = 0.0;

	projection.at<float>(0, 2) = r3.at<float>(0, 0);
	projection.at<float>(1, 2) = r3.at<float>(1, 0);
	projection.at<float>(2, 2) = r3.at<float>(2, 0);
	projection.at<float>(3, 2) = 0.0;

	projection.at<float>(0, 3) = tx;
	projection.at<float>(1, 3) = ty;
	projection.at<float>(2, 3) = tz;
	projection.at<float>(3, 3) = 1.0;

	*/
	//cout << endl << endl << "Projection\n\n";
	/*for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++){
			cout << projection.at<float>(i, j) << "   ";
		}

		cout << endl;
	}*/
}

void Calibration::loadCalibration(){
	std::vector<Point2f> puncteProiector;
	std::vector<Point3f> puncteKinect;
	float x, y, z;
	//citesc coordonatele proiectate({x,y})
	ifstream proiectorFile("coordonateProiector.txt");
	for (int i = 0; i < NR_POINTS; i++){
		proiectorFile >> x >> y;
		puncteProiector.push_back(Point2f(x, y));
	}

	//citesc coordonatele detectate(centrul cercurilor {x, y, z})
	ifstream kinectFile("coordonateDetectate.txt");
	ifstream pp("coordonateXYPuncteDetectate.txt");
	for (int i = 0; i < NR_POINTS; i++){
		kinectFile >> x >> y >> z;
		pp >> x >> y;
		puncteKinect.push_back(Point3f(x, y, z));
	}

	pp.close();
	proiectorFile.close();
	kinectFile.close();

	vector<pair<Point3f, pair<Point2f, Point2f>>> perechi;
	int index=0;
	for (auto point : puncteKinect){
		Point2f * pointProjector = this->transformPoint(point);

		perechi.push_back(make_pair(point, make_pair(puncteProiector.at(index), *pointProjector)));

		index++;
	}

	ofstream file("perechi.txt");

	for (auto pereche : perechi){
		file << pereche.first.x << setw(20) << pereche.first.y << setw(20) << pereche.first.z << "  ; " << pereche.second.first.x << setw(20)
			<< pereche.second.first.y<<"      "<<pereche.second.second.x<< setw(20)<<pereche.second.second.y << endl;
	}
	file.close();
}

void Calibration::drawGrid(Mat image){
	Mat mat_img(image);
	int stepSize = 165;
	ofstream g("coordonateProiector.txt");

	int width = mat_img.size().width;
	int height = mat_img.size().height;

	for (int i = 0; i<height; i += stepSize){

		cv::line(mat_img, Point(0, i), Point(width, i), cv::Scalar(255, 255, 255));
	}
	for (int i = 0; i<width; i += stepSize){
		cv::line(mat_img, Point(i, 0), Point(i, height), cv::Scalar(255, 255, 255));
	}
	imshow("Grid", mat_img);
	waitKey(0);
}