#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "Calibration.h"
#include <strsafe.h>
#include <iostream>
#include <string>
#include <fstream>
#include <math.h>
#include <NuiSensor.h>
#include <NuiImageCamera.h>
#include <NuiApi.h>
#include <NuiSkeleton.h>



int Calibration::nrFrame = 0;
int Calibration::nrClick = 0;
int Calibration::x1 = 144;
int Calibration::y1 = 2;
int Calibration::x2 = 584;
int Calibration::y2 = 247;
int Calibration::THRESH_MAX = 1111;
int Calibration::THRESH_MIN = 1000;

Mat Calibration::thresh = Mat(Size(640, 480), CV_8UC1);
int Calibration::lowThreshold = 100;

CvMat *Calibration::solutieQ = cvCreateMat(12, 1, CV_64FC1);
Mat Calibration::homographyMatrix =  Mat(3, 4, CV_32FC1);

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

void Calibration::createAxis(int x, int y, Mat *image) {
	line(*image, Point(x - 10, y), Point(x + 10, y), Scalar((unsigned short)-1), 1, 8);
	line(*image, Point(x, y - 10), Point(x, y + 10), Scalar((unsigned short)-1), 1, 8);
}

int Calibration::showAxes(USHORT * imageArray) {

	Mat depthImage = Mat(Size(640, 480), CV_16UC1, imageArray);
	Mat save = depthImage.clone();

	cvNamedWindow("axis", CV_WINDOW_NORMAL );
	cvSetWindowProperty("axis", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	//this->createAxis(config[nrFrame][0], config[nrFrame][1], config[nrFrame][2], config[nrFrame][3], config[nrFrame][4],config[nrFrame][5], &save);
	this->createAxis(vecIntersectionPoints[nrFrame].first, vecIntersectionPoints[nrFrame].second, &save);

	/*//this->setROI(save, x1, x2, y1, y2);
	//this->setROI(depthImage, x1, x2, y1, y2);
	for (int y = y1; y < y2; y++)
		for(int x = x1; x < x2; x++)
			save.at<ushort>(y,x) +=4000;*/

	int newWidth = abs(x2 - x1);
	int newHeight = abs(y2 - y1);

	/*Rect region_of_interest = Rect(x1, y1, newWidth, newHeight);
	Mat roi = Mat(Size(640, 480), CV_16UC1);
	roi	= save(region_of_interest);
	*/
	//Mat newRoi = Mat(640, 480, CV_16UC1);

	/*flip(roi, roi, 1);
	resize(roi, roi, depthImage.size());
	resize(save, save, depthImage.size());
	*/
	/*cout << roi.type() << " " << roi.channels()<<endl;

	for (int y = 0; y < roi.size().height; y++){
		for (int x = 0; x < 640; x++)
			roi.at<uchar>(Point(x, y)) += 5000;
	}
*/
	
	imshow("axis", save);
	int pressedKey =  waitKey(10);

	int checkDetectedCircle = -1;

	if(pressedKey == 49){
		Mat depthImageForCircle = Mat(Size(640, 480), CV_8UC1);
		save.convertTo(depthImageForCircle, CV_8UC1, 1.0/255.0);

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

void Calibration::getFrames(){
	
	int a = 0;
	int count = 0;

	while(count != 36) {
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

	fcout.close();
	cvDestroyWindow("axis");

}

void Calibration::CannyThreshold(int, void *) {
	
	
	Canny(thresh, thresh, lowThreshold, lowThreshold * 3, 3);

	Mat dst;
	thresh.copyTo(dst, thresh);
	imshow("Edge Map", dst);
	waitKey(10);

	/*vector<cv::Vec3f> circles;
	HoughCircles(thresh, circles, HOUGH_GRADIENT, 1, thresh.rows / 4, lowThreshold, 100, 0, 10);
	
	Mat dst;
	thresh.copyTo(dst, thresh);
	

	if (circles.size() == 0) std::exit(-1);
	for (size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
		cv::Point center(std::round(circles[current_circle][0]), std::round(circles[current_circle][1]));
		int radius = std::round(circles[current_circle][2]);

		cv::circle(dst, center, radius, cv::Scalar(0, 255, 0), 5);

	}*/

	//imshow("Edge Map", dst);
	//waitKey(1);

}

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
		//imshow("thresh", thresh);
		//waitKey(10);


		/*noComp = connectedComponentsWithStats(thresh, labels, stats, centroids, 8, CV_16U);
		labels.convertTo(labels, CV_16U);

		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(9);
		const string imageName = "labels.png";


		std::vector<Vec3b> colors(noComp);

		colors[0] = Vec3b(0, 0, 0);//background
		int val = 1;
		for (int label = 1; label < noComp; ++label){
			colors[label] = Vec3b((val & 255), (val + 10 & 255), (val + label & 255));
			val++;
		}

		cout << stats << endl;
		cout << stats.at<int>(0, 4) << endl;

		vector<pair<int, int>> areas;

		for (int i = 0; i < noComp; i++)
			areas.push_back(make_pair(i, stats.at<int>(i, 4)));

		std::sort(areas.begin(), areas.end(), [](const std::pair<int, int> &left, const std::pair<int, int> &right) {
			return left.second < right.second;
		});

		Mat dst(thresh.size(), CV_8UC3);
		for (int r = 0; r < dst.rows; ++r){
			for (int c = 0; c < dst.cols; ++c){
				int label = labels.at<ushort>(r, c);
				Vec3b &pixel = dst.at<Vec3b>(r, c);
				pixel = colors[label];
			}
		}


		imshow("ceva", dst);
		imwrite(imageName, dst, compression_params);
	

		//get Circle from image
		Mat circle;

		compare(labels, areas[noComp - 4].first, circle, CMP_EQ);

		imshow("circle", circle);
		waitKey(10);*/


		Mat lower_red_hue_range;
		Mat upper_red_hue_range;
		
		medianBlur(thresh, thresh, 3);
		GaussianBlur(thresh, thresh, Size(9, 9), 2, 2);

		/*//imshow("Thresh with blur", thresh);

		//char* window_name = "Edge Map";

		//namedWindow("Edge Map", WINDOW_AUTOSIZE);
		//createTrackbar("Min Threshold:", window_name, &lowThreshold, 100, CannyThreshold);*/

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
			//vecOfDetectedCoordinates.push_back(Point3f((worldCoordinates.x * 1000), (worldCoordinates.y * 1000), (worldCoordinates.z * 1000)));
			vecOfDetectedCoordinates.push_back(Point3f((worldCoordinates.x ), (worldCoordinates.y ), (worldCoordinates.z )));

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

void Calibration::readConfig() {

	/*ifstream file("coordinates.txt");

	for(int i = 0; i < 36; i++) {
		for(int j = 0; j < 6; j++) {
			file>>this->config[i][j];
		}
	}*/

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

void Calibration::savePointsForAxes(){
	
	ofstream g("coordonateProiector.txt");
	ofstream f("coordinates.txt");
	

	random_device rd;     // only used once to initialise (seed) engine
	mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
	uniform_int_distribution<int> x_rnd(x1, x2);
	uniform_int_distribution<int> y_rnd(y1, y2);

	for (int i = 0; i < 36; i++){
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

void Calibration::getCoef(){

	CvMat *coordonateProiector = cvCreateMat(72, 1, CV_64FC1);
	CvMat *coordonateXProiector = cvCreateMat(36, 1, CV_64FC1);
	CvMat *coordonateYProiector = cvCreateMat(36, 1, CV_64FC1);
	CvMat *coordonateXKinect = cvCreateMat(36, 1, CV_64FC1);
	CvMat *coordonateYKinect = cvCreateMat(36, 1, CV_64FC1);
	CvMat *coordonateZKinect = cvCreateMat(36, 1, CV_64FC1);
	
	CvMat *matrice = cvCreateMat(72, 12, CV_64FC1);

	float x, y, z;

	ifstream proiectorFile("coordonateProiector.txt");
	for (int i = 0; i < 36; i++){
		proiectorFile >> x >> y;

		cvmSet(coordonateXProiector, i, 0, x);
		cvmSet(coordonateYProiector, i, 0, y);
		cvmSet(coordonateProiector, 2 * i, 0, x);
		cvmSet(coordonateProiector, 2 * i + 1, 0, y);
	}

	ifstream kinectFile("coordonateDetectate.txt");
	for (int i = 0; i < 36; i++){
		kinectFile >> x >> y >> z;

		cvmSet(coordonateXKinect, i, 0, x);
		cvmSet(coordonateYKinect, i, 0, y);
		cvmSet(coordonateZKinect, i, 0, z);
	}

	proiectorFile.close();
	kinectFile.close();

	int index = 0;
	for (int i = 0; i < 72; i++) {

		if (i % 2 == 0) {
			cout << cvmGet(coordonateXProiector, i / 2, 0) << "  " << cvmGet(coordonateYProiector, i / 2, 0);
			cout << " " << cvmGet(coordonateXKinect, i / 2, 0) << " " << cvmGet(coordonateYKinect, i / 2, 0) << " " << cvmGet(coordonateZKinect, i / 2, 0) << endl;

			cvmSet(matrice, i, 0, cvmGet(coordonateXKinect, i / 2, 0)); //xk
			cvmSet(matrice, i, 1, cvmGet(coordonateYKinect, i / 2, 0)); //yk
			cvmSet(matrice, i, 2, cvmGet(coordonateZKinect, i / 2, 0)); //zk
			cvmSet(matrice, i, 3, 1);
			cvmSet(matrice, i, 4, 0);
			cvmSet(matrice, i, 5, 0);
			cvmSet(matrice, i, 6, 0);
			cvmSet(matrice, i, 7, 0);
			cvmSet(matrice, i, 8, (-1) * cvmGet(coordonateXKinect, i / 2, 0) * cvmGet(coordonateXProiector, i / 2, 0));
			cvmSet(matrice, i, 9, (-1) * cvmGet(coordonateYKinect, i / 2, 0) * cvmGet(coordonateXProiector, i / 2, 0) );
			cvmSet(matrice, i, 10, (-1) * (cvmGet(coordonateZKinect, i / 2, 0)) * cvmGet(coordonateXProiector, i / 2, 0));
			cvmSet(matrice, i, 11, (-1) * cvmGet(coordonateXProiector, i / 2, 0));

		}
		else {
			cout << cvmGet(coordonateXProiector, i / 2, 0) << "  " << cvmGet(coordonateYProiector, i / 2, 0);
			cout << " " << cvmGet(coordonateXKinect, i / 2, 0) << " " << cvmGet(coordonateYKinect, i / 2, 0) << " " << cvmGet(coordonateZKinect, i / 2, 0) << endl;			cvmSet(matrice, i, 0, 0);

			cvmSet(matrice, i, 0, 0);
			cvmSet(matrice, i, 1, 0);
			cvmSet(matrice, i, 2, 0);
			cvmSet(matrice, i, 3, 0);
			cvmSet(matrice, i, 4, cvmGet(coordonateXKinect, i / 2, 0)); //xk
			cvmSet(matrice, i, 5, cvmGet(coordonateYKinect, i / 2, 0)); //yk
			cvmSet(matrice, i, 6, cvmGet(coordonateZKinect, i / 2, 0)); //zk
			cvmSet(matrice, i, 7, 1);
			cvmSet(matrice, i, 8, (-1) * cvmGet(coordonateXKinect, i / 2, 0) * cvmGet(coordonateYProiector, i / 2, 0));
			cvmSet(matrice, i, 9, (-1) * cvmGet(coordonateYKinect, i / 2, 0) * cvmGet(coordonateYProiector, i / 2, 0));
			cvmSet(matrice, i, 10, (-1) * (cvmGet(coordonateZKinect, i / 2, 0)) * cvmGet(coordonateYProiector, i / 2, 0));
			cvmSet(matrice, i, 11, (-1) * cvmGet(coordonateYProiector, i / 2, 0));

		}
	}

	cout << "-------------------------------------------------------------------------\n";
	for (int i = 0; i < 72; i++){
		for (int j = 0; j < 12; j++){
			cout << cvmGet(matrice, i, j) << " ";
		}
		cout << endl;
	}

	cout << "Coordonate proiector-------------------------------------------------------------------------\n";

	for (int i = 0; i < 72; i += 2){
		cout << cvmGet(coordonateProiector, i, 0) << "  " << cvmGet(coordonateProiector, i + 1, 0) << endl;
	}

	//cvSolve(matrice, coordonateProiector, solutieQ, CV_SVD);

	Mat solutieValori;
	Mat solutieValoriVector;
	Mat matrice2 = cvarrToMat(matrice);
	Mat transpusa; 
		
	transpose(matrice2, transpusa);
	Mat eigenInput = transpusa * matrice2;

	Mat eigenInputFloat;

	eigenInput.convertTo(eigenInputFloat, CV_32FC1);

	eigen(eigenInputFloat, solutieValori, solutieValoriVector);
	//cout << endl << "Solutia:\n";

	cout << "Valori singulare---------------------------------\n";
	for (int i = 0; i < 12; i++){
		cout << solutieValori.at<float>( i, 0) << "   ";
	}

	cout << endl;
	cout << "Vectori valori singulare cel mai mic-----------------------------\n";
	
	for (int j = 0; j < 12; j++){
		for (int i = 0; i < 12; i++){
			cout << solutieValoriVector.at<float>(i, j) << "   ";
			cvmSet(solutieQ, i, 0, solutieValoriVector.at<float>(i, 10));
		}
		cout << endl<<endl<<endl;
	}


	ofstream f("calibration.txt");

	cout << "solutieQ------------------------------\n";
	 for (int i = 0; i < 12; i++){
		cout << cvmGet(solutieQ, i, 0) << "   ";
		f << cvmGet(solutieQ, i, 0) << "   ";
	}
	f.close();

	cout << "Homography----------------------------------\n";
	//create homography matrix
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 4; j++){
			homographyMatrix.at<float>(i, j) = cvmGet(solutieQ, i*4+j, 0);
			cout << homographyMatrix.at<float>(i, j) << " ";
		}
		cout << endl;
	}


	/*delete coordonateProiector;
	delete coordonateXKinect;
	delete coordonateYKinect;
	delete coordonateZKinect;
	delete coordonateXProiector;
	delete coordonateYProiector;
	delete matrice;*/
	
}

Point2d * Calibration::transformPoint(Point3d kinect) {
	Point2d *proiector = new Point2d();

	double denominator = kinect.x * cvmGet(solutieQ, 8, 0)
						+ kinect.y * cvmGet(solutieQ, 9, 0)
						+ (kinect.z) * cvmGet(solutieQ, 10, 0)
						+ cvmGet(solutieQ, 11, 0);

	double xNumerator = kinect.x * cvmGet(solutieQ, 0, 0)
						+ kinect.y * cvmGet(solutieQ, 1, 0)
						+ (kinect.z) * cvmGet(solutieQ, 2, 0)
						+ cvmGet(solutieQ, 3, 0);

	double yNumerator = kinect.x * cvmGet(solutieQ, 4, 0)
						+ kinect.y * cvmGet(solutieQ, 5, 0)
						+ (kinect.z) * cvmGet(solutieQ, 6, 0)
						+ cvmGet(solutieQ, 7, 0);
	Mat XYZ = Mat(4, 1, CV_32FC1);
	XYZ.at<float>(0, 0) = kinect.x;
	XYZ.at<float>(1, 0) = kinect.y;
	XYZ.at<float>(2, 0) = kinect.z;
	XYZ.at<float>(3, 0) = 1;
	Mat UV = homographyMatrix * XYZ;

	proiector->x = round(xNumerator / denominator);

	proiector->y = round(yNumerator / denominator);

	int xPrim = round(UV.at<float>(0, 0) / UV.at<float>(2, 0));
	int yPrim = round(UV.at<float>(1, 0) / UV.at<float>(2, 0));

	return proiector;
}

void Calibration::doTransformationOfImage(){
	Mat imgKinect1 = imread("calibration\\calibration0.png", CV_LOAD_IMAGE_UNCHANGED | CV_LOAD_IMAGE_ANYDEPTH);

	//Mat imgKinect1 = Mat(Size(640, 480), CV_16UC1, this->aquisition.Update());

	Rect region_of_interest = Rect(x1, y1, abs(x2 - x1), abs(y2 - y1));
	Mat imgKinect = imgKinect1(region_of_interest);


	resize(imgKinect, imgKinect, imgKinect1.size());
	cout<<"Type of imgKinect :"<<imgKinect.type()<<endl;

	Mat imgProjector = Mat(Size(640, 480), CV_16UC1, Scalar(0, 0, 0));
	Size size = imgKinect.size();
	int width = size.width;
	int height = size.height;

	//INuiCoordinateMapper *mapper;
	//this->aquisition.m_pNuiSensor->NuiGetCoordinateMapper(&mapper);

	ofstream f("imagine.txt");
	for(int y = 0; y < height; y++)
	{
		for(int x = 0; x < width; x++)
		{
			int z = imgKinect.at<ushort>(Point(x, y));

			Vector4 worldCoordinates = NuiTransformDepthImageToSkeleton((long)x, (long)y, z << 3, NUI_IMAGE_RESOLUTION_640x480);

			Point3d kinect = Point3d(worldCoordinates.x , worldCoordinates.y , (double)z/1000);

			Point2d * projector = this->transformPoint(kinect);

			FLOAT fSkeletonZ = static_cast<FLOAT>(z) / 1000.0f;
			FLOAT fSkeletonX = (x - width / 2.0f) * (320.0f / width) * NUI_CAMERA_DEPTH_IMAGE_TO_SKELETON_MULTIPLIER_320x240 * fSkeletonZ;

			FLOAT fSkeletonY = -(y - height / 2.0f) * (240.0f / height) * NUI_CAMERA_DEPTH_IMAGE_TO_SKELETON_MULTIPLIER_320x240 * fSkeletonZ;
			

			//int newX = projector->x / ((320.0f / width) * NUI_CAMERA_DEPTH_IMAGE_TO_SKELETON_MULTIPLIER_320x240 * fSkeletonZ) + 

			//if(projector->x > 0 && projector->x < 640 && projector->y > 0 && projector->y < 480){
//				imgProjector.at<ushort>(Point(projector->x , projector->y )) = z + 4000;
			//}
			//else {
				f << "x pixel " << x << "y pixel:" << y << " world->x: " << worldCoordinates.x << " world->y: " << worldCoordinates.y << " world->z: " << worldCoordinates.z<<"projector x = " << projector->x << "prjector y = " << projector->y << endl;

			//}
			
			//f << "x pixel " << x << "y pixel:" << y << " world->x: " << worldCoordinates.x << " world->y: " << worldCoordinates.y << " world->z: " << worldCoordinates.z<<"projector x = " << projector->x << "prjector y = " << projector->y << endl;

			//}

		}
	}

	f.close();

	/*int z = 1120;
	int x = 0;
	int y = 0;
	Vector4 worldCoordinates = NuiTransformDepthImageToSkeleton((long)x, (long)y, z << 3, NUI_IMAGE_RESOLUTION_640x480);

	Point3d kinect = Point3d(worldCoordinates.x * 1000, worldCoordinates.y * 1000, worldCoordinates.z * 1000);
*/

	//cout << "estimat: " << kinect.x << " " << kinect.y << " " << kinect.z << endl;


	for (int y = 0; y < 480; y++){
		for (int x = 0; x < 640; x++){
			imgProjector.at<ushort>(Point(x, y)) += 5000;
			imgKinect.at<ushort>(Point(x, y)) += 5000;
		}
	}
	imshow("proiector",imgProjector);
	imshow("kinect", imgKinect);
	waitKey(0);
}

void Calibration::loadCalibration(){
	ifstream f("calibration.txt");

	double number;
	for (int i = 0; i < 11; i++){
		f >> number;
		cvmSet(solutieQ, i, 0, number);
	}

	f.close();
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