#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "Calibration.h"
#include <strsafe.h>
#include <iostream>
#include <string>
#include <fstream>
#include <math.h>


int Calibration::nrFrame = 0;
int Calibration::nrClick = 0;
int Calibration::x1 = 0;
int Calibration::y1 = 0;
int Calibration::x2 = 0;
int Calibration::y2 = 0;
CvMat *Calibration::solutieQ = cvCreateMat(11, 1, CV_64FC1);

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

int Calibration::showAxes(USHORT * imageArray) {

	Mat depthImage = Mat(Size(640, 480), CV_16UC1, imageArray).clone();
	Mat save = depthImage.clone();

	cvNamedWindow("axis", CV_WINDOW_NORMAL );
	cvSetWindowProperty("axis", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
	this->createAxis(config[nrFrame][0], config[nrFrame][1], config[nrFrame][2], config[nrFrame][3], config[nrFrame][4],config[nrFrame][5], &save);


	this->setROI(save, x1, x2, y1, y2);
	this->setROI(depthImage, x1, x2, y1, y2);
	for (int y = y1; y < y2; y++)
		for(int x = x1; x < x2; x++)
			save.at<ushort>(y,x) +=1000;
	imshow("axis", save);
	int pressedKey =  waitKey(10);
	if(pressedKey == 49){
		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(9);
		ostringstream imageName;
		imageName<<"calibration\\calibration" + to_string(nrFrame) << ".png";
		cout<<imageName.str()<<endl;
		IplImage img = depthImage;

		imwrite(imageName.str().c_str(), depthImage, compression_params);
		nrFrame++;
	}

	return pressedKey;
}

void Calibration::getFrames(){
	
	int a = 0;
	int count = 0;

	while(count != 12) {
		cout<<"aici\n";
		while(a != 49){
			a = this->showAxes(this->aquisition.Update());
		}
		count++;
		a = 0;
	}
}

void Calibration::showCircles() {

	ofstream file("coordonateDetectate.txt"); 

	int nrFrame = 0;

	while(nrFrame != 12){
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
				if(src.at<ushort>(Point(x, y)) > 1000 && src.at<ushort>(Point(x, y)) < 1150)
					{
						nrPixels ++;
						nr_x += x;
						nr_y += y;


						src.at<ushort>(Point(x, y)) = 54000;
						
				} else {
					src.at<ushort>(Point(x, y)) = 0;					
				}
			}
		}
		imshow("tresh", src);

		//cout<<src.data;

	
		int x = nr_x/nrPixels;
		int y = nr_y/nrPixels;
		int z = (int)tresh.at<ushort>(Point(x, y));
		Point center(x, y);
		file<<x << " " << y<<" "<<z<<"\n";
		imshow("test", src);
		waitKey(0);
	nrFrame++;
	}

	file.close();

}

void Calibration::readConfig() {

	ifstream file("coordinates.txt");

	for(int i = 0; i < 12; i++) {
		for(int j = 0; j < 6; j++) {
			file>>this->config[i][j];
		}
	}

	for(int i = 0; i < 12; i++) {
		for(int j = 0; j < 6; j++) {


			cout<<this->config[i][j]<<" ";
		}
		cout<<endl;
	}
	file.close();
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
	
	while(nrClick != 2) {
		Mat img = Mat(Size(640, 480), CV_16UC1, this->aquisition.Update()).clone();
		namedWindow("GetCoordinates");
		setMouseCallback("GetCoordinates", CallBackFunc, NULL);
		imshow("GetCoordinates", img);
		waitKey(10);

		if(nrClick == 1){
			cout<<"x1 = "<<x1<<" y1 = "<<y1<<" z = "<<img.at<ushort>(Point(x1, y1))<<endl;
			
		}

		if(nrClick == 2){
			cout<<"x2 = "<<x2<<" y2 = "<<y2<<" z = "<<img.at<ushort>(x2, y2)<<endl;
		}
	}

	cvDestroyWindow("GetCoordinates");
}

void Calibration::setROI(Mat &img, int x1, int x2, int y1, int y2){
	Size size = img.size();
	int width = size.width;
	int height = size.height;

	//cout<<width<<"   "<<height<<endl;
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

	//circle(img, Point(x1, y1), 10, Scalar(255,0,0), -1, 8, 0);
	//circle(img, Point(x2, y2), 10, Scalar(255,0,0), -1, 8, 0);
	//rectangle(img, Point(x1, y1), Point(x2, y2), Scalar(0,255,0), 3);
}

void Calibration::savePointsForAxes(){
	
	ofstream f("coordinates.txt");
	//f<<"coordonatele sunt: "<<x1<<"  "<<y1<<" "<<x2<<" "<<y2<<endl;
	f << (x2 + 5 * x1) / 6 << " "<<(x2 + 5 * x1) / 6 << " "<< (x2 + 5 * x1) / 6 << " " << ( y2 + 7 * y1) / 8 << " " << ( y2 + 7 * y1) / 8 << " " << ( y2 + 7 * y1) / 8 <<"\n";
	f << (x2 + x1) / 2 << " " << (x2 + x1) / 2 << " "<< (x2 + x1) / 2 << " "<< ( y2 + 7 * y1) / 8 << " " << ( y2 + 7 * y1) / 8 << " " << ( y2 + 7 * y1) / 8<<"\n";
	f << ((5 * x2) + x1) / 6 << " "<< ((5 * x2) + x1) / 6 << " "<< ((5 * x2) + x1) / 6 << " " << ( y2 + 7 * y1) / 8 << " " << ( y2 + 7 * y1) / 8 << " " << ( y2 + 7 * y1) / 8<<"\n";
	f << (x2 + 5 * x1) / 6 << " "<< (x2 + 5 * x1) / 6 << " "<< (x2 + 5 * x1) / 6 << " " << ( 7 * y2 - y1) / 8<< " " << ( 7 * y2 - y1) / 8<< " " << ( 7 * y2 - y1) / 8 <<"\n";
	f << (x2 + x1) / 2 << " "<< (x2 + x1) / 2 << " "<< (x2 + x1) / 2 << " " << ( 7 * y2 - y1) / 8<< " " << ( 7 * y2 - y1) / 8 << " " << ( 7 * y2 - y1) / 8  <<"\n";
	f << ((5 * x2) + x1) / 6 << " "<< ((5 * x2) + x1) / 6 << " "<< ((5 * x2) + x1) / 6 << " " << ( 7 * y2 - y1) / 8 << " " << ( 7 * y2 - y1) / 8<< " " << ( 7 * y2 - y1) / 8<<"\n";

	f << (x2 + 3 * x1) / 4 << " "<< (x2 + 3 * x1) / 4 << " "<< (x2 + 3 * x1) / 4 << " "<<(y2 + 2 * y1) / 3<<" "<<(y2 + 2 * y1) / 3<<" "<<(y2 + 2 * y1) / 3<<" "<<"\n";
	f << (3 * x2 + x1) / 4 << " "<< (3 * x2 + x1) / 4 << " "<< (3 * x2 + x1) / 4 << " "<<(y2 + 2 * y1) / 3<<" "<<(y2 + 2 * y1) / 3<<" "<<(y2 + 2 * y1) / 3<<" "<<"\n";
	f << (x2 + 3 * x1) / 4 << " "<< (x2 + 3 * x1) / 4 << " "<< (x2 + 3 * x1) / 4 << " "<<(2 * y2 + y1) / 3<<" "<< " "<<(2 * y2 + y1) / 3<<" "<< " "<<(2 * y2 + y1) / 3<<"\n";
	f << (3 * x2 + x1) / 4 << " "<< (3 * x2 + x1) / 4 << " "<< (3 * x2 + x1) / 4 << " "<<(2 * y2 + y1) / 3<<" "<< " "<<(2 * y2 + y1) / 3<<" "<< " "<<(2 * y2 + y1) / 3<<"\n";
	
	f << (((x2 + x1) / 2) - 30) << " "<< (((x2 + x1) / 2) - 30) << " "<< (((x2 + x1) / 2) - 30) << " "<< ((y1 + y2) / 2)<< " "<< ((y1 + y2) / 2)<< " "<< ((y1 + y2) / 2)<< "\n";
	f << (((x2 + x1) / 2) + 30) << " "<< (((x2 + x1) / 2) + 30) << " "<< (((x2 + x1) / 2) + 30) << " "<< ((y1 + y2) / 2)<< " "<< ((y1 + y2) / 2)<< " "<< ((y1 + y2) / 2)<< "\n";
	f.close();


	ofstream g("coordonateProiector.txt");
	g << (x2 + 5 * x1) / 6 << " "<< ( y2 + 7 * y1) / 8  <<"\n";
	g << (x2 + x1) / 2 << " "  << ( y2 + 7 * y1) / 8<<"\n";
	g << ((5 * x2) + x1) / 6 << " "<<( y2 + 7 * y1) / 8<<"\n";
	g << (x2 + 5 * x1) / 6 << " " << ( 7 * y2 - y1) / 8 <<"\n";
	g << (x2 + x1) / 2 << " " << ( 7 * y2 - y1) / 8  <<"\n";
	g << ((5 * x2) + x1) / 6 << " " << ( 7 * y2 - y1) / 8<<"\n";

	g << (x2 + 3 * x1) / 4 << " "<<(y2 + 2 * y1) / 3<<" "<<"\n";
	g << (3 * x2 + x1) / 4 << " "<<" "<<(y2 + 2 * y1) / 3<<" "<<"\n";
	g << (x2 + 3 * x1) / 4 << " "<<(2 * y2 + y1) / 3<<"\n";
	g << (3 * x2 + x1) / 4 << " "<<(2 * y2 + y1) / 3<<"\n";
	
	g << (((x2 + x1) / 2) - 30) << " "<< ((y1 + y2) / 2)<< "\n";
	g << (((x2 + x1) / 2) + 30) << " "<< ((y1 + y2) / 2)<< "\n";

	g.close();

}

void Calibration::detectCircle(){
	
	Mat image = imread("calibration\\calibration1.png", 0);
	Mat dst;
	
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

	CvMat *coordonateProiector = cvCreateMat(24, 1, CV_64FC1);
	CvMat *coordonateXProiector = cvCreateMat(12, 1, CV_64FC1);
	CvMat *coordonateYProiector = cvCreateMat(12, 1, CV_64FC1);
	CvMat *coordonateXKinect = cvCreateMat(12, 1, CV_64FC1);
	CvMat *coordonateYKinect = cvCreateMat(12, 1, CV_64FC1);
	CvMat *coordonateZKinect = cvCreateMat(12, 1, CV_64FC1);
	CvMat *matrice1 = cvCreateMat(12, 11, CV_64FC1);
	CvMat *matrice2 = cvCreateMat(12, 11, CV_64FC1);
	CvMat *matrice = cvCreateMat(24, 11, CV_64FC1);
	
	int x, y, z;

	ifstream proiectorFile("coordonateProiector.txt");
	for(int i = 0; i < 12; i++){
		proiectorFile>>x>>y;
		//cout<<x<<" "<<y<<endl;
		cvmSet(coordonateXProiector, i, 0, x);
		cvmSet(coordonateYProiector, i, 0, y);
		/*cvmSet(coordonateXProiector, i+12, 0, x);
		cvmSet(coordonateYProiector, i+12, 0, y);*/

		cvmSet(coordonateProiector, 2*i, 0, x);
		cvmSet(coordonateProiector, 2*i + 1, 0, y);
		
	}


	
	ifstream kinectFile("coordonateDetectate.txt");
	for(int i = 0; i < 12; i++){
		kinectFile>>x>>y>>z;

		//cout<<x<<"  "<<y<<"  "<<z<<"  "<<endl;
		cvmSet(coordonateXKinect, i, 0, x);
		cvmSet(coordonateYKinect, i, 0, y);
		cvmSet(coordonateZKinect, i, 0, z);
	}

	proiectorFile.close();
	kinectFile.close();

	int index = 0;
	for(int i = 0; i < 24; i++){

		if(i % 2 == 0) {
			cout<<cvmGet(coordonateXProiector, i/2, 0)<<"  "<<cvmGet(coordonateYProiector, i/2, 0);
			cout<<" "<<cvmGet(coordonateXKinect, i/2, 0)<<" "<<cvmGet(coordonateYKinect, i/2, 0)<<" "<<cvmGet(coordonateZKinect, i/2, 0)<<endl;
			
			cvmSet(matrice, i, 0, cvmGet(coordonateXKinect, i/2, 0)); //xk
			cvmSet(matrice, i, 1, cvmGet(coordonateYKinect, i/2, 0)); //yk
			cvmSet(matrice, i, 2, cvmGet(coordonateZKinect, i/2, 0)); //zk
			cvmSet(matrice, i, 3, 1);						
			cvmSet(matrice, i, 4, 0);						
			cvmSet(matrice, i, 5, 0);						
			cvmSet(matrice, i, 6, 0);						
			cvmSet(matrice, i, 7, 0);						
			cvmSet(matrice, i, 8, (-1) * cvmGet(coordonateXKinect, i/2, 0) * cvmGet(coordonateXProiector, i/2, 0));
			cvmSet(matrice, i, 9, (-1) * cvmGet(coordonateYKinect, i/2, 0) * cvmGet(coordonateXProiector, i/2, 0));
			cvmSet(matrice, i, 10, (-1) * cvmGet(coordonateZKinect, i/2, 0) * cvmGet(coordonateXProiector, i/2, 0));
		} else {
            cout<<cvmGet(coordonateXProiector, i/2, 0)<<"  "<<cvmGet(coordonateYProiector, i/2, 0);
			cout<<" "<<cvmGet(coordonateXKinect, i/2, 0)<<" "<<cvmGet(coordonateYKinect, i/2, 0)<<" "<<cvmGet(coordonateZKinect, i/2, 0)<<endl;			cvmSet(matrice, i, 0, 0);
			
			cvmSet(matrice, i, 0, 0);
			cvmSet(matrice, i, 1, 0);
			cvmSet(matrice, i, 2, 0);
			cvmSet(matrice, i, 3, 0);
			cvmSet(matrice, i, 4, cvmGet(coordonateXKinect, i/2, 0)); //xk
			cvmSet(matrice, i, 5, cvmGet(coordonateYKinect, i/2, 0)); //yk
			cvmSet(matrice, i, 6, cvmGet(coordonateZKinect, i/2, 0)); //zk
			cvmSet(matrice, i, 7, 1);
			cvmSet(matrice, i, 8, (-1) * cvmGet(coordonateXKinect, i/2, 0) * cvmGet(coordonateYProiector, i/2, 0));
			cvmSet(matrice, i, 9, (-1) * cvmGet(coordonateYKinect, i/2, 0) * cvmGet(coordonateYProiector, i/2, 0));
			cvmSet(matrice, i, 10, (-1) * cvmGet(coordonateZKinect, i/2, 0) * cvmGet(coordonateYProiector, i/2, 0));
		}
	}

	cout<<"-------------------------------------------------------------------------\n";
	for(int i = 0; i < 24; i++){
		for(int j = 0; j < 11; j++){
			cout<<cvmGet(matrice, i, j)<<" ";
		}
		cout<<endl;
	}
	
	for(int i = 0; i < 24; i++){
		cout<<cvmGet(coordonateProiector, i, 0)<<endl;
	}

	cvSolve(matrice, coordonateProiector, solutieQ, CV_SVD);
	cout<<endl<<"Solutia:\n";

	for(int i = 0; i <11; i++)
		cout<<cvmGet(solutieQ, i, 0)<<"   ";
}

Point2d * Calibration::transformPoint(Point3d kinect) {
	Point2d *proiector = new Point2d();

	proiector->x = (int)((kinect.x * cvmGet(solutieQ, 0, 0) 
					+ kinect.y * cvmGet(solutieQ, 1, 0) 
					+ kinect.z * cvmGet(solutieQ, 2, 0) 
					+ cvmGet(solutieQ, 3, 0)) 
					/(kinect.x * cvmGet(solutieQ, 8, 0)
					+ kinect.y * cvmGet(solutieQ, 9, 0)
					+ kinect.z * cvmGet(solutieQ, 10, 0)
					+ 1));

	proiector->y = (int)((kinect.x * cvmGet(solutieQ, 4, 0) 
					+ kinect.y * cvmGet(solutieQ, 5, 0) 
					+ kinect.z * cvmGet(solutieQ, 6, 0) 
					+ cvmGet(solutieQ, 7, 0)) 
					/(kinect.x * cvmGet(solutieQ, 8, 0)
					+ kinect.y * cvmGet(solutieQ, 9, 0)
					+ kinect.z * cvmGet(solutieQ, 10, 0)
					+ 1));

	return proiector;
}

void Calibration::doTransformationOfImage(){
	Mat imgKinect = imread("calibration\\calibration1.png");
	Mat imgProjector = Mat(Size(640, 480), CV_16UC1);
	Size size = imgKinect.size();
	int width = size.width;
	int height = size.height;

	for(int y = 0; y < height; y++)
	{
		for(int x = 0; x < width; x++)
		{
			int z = imgKinect.at<ushort>(Point(x, y));
			//cout<<z<<" ";
			Point3d kinect = Point3d(x, y, z);
			Point2d * projector = this->transformPoint(kinect);
			
			//cout<<" xk= "<<x<<" xp= "<<projector->x<<" yk= "<<y<<" yp= "<<projector->y<<endl;
			imgProjector.at<ushort>(Point(projector->x, projector->y)) = z+4000;
		}
		//cout<<endl;
	}


	imshow("tets",imgProjector);
	waitKey(0);
}