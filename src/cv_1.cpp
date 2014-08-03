//============================================================================
// Name        : cv_1.cpp
// Author      : Alejandro
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

/*#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/opencv.hpp"
#include <stdio.h>
#include "cv.h"

using namespace cv;
using namespace std;

const int dmHeight = 5;
const int dmPos = 10;



int main(int argc, char** argv) {
	Mat img1, img2;
	Size scale = Size(300, 200);

	img1 = imread( argv[1], 0);
	img2 = imread( argv[2], 0);

	if(!img1.data || !img2.data)
	    {
	      printf( "No image data \n" );
	      return -1;
	    }

	resize(img1, img1, scale);
	resize(img2, img2, scale);


	StereoSGBM sgbm;
	sgbm.SADWindowSize = 7;
	sgbm.numberOfDisparities = 208;
	sgbm.preFilterCap = 4;
	sgbm.minDisparity = 0;
	sgbm.uniquenessRatio = 1;
	sgbm.speckleWindowSize = 100;
	sgbm.speckleRange = 2;
	sgbm.disp12MaxDiff = 10;
	sgbm.fullDP = false;
	sgbm.P1 = 600;
	sgbm.P2 = 2400;

    namedWindow("disp", 1);
    namedWindow("img", 1);
	int slider_dmPos = dmPos;
	createTrackbar("pos", "img", &slider_dmPos, 200-dmHeight);

	while (true)
	{
		Mat imcrp1, imcrp2, disp;
		Rect cropArea = Rect(0, slider_dmPos, img1.cols, dmHeight);
		imcrp1 = img1(cropArea).clone();
		imcrp2 = img2(cropArea).clone();
		sgbm(imcrp1, imcrp2, disp);
		normalize(disp, disp, 0, 255, CV_MINMAX, CV_8U);
		Mat tview = Mat(Size(300, 255), CV_8UC3, Scalar(255, 255, 255));

		for(int x=0; x<300; x++){
			int depth = 0;
			for(int i=0; i<dmHeight; i++){
				Scalar _depth = disp.at<uchar>(i, x);
				depth += _depth[0];
			}
		depth = (int)depth/dmHeight;
		tview.at<Vec3b>(depth, x) = Vec3b(0, 255 -depth, depth);
		if(depth<255 && depth >0){
			tview.at<Vec3b>(depth-1, x) = Vec3b(0, 255-depth, depth);
			tview.at<Vec3b>(depth-2, x) = Vec3b(0, 255-depth, depth);
			}
		}
		Mat _img1 = img1.clone();
		rectangle(_img1,cropArea, Scalar(255, 0, 0));
		imshow("img", _img1);
		imshow("disp", tview);

		// Wait until user press some key for 50ms
		int iKey = waitKey(50);

		//if user press 'ESC' key
		if (iKey == 27) {break;}
	}

	waitKey(0);

	return 0;/*
	VideoCapture cap(0); // open the default camera

	if(!cap.isOpened()){ // check if we succeeded
		printf("no webcam");
		return -1;
	}
	printf("sdce");
	Mat edges;

	namedWindow("frame",1);

	Mat frame;
	for(;;){
	cap.retrieve(frame); // get a new frame from camera
	if(!frame.data){
		printf("no data \n");
		continue;
	}
	imshow("frame", frame);
	}
	waitKey(0);

	return 0;*/
//}
