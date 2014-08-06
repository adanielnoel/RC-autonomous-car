/*
 * StereoPair.h
 *
 *  Created on: Jul 26, 2014
 *      Author: Alejandro Daniel Noel
 *   Objective:	This class will group all image based computations, such as:
 *   - Image rectification
 *   - Depth map generation
 *   - Key point extraction
 *   - Features generation
 *   - 3D block depth map generation (Minecraft style)
 */

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/opencv.hpp"
#include <stdio.h>
#include "cv.h"

#ifndef STEREOPAIR_H_
#define STEREOPAIR_H_

using namespace cv;
using namespace std;

struct Rectification
{
	Mat rmap[2][2];
	Mat K;
	double B;
};

class StereoPair {
	//Atributes
	VideoCapture	camL;	// Left camera
	VideoCapture	camR;	// right camera
	Rectification	recti;	// Rectification maps
	StereoSGBM		sgbm;	// Disparity computation method
	Mat				imgl;	// Rectified left image
	Mat				imgr;	// Rectified right image
	Mat				dmp;	// Disparity map (not normalised)
	static double depthCoef; //multiplying by depth value gives the metric depth.
	static double scaleCoef; //multiplying by depth value gives what distance represents a pixel at that depth.

public:
	//Constructors and destructors
	StereoPair();			//TODO: default constructor does nothing!
	StereoPair(int lCamId, int rCamId, int camFPS);
	virtual ~StereoPair();

	//Initialization methods
	void setupRectification(string calibrationFile, string calOutput);
	void setupDisparity();

	//Functions
	Mat rectifyImage(const Mat& I, const Rectification& recti, bool left);
	bool updateRectifiedPair();
	void updateDepthMap();
	bool pixelToPoint(Mat& _dmp, Point2f pixel, Point3d& point);

	//Get methods
	Mat getMainImg();
	Mat getDepthMap();
	Mat getDepthMapNormalised();

	//Utilities
	void calibrateCoefs();								//Utility to calibrate depthCoef and scaleCoef
	void saveCalibrationFrames(string outputFolder);	//on 's' key press saves stereo images. Useful to get chess board images.
	void saveCalibratedImages(string outputFolder);		//on 's' key press saves rectified images.
	void RectificationViewer();							//Shows rectified images side to side with horizontal lines.
};


#endif /* STEREOPAIR_H_ */
