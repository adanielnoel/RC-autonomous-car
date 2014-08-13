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
	VideoCapture	camL;			// Left camera
	VideoCapture	camR;			// right camera
	Rectification	recti;			// Rectification maps
	StereoSGBM		sgbm;			// Disparity computation method
	Mat				imgl;			// Rectified left image
	Mat				imgr;			// Rectified right image
	Mat				dsp;			// Disparity map (not normalized)
	Mat				img3D;			// Depth map
	Mat				dispToDepthMat; //matrix from stereoRectify(..., Q, ...);

public:
	//Constructors and destructors
	StereoPair();			//TODO: default constructor does nothing!
	StereoPair(int lCamId, int rCamId, int camFPS, bool & success);
	virtual ~StereoPair();

	//Initialization methods
	void setupRectification(string calibrationFile, string calOutput);
	void setupDisparity();

	//Functions
	Mat rectifyImage(const Mat& unrectifiedImage, const Rectification& recti, bool left);
	bool updateRectifiedPair();
	void updateDisparityImg();
	void updateImg3D();

	//Get methods
	Mat getMainImg();
	Mat getDisparityImg();
	Mat getImg3D();
	Mat getDisparityImgNormalised();

	//Utilities
	void saveUncalibratedStereoImages(string outputFolder);		//on 's' key press saves stereo images. Useful to get chess board images.
	void saveCalibratedStereoImages(string outputFolder);		//on 's' key press saves rectified images.
	void RectificationViewer();									//Shows rectified images side to side with horizontal lines.
};


#endif /* STEREOPAIR_H_ */
