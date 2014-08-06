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

namespace sp 
{

class StereoPair 
{
	Mat imgl, imgr, dmp;  //imgl and imgr store rectified images
	VideoCapture camL, camR;
	Rectification recti;

public:

	// Constructors and destructors
	StereoPair					();
	StereoPair					(Mat IL, Mat IR);
	StereoPair					(const int lCamId, const int rCamId, const int camFPS);
	virtual ~StereoPair			();

	// Initialization methods
	void setupRectification		(const string& calibrationFile, const string& calOutput); // This version writes the new calibration parameters to the specified file
	void setupRectification		(const string& calibrationFile);

	// Functions
	Mat rectifyImage			(const Mat& I, const Rectification& recti, const bool left);
	void RectificationViewer	(const Mat& IL, const Mat& IR);
	bool updateRectifiedPair	();
	void updateDepthMap			();
	void saveCalibrationFrames	(string outputFolder); //On key press saves stereo images. Useful to get chess board images.
	
	// Get methods
	Mat getMainImg				();
	
	// Set methods
};

} /* namespace sp */
#endif /* STEREOPAIR_H_ */
