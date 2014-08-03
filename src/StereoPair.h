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

namespace sp {

class StereoPair {
	Mat imgl, imgr, dmp;  //imgl and imgr store rectified images
	VideoCapture camL, camR;
	Rectification recti;
public:
	StereoPair();
	virtual ~StereoPair();
	StereoPair(Mat IL, Mat IR);
	StereoPair(int lCamId, int rCamId, int camFPS);
	void setupRectification(string calibrationFile, string calOutput); //this version writes the new calibration parameters to the specified file
	void setupRectification(string calibrationFile);
	Mat rectifyImage(const Mat& I, const Rectification& recti, bool left);
	void RectificationViewer(Mat& IL, Mat& IR);
	bool updateRectifiedPair();
	void updateDepthMap();
	Mat getMainImg();
	void saveCalibrationFrames(string outputFolder); //on key press saves stereo images. Useful to get chess board images.
};

} /* namespace sp */
#endif /* STEREOPAIR_H_ */
