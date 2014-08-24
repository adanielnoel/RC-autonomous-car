/*
 * Odometry.h
 *
 *  Created on: Jul 29, 2014
 *      Author: alejandro
 */

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/opencv.hpp"
#include <stdio.h>
#include "cv.h"
#include <cmath>
#include "StereoPair.h"

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

using namespace cv;
using namespace std;

struct ImgData{
	Mat image;
	vector<KeyPoint> keyPoints;
	Mat descriptors;
};

struct pointAndFeat{
	Point3d point;
	Mat descriptor;
};

class Odometry {

	//Atributes

		//Data storage
	Mat lastImage;
	Mat lastDescriptors;
	vector<Point3f> lastPoints;

		//Tool objects
	Ptr<FeatureDetector> detector;
	Ptr<DescriptorExtractor> descriptor;
	Ptr<DescriptorMatcher> matcher;
	StereoPair camera;

		//Parameters
	double ransacReprojThreshold = 3; //match filtering threshold for RANSAC

public:
	//Class parameters
	static const int AUTO_REPEAT;
	static const int PROMPT_REPEAT;

	//Constructors and destructors
	Odometry();
	Odometry(StereoPair _camera);
	virtual ~Odometry();

	//Functions
	bool updateOdometry();
	bool processNewFrame(Mat& image, vector<KeyPoint> & kp, Mat & descriptors);
	void matchLeftRight(Mat& imgL, Mat& imgR, vector<KeyPoint>matchedKeypoints, Mat matchedDescriptors,vector<DMatch> matches);
	vector<DMatch> filteredMatch(vector<KeyPoint> kpL, vector<KeyPoint> kpR, Mat& descL, Mat& descR, bool doCrossCheck);
	Mat findCommonDescriptors(Mat desc1, Mat desc2, vector<DMatch> & matches);

	//Utilities
	void showLRMatches();
};

/*
 * When a new stereo frame is received, left and right images are matched to find valid key points (which 3D point can be calculated).
 * The valid key points are matched with the previous frame and separated between known (seen on the last frame) and new (not seen before).
 * The camera pose is updated with the function solvePNPRANSAC passing the known key points positions on the new frame and their
 * global 3D position (calculated on the frame on which they where first seen).
 * The local 3D positions of the new key points are calculated using the disparity value and the "dispToDepthMat" matrix (from the camera object).
 * The 3D positions are transformed to global coordinates using the updated camera pose.
 */

#endif /* ODOMETRY_H_ */
