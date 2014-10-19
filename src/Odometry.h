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
//#include "cv.h"
#include <cmath>
#include "StereoPair.h"

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

using namespace cv;
using namespace std;

struct ImgData{
	Mat				image;
	vector<KeyPoint>keyPoints;
	Mat				descriptors;
};

struct pointAndFeat{
	Point3d	point;
	Mat		descriptor;
};

class Odometry {

	//Atributes

		//Data storage
	vector<Mat>					img0;
	vector<vector<KeyPoint> >	kp0;
	vector<Mat>					desc0;

		//Tool objects
	Ptr<FeatureDetector>	detector;
	Ptr<DescriptorExtractor>descriptor;
	Ptr<DescriptorMatcher>	matcher;
	StereoPair* 			camera;

		//Parameters
	static const float	MAXIMUM_EPIPOLAR_DIFFERENCE;//Maximum y difference between key points when DO_EPIPOLAR_FILTER is activated.
	static const bool	DO_CROSS_CHECK = true;			//Enabling cross check ensures that each feature has one only match
	static const bool	DO_EPIPOLAR_FILTER = true;		//Enables epipolar filter for matching stereo images

public:

	//Constructors and destructors
	Odometry();
	Odometry(StereoPair& _camera);
	virtual ~Odometry();

	//Functions
	bool			updateOdometry();
	bool			processNewFrame(Mat& IL, Mat& IR, vector<vector<KeyPoint> > & kpts, vector<Mat> descriptors);
	void			computeFeatures(Mat& img, vector<KeyPoint> & kp, Mat & descriptors);
	vector<Point3f>	localToGlobalCoords(vector<Point3f> localCoordPoints, Mat T, Mat R);
	vector<DMatch>	filteredMatch(vector<KeyPoint> kp1, vector<KeyPoint> kp2, Mat& desc1, Mat& desc2, bool doCrossCheck = true, bool doEpipolarFilter = false);
	Mat				findCommonDescriptors(Mat desc1, Mat desc2, vector<DMatch> & matches);

	//Utilities
	void showLRMatches();
};

/*
 * When a new stereo frame is received, the left image is matched with the last left image to look for known points.
 * The camera pose is updated using the function solvePNPRANSAC passing the known key points positions on the new frame and their
 * global 3D position (calculated on the frame on which they where first seen).
 * Then the new key points (Those that didn't match) are matched with the right image.
 * The local 3D positions of the new key points are calculated using the disparity value and the "dispToDepthMat" matrix
 * (from the camera object).
 * The 3D positions are transformed to global coordinates using the updated camera pose.
 */

#endif /* ODOMETRY_H_ */
