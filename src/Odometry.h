/*
 * Odometry.h
 *
 *  Created on: Jul 29, 2014
 *      Author: alejandro
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_


#include "opencv2/opencv.hpp"
//#include <stdio.h>
//#include "cv.h"
#include <cmath>
//#include "StereoPair.h"

class StereoPair;

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
    float maxEpipolarDifference;                        //Maximum y difference between key points when DO_EPIPOLAR_FILTER is activated.
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


#endif /* ODOMETRY_H_ */
