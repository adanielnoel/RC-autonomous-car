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

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

using namespace cv;
using namespace std;

class Odometry {

	// Atributes
	vector<Mat>			imgFeed;	// Queue of images
	vector<KeyPoint>	kpFeed;		// Queue of key point by image
	vector<Point3d>		points;		// Queue of ...
	Rect				mask;		// Constrains the feature detector algorithms to an area of the image
	int					queueSize;	// Size of the queue

public:
	// Constructors and destructors
	Odometry();
	Odometry(const string& detector, const string& descriptor, const string& matcher);
	virtual ~Odometry();
	
	// Functions
	void updateQueue(Mat img, const bool showResult);
	Point3d triangulatePos(Point3d point1, Point3d point2, Point3d point3, float dist1, float dist2, float dist3);
	
	// Get methods
	
	// Set methods
};

#endif /* ODOMETRY_H_ */
