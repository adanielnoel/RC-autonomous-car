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
	vector<Mat> imgFeed;
	vector<KeyPoint> kpFeed;
	vector<Point3d> points;
	Rect mask; //constrains the feature detector algorithms to an area of the image
	int queueSize;

public:
	Odometry();
	Odometry(string detector, string descriptor, string matcher);
	virtual ~Odometry();
	void updateQueue(Mat img, bool showResult);
	Point3d triangulatePos(Point3d point1, Point3d point2, Point3d point3, float dist1, float dist2, float dist3);
};

#endif /* ODOMETRY_H_ */
