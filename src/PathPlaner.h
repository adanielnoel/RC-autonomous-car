/*
 * PathPlaner.h
 *
 *  Created on: Sep 1, 2014
 *      Author: alejandro
 */

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/opencv.hpp"
#include <stdio.h>
//#include "cv.h"

#include "ObstacleScenario.h"

#ifndef PATHPLANER_H_
#define PATHPLANER_H_

using namespace std;
using namespace cv;

class PathPlaner {
	static const float WAYPOINT_DIST;				//In the same units as Scenario.squareSize
	static const float MIN_DIST_FROM_OBSTACLE;	//In the same units as Scenario.squareSize

public:
	PathPlaner();
	virtual ~PathPlaner();
	bool findNavigationPath(ObstacleScenario scenario, Point2i initialLocation, Point2i targetLocation);
    bool findAvoidancePath(ObstacleScenario scenario, Mat &display, int squarePixelSize);
	Point expandRange();
};

#endif /* PATHPLANER_H_ */
