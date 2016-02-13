/*
 * PathPlaner.h
 *
 *  Created on: Sep 1, 2014
 *      Author: alejandro
 */

#ifndef PATHPLANER_H_
#define PATHPLANER_H_

#include "opencv2/opencv.hpp"
#include <stdio.h>
#include <cmath>
//#include "cv.h"

#include "ObstacleScenario.h"

using namespace std;
using namespace cv;

struct Corner{
    Point2f cornerLoc;
    bool isTop;
    bool isLeft;
};

struct RadiusPair{
    float radius1;
    float radius2;
};

class PathPlaner {
	static const float WAYPOINT_DIST;				//In the same units as Scenario.squareSize
    static const Scalar COLOR_PATH;
    static const Scalar COLOR_PATH_LIMITS;
    static const Scalar COLOR_CORNERS;
    vector<vector<int> > scen;
    int pixelsPerMeter; //In meters
    float squareSize;   //In meters
    float radiusForTangency(Point2f tangentPoint);
    void drawAvoidancePaths(Mat &display, vector<RadiusPair> radiusRanges, Point2f viewerLoc);
    void neighborExpand(Point currentSquare, Point viewerLoc, vector<Corner> &corners);
    
public:
	static const float HALF_VEHICLE_WIDTH;          //In the same units as Scenario.squareSize
    static const float MINIMUM_RADIUS;
	PathPlaner();
	virtual ~PathPlaner();
    bool updateObstacleScenario(Mat dispImg);
    float findAvoidancePath(ObstacleScenario &scenario, float initialCurveRadius, Mat &display, int squarePixelSize);
	bool findNavigationPath(ObstacleScenario &scenario, Point2i initialLocation, Point2i targetLocation);
	Point expandRange();
};

#endif /* PATHPLANER_H_ */
