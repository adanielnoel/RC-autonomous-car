/*
 * ObstacleScenario.h
 *
 *  Created on: Sep 27, 2014
 *      Author: alejandro
 */

#ifndef OBSTACLESCENARIO_H_
#define OBSTACLESCENARIO_H_

//#include "cv.h"

using namespace cv;

class ObstacleScenario {
public:
	int width;			//In meters
	int height;			//In meters
	float squareSize;	//In meters
	vector< vector<int> > scenario; //A value of 0 means free, 1 means occupied
	vector< vector<int> > path;
	vector<Point2f> curvePath;
};

#endif /* OBSTACLESCENARIO_H_ */
