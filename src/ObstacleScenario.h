/*
 * ObstacleScenario.h
 *
 *  Created on: Sep 27, 2014
 *      Author: alejandro
 */

#ifndef OBSTACLESCENARIO_H_
#define OBSTACLESCENARIO_H_

#include "cv.h"

using namespace cv;

class ObstacleScenario {
public:
	int width;			//In meters
	int height;			//In meters
	float squareSize;	//In meters
	vector< vector<int> > scenario;
	vector<Point2f> path;
};

#endif /* OBSTACLESCENARIO_H_ */
