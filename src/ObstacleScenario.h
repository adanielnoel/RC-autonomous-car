/*
 * ObstacleScenario.h
 *
 *  Created on: Sep 27, 2014
 *      Author: alejandro
 */

#ifndef OBSTACLESCENARIO_H_
#define OBSTACLESCENARIO_H_


#include "opencv2/opencv.hpp"
#include <stdio.h>

using namespace cv;
using namespace std;

class ObstacleScenario {
public:
	float width;		//  In meters
	float depth;		//  In meters
	float squareSize;	//  In meters
    float minY;         //  In meters. This is the minimum elevation to start considering.
    float maxY;         //  In meters. This is the maximum elevation to consider.
    float scaleFactor;
	vector< vector<int> > scenario; //A value of 0 means free, 1 means occupied. The origin is the upper-left corner
    vector<Point2f> points;
    
    ObstacleScenario();
    ObstacleScenario(float _width, float _depth, float _squareSize, float _minY = 0, float _maxY = 0);
    ObstacleScenario(int _XSquares, int _YSquares);
    void populateScenario(Mat &image3D, bool &obstaclesDetected);
    void clearScenario();
};

#endif /* OBSTACLESCENARIO_H_ */
