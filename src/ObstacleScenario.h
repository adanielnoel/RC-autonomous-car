/*
 * ObstacleScenario.h
 *
 *  Created on: Sep 27, 2014
 *      Author: alejandro
 */

#ifndef OBSTACLESCENARIO_H_
#define OBSTACLESCENARIO_H_

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

class ObstacleScenario {
public:
    Rect regionOfInterest; //This contains the region of the disparyty image used to populate the 2D map.
	float width;			//In meters
	float depth;			//In meters
	float squareSize;	//In meters
	vector< vector<int> > scenario; //A value of 0 means free, 1 means occupied
    
    ObstacleScenario();
    ObstacleScenario(float _width, float _depth, float _squareSize);
    ObstacleScenario(int _XSquares, int _YSquares);
    void populateScenario(Mat &image3D, bool &obstaclesDetected);
    void clearScenario();
};

#endif /* OBSTACLESCENARIO_H_ */
