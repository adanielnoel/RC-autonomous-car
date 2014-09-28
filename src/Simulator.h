/*
 * Simulator.h
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
#include "cv.h"
#include <cmath>

#include "ObstacleScenario.h"
#include "PathPlaner.h"

#ifndef SIMULATOR_H_
#define SIMULATOR_H_

using namespace std;
using namespace cv;


class Simulator {
	Scalar colorEmpty;
	Scalar colorOccupied;
	Scalar colorShadow;
	Scalar colorGrid;
	Scalar colorMessage;
	Scalar colorTargetDirection;
	int XSquares;
	int YSquares;	//in square count
	Size windowSize;
public:
	Simulator(float _depth, float fov, float squareSize, Size _windowSize);	//Use real world measures
	void runSimulation();
	virtual ~Simulator();
};

#endif /* SIMULATOR_H_ */
