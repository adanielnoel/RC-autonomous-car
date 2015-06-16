/*
 * Simulator.h
 *
 *  Created on: Sep 1, 2014
 *      Author: alejandro
 */

#ifndef SIMULATOR_H_
#define SIMULATOR_H_

#include <stdio.h>
#include <cmath>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/opencv.hpp"
//#include "cv.h"

#include "ObstacleScenario.h"
#include "PathPlaner.h"

using namespace std;
using namespace cv;


class Simulator {
	static const Scalar COLOR_EMPTY;
    static const Scalar COLOR_OCCUPIED;
	static const Scalar COLOR_POSITION;
	static const Scalar COLOR_TARGET;
    static const Scalar COLOR_SHADOW;
	static const Scalar COLOR_GRID;
    static const Scalar COLOR_SEPARATOR_LINE;
    
	int XSquares;
	int YSquares;	//in square count
	Size windowSize;
	void markSquare(int markType, Point2i square, bool isShadow);
    void clearColumn(int column);
    void drawShadow(Point2i square);
    void drawGrid();

public:
    static const int TYPE_AVOIDANCE;
    static const int TYPE_NAVIGATION;
    static const int TYPE_NONE;
    
    int simulatorType;
    float fov;  //Used only for avoidance simulator
    float depth;//Used only for avoidance simulator
    ObstacleScenario scenario;
    float squareRealSize;
    int squarePixelSize;
    Mat display;
    
    Simulator();
	Simulator(float width, float height, float squareSize, int type, int windowWidth);	//Use real world measures
	void runSimulation();
    void avoidanceSimulator(bool autoEraseColumns);
    void navigationSimulator();
    Mat drawScenario();
	virtual ~Simulator();
};

#endif /* SIMULATOR_H_ */
