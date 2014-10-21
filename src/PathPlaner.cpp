/*
 * PathPlaner.cpp
 *
 *  Created on: Sep 1, 2014
 *      Author: alejandro
 */

#include "PathPlaner.h"


const float PathPlaner::WAYPOINT_DIST = 0.3;
const float PathPlaner::MIN_DIST_FROM_OBSTACLE = 0.3;
const Scalar PathPlaner::COLOR_PATH = Scalar(30, 150, 30);
const Scalar PathPlaner::COLOR_PATH_LIMITS = Scalar(30, 30, 150);
const Scalar PathPlaner::COLOR_CORNERS = Scalar(30, 30, 150);


PathPlaner::PathPlaner() {
	// TODO Auto-generated constructor stub
}

PathPlaner::~PathPlaner() {
	// TODO Auto-generated destructor stub
}

bool PathPlaner::findNavigationPath(ObstacleScenario scenario, Point2i initialLocation, Point2i targetLocation){

	return false; //While this section is not written
}

float PathPlaner::findAvoidancePath(ObstacleScenario scenario, float initialCurveRadius, Mat &display, int squarePixelSize){
    //If the initial curve radius is greather than 1000, the curve is considered a straight line

    vector<vector<int> > scen = scenario.scenario;
    int scenX = (int)scen.size();
    int scenY = (int)scen.at(0).size();
    Point viewerLoc = Point(scenX/2, scenY);
    int pixelsPerMeter = ceil(squarePixelSize/scenario.squareSize);
    vector<float> goodRadius;
    float bestRadius = NULL; //A value of 0 is considered as null
    
    //First step: Check if keeping straight is posible///////////////////////////////////////////
    bool isFrontCleared = true;
    for (int i=0; i<ceil(MIN_DIST_FROM_OBSTACLE/scenario.squareSize); i++){
        for (int j=0; j<scenY; j++) {
            if (scen.at(viewerLoc.x+i).at(j) == 1) {
                isFrontCleared = false;
                break;
            }
            else if (scen.at(viewerLoc.x-i).at(j) == 1){
                isFrontCleared = false;
                break;
            }
        }
    }
    if (isFrontCleared) {      //Save and draw the straight path
        goodRadius.push_back(10000);    //A value greather than 1000 is considered a straight line
        int XOffset1 = ceil(viewerLoc.x*squarePixelSize-MIN_DIST_FROM_OBSTACLE*pixelsPerMeter);
        int XOffset2 = ceil(viewerLoc.x*squarePixelSize+MIN_DIST_FROM_OBSTACLE*pixelsPerMeter);
        line(display, Point(XOffset1, 0), Point(XOffset1, display.rows), COLOR_PATH_LIMITS, 1);
        line(display, Point(XOffset2, 0), Point(XOffset2, display.rows), COLOR_PATH_LIMITS, 1);
        line(display, Point(viewerLoc.x*squarePixelSize, 0), Point(viewerLoc.x*squarePixelSize, display.rows), COLOR_PATH, 3);
    }
    ////////////////////////////////////////////////////////////////////////////////////////////
    
    //TODO: Check curve paths
    
    /*for (int i = 1; i<Xsquares-1; i++) {
        for (int j = 1; j<Ysquares-1; j++) {
            int squareType = scen.at(i).at(j);
            int L_Neigbour[] = [scen.at(i-1).at(j-1), scen.at(i-1).at(j), scen.at(i-1).at(j+1)];
            int R_Neigbour[] = [scen.at(i+1).at(j-1), scen.at(i+1).at(j), scen.at(i+1).at(j+1)];
            Point currentSquareLocation = Point(i, j);
            if (squareType == 1) {  //If the square is an obstacle
                if(L_Neigbour[0] == 0 && L_Neigbour[1] == 0 && L_Neigbour[2] == 0){
                    float newAngle = atan2(i-Xcenter, Ysquares-j);
                    if (newAngle > L_lastAngle) {
                        Lcorners.pop_back();
                    }
                    Lcorners.push_back(currentSquareLocation);
                }
            }
        }
    }*/
    ///////provisional////////////
    bestRadius = goodRadius.at(0);
    //////////////////////////////
    return bestRadius;
}

