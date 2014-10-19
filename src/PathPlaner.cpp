/*
 * PathPlaner.cpp
 *
 *  Created on: Sep 1, 2014
 *      Author: alejandro
 */

#include "PathPlaner.h"


const float PathPlaner::WAYPOINT_DIST = 0.3;
const float PathPlaner::MIN_DIST_FROM_OBSTACLE = 0.3;


PathPlaner::PathPlaner() {
	// TODO Auto-generated constructor stub
}

PathPlaner::~PathPlaner() {
	// TODO Auto-generated destructor stub
}

bool PathPlaner::findNavigationPath(ObstacleScenario scenario, Point2i initialLocation, Point2i targetLocation){

	return false; //While this section is not written
}

bool PathPlaner::findAvoidancePath(ObstacleScenario scenario, Mat &display, int squarePixelSize){
    vector<Point> Lcorners;
    vector<Point> Rcorners;
    vector<vector<int> > scen = scenario.scenario;
    for (unsigned int i = 0; i<scen.size(); i++) {
        for (unsigned int j = 0; j<scen.at(i).size(); j++) {
            int squareType = scen.at(i).at(j);
            Point squareLocation = Point(i, j);
            if (squareType == 1) {  //If the square is an obstacle
                
            }
        }
    }

    return false;
}

