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
    float squareSize = scenario.squareSize;
    vector<vector<int> > scen = scenario.scenario;
    int scenX = (int)scen.size();
    int scenY = (int)scen.at(0).size();
    Point viewerLoc = Point(scenX/2, scenY);
    Point2f viewerLocMeters = Point2f(viewerLoc.x*squareSize, viewerLoc.y*squareSize);
    int pixelsPerMeter = ceil(squarePixelSize/scenario.squareSize);
    vector<float> goodMinRadiusRanges;
    vector<float> goodMaxRadiusRanges;
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
        //goodMinRadiusRanges.push_back(1000);    //A value greather than 1000 is considered a straight line
        goodMaxRadiusRanges.push_back(10000);    //A value greather than 1000 is considered a straight line
        int XOffset1 = ceil(viewerLoc.x*squarePixelSize-MIN_DIST_FROM_OBSTACLE*pixelsPerMeter);
        int XOffset2 = ceil(viewerLoc.x*squarePixelSize+MIN_DIST_FROM_OBSTACLE*pixelsPerMeter);
        line(display, Point(XOffset1, 0), Point(XOffset1, display.rows), COLOR_PATH_LIMITS, 1);
        line(display, Point(XOffset2, 0), Point(XOffset2, display.rows), COLOR_PATH_LIMITS, 1);
        line(display, Point(viewerLoc.x*squarePixelSize, 0), Point(viewerLoc.x*squarePixelSize, display.rows), COLOR_PATH, 3);
    }
    ////////////////////////////////////////////////////////////////////////////////////////////
    
    //TODO: Check curve paths
    float Xcenter = scenX/2;
    
    for (int i = 1; i<scenX-1; i++) {
        for (int j = 1; j<scenY-1; j++) {
            Point sqLoc = Point(i, j);
            int squareType = scen.at(i).at(j);
            int L_Neigbour[] = {scen.at(i-1).at(j-1), scen.at(i-1).at(j), scen.at(i-1).at(j+1)};
            int R_Neigbour[] = {scen.at(i+1).at(j-1), scen.at(i+1).at(j), scen.at(i+1).at(j+1)};
            
            if (squareType == 1) {  //If the square is an obstacle
                if(L_Neigbour[0] == 0 && L_Neigbour[1] == 0 && L_Neigbour[2] == 0){ //If it has no left neighbours

                    Point2f cornerRelativePos;
                    
                    if (i <= Xcenter) {
                        cornerRelativePos = Point2f(i*squareSize-viewerLocMeters.x, abs(viewerLocMeters.y-(j+1)*squareSize));  //In meters
                    }
                    else if (i > Xcenter) {
                        cornerRelativePos = Point2f(i*squareSize-viewerLocMeters.x, abs(viewerLocMeters.y-j*squareSize));  //In meters
                    }
                    //CRP_TLB : Corner relative pose to left boundary
                    Point2f CRP_TLB = cornerRelativePos - Point2f(MIN_DIST_FROM_OBSTACLE, 0);
                    
                    cout << "CRP: " << CRP_TLB << endl;
                    /*
                     *Check if the angle from the left path limit is greater than 45º,
                     *otherwise would not be possible to do a circular path.
                     */
                    if(abs(CRP_TLB.y) >= abs(CRP_TLB.x)){
                        //calculate curve radius to tangency
                        float alpha = atan(CRP_TLB.y / abs(CRP_TLB.x));
                        float beta = M_PI/2-alpha;
                        float cathetus = sqrt((CRP_TLB.x*CRP_TLB.x) + (CRP_TLB.y*CRP_TLB.y));
                        cout << sqrt(4) << endl;
                        float radius = (cathetus/abs(sin(beta)))/2; //Negative because it is at the left side of the viewer
                        goodMinRadiusRanges.push_back(0);
                        if (i <= Xcenter)
                            goodMaxRadiusRanges.push_back(-(radius));
                        else if (i > Xcenter)
                            goodMaxRadiusRanges.push_back(radius);
                    }
                }
                else if(L_Neigbour[0] == 0 && L_Neigbour[1] == 0 && L_Neigbour[2] == 0){ //If it has no right neighbours
                    
                }
            }
        }
    }

    for (unsigned int i=0; i<goodMaxRadiusRanges.size(); i++) {
        float goodRadius = goodMaxRadiusRanges.at(i);
        cout << "Radius: " << goodRadius << endl;
        if (goodRadius < 0) {
            goodRadius *= -1;
            Point curveCenter = Point((Xcenter*squareSize+(MIN_DIST_FROM_OBSTACLE-goodRadius))*pixelsPerMeter,display.rows);
            circle(display, curveCenter, goodRadius*pixelsPerMeter, COLOR_PATH_LIMITS, 2);
            circle(display, curveCenter, (goodRadius-2*MIN_DIST_FROM_OBSTACLE)*pixelsPerMeter, COLOR_PATH_LIMITS, 2);
        }
        else {
            Point curveCenter = Point((Xcenter*squareSize+(MIN_DIST_FROM_OBSTACLE+goodRadius))*pixelsPerMeter, display.rows);
            circle(display, curveCenter, goodRadius*pixelsPerMeter, COLOR_PATH_LIMITS, 2);
            circle(display, curveCenter, (goodRadius+2*MIN_DIST_FROM_OBSTACLE)*pixelsPerMeter, COLOR_PATH_LIMITS, 2);
        }
    }
    ///////provisional////////////
    bestRadius = goodMaxRadiusRanges.at(0);
    //////////////////////////////
    return bestRadius;
}

