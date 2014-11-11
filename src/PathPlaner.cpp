/*
 * PathPlaner.cpp
 *
 *  Created on: Sep 1, 2014
 *      Author: alejandro
 */

#include "PathPlaner.h"


const float PathPlaner::WAYPOINT_DIST = 0.3;
const float PathPlaner::HALF_VEHICLE_WIDTH = 0.3;
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

void PathPlaner::drawAvoidancePaths(cv::Mat &display, vector<RadiusPair> radiusRanges, Point2f viewerLoc){
    
    for (unsigned int i = 0; i < radiusRanges.size(); i++) {
        float minRad = radiusRanges.at(i).radius1;
        float maxRad = radiusRanges.at(i).radius2;
        Point curveCenter;
        
        //Draw the minimum radius
        if (minRad == 0) continue;
        else if (minRad > 1000){
            line(display, Point(viewerLoc.x*pixelsPerMeter, 0), Point(viewerLoc.x*pixelsPerMeter, display.rows), Scalar(50, 200, 50), 2);
            continue;
        }
        curveCenter = Point((viewerLoc.x+minRad)*pixelsPerMeter, display.rows);
        minRad = abs(minRad);
        circle(display, curveCenter, minRad*pixelsPerMeter, Scalar(50, 50, 200), 2);
        cout << "Min rad " << i << ": " << minRad << endl;
        
        //Draw the maximum radius
        if (maxRad == 0) continue;
        else if (maxRad > 1000){
            line(display, Point(viewerLoc.x*pixelsPerMeter, 0), Point(viewerLoc.x*pixelsPerMeter, display.rows), Scalar(50, 200, 50), 2);
            continue;
        }
        curveCenter = Point((viewerLoc.x+maxRad)*pixelsPerMeter, display.rows);
        maxRad = abs(maxRad);
        circle(display, curveCenter, maxRad*pixelsPerMeter, Scalar(200, 50, 50), 2);
        cout << "Max rad " << i << ": " << maxRad << endl;
    }
}

/*****************************************************************************************
 * Calculate the radius of the smaller circumference tangent to both the initLoc and the *
 * tangentPoint.                                                                         *
 *****************************************************************************************/
float radiusForTangency(Point2f initLoc, Point2f tangentPoint){
    cout << "Tan point" << tangentPoint << endl;
    float relX = tangentPoint.x - initLoc.x;
    float relY = initLoc.y - tangentPoint.y;
    if (relX == 0) return 10000.0; //Values greather than 1000 are considered a straight path
    if (abs(relX) > relY) return -1; //Not possible to calculate a tangent curve
    float radiusSign = relX/abs(relX); //will be eigther 1 or -1
    cout << "Rel loc:  X = " << relX << "  |  Y = " << relY << endl;
    //calculate curve radius to tangency
    float alpha = atan(relY / abs(relX));
    float beta = M_PI/2-alpha;
    float cathetus = sqrt((relX*relX) + (relY*relY));
    return ((cathetus/abs(sin(beta)))/2)*radiusSign; //Negative because it is at the left side of the
}

/*******************************************************************
 * This function recursively finds the outher corner of a block.   *
 *******************************************************************/
void PathPlaner::neigbourExpand(Point currentSquare, vector<Corner> &corners){
    int x = currentSquare.x;
    int y = currentSquare.y;
    
    //Neighours are stored clockwise starting from top
    int neigbourType[] = {scen.at(x).at(y-1), scen.at(x+1).at(y-1), scen.at(x+1).at(y), scen.at(x+1).at(y+1), scen.at(x).at(y+1), scen.at(x-1).at(y+1), scen.at(x-1).at(y), scen.at(x-1).at(y-1)};

    if (neigbourType[0] == 0 && neigbourType[1] == 0 && neigbourType[2] == 0){  //Top-right corner is free
        Corner corner;
        corner.cornerLoc = Point2f((x+1)*squareSize, y*squareSize);
        corner.isTop = true;
    }
    if (neigbourType[2] == 0 && neigbourType[3] == 0 && neigbourType[4] == 0){  //Bottom-right corner is free
        Corner corner;
        corner.cornerLoc = Point2f((x+1)*squareSize, (y+1)*squareSize);
        corner.isTop = false;
    }
    if (neigbourType[4] == 0 && neigbourType[5] == 0 && neigbourType[6] == 0){  //Bottom-left corner is free
        Corner corner;
        corner.cornerLoc = Point2f((x)*squareSize, (y+1)*squareSize);
        corner.isTop = false;
    }
    if (neigbourType[6] == 0 && neigbourType[7] == 0 && neigbourType[0] == 0){  //Top-left corner is free
        Corner corner;
        corner.cornerLoc = Point2f((x+1)*squareSize, (y+1)*squareSize);
        corner.isTop = true;
    }
    
    if (neigbourType[0] == 1)   neigbourExpand(Point(x  , y-1), corners);
    if (neigbourType[1] == 1)   neigbourExpand(Point(x+1, y-1), corners);
    if (neigbourType[2] == 1)   neigbourExpand(Point(x+1,   y), corners);
    if (neigbourType[3] == 1)   neigbourExpand(Point(x+1, y+1), corners);
    if (neigbourType[4] == 1)   neigbourExpand(Point(x  , y+1), corners);
    if (neigbourType[5] == 1)   neigbourExpand(Point(x-1, y+1), corners);
    if (neigbourType[6] == 1)   neigbourExpand(Point(x-1,   y), corners);
    if (neigbourType[7] == 1)   neigbourExpand(Point(x-1, y-1), corners);
}


/*************************************************************************************
 * The following function finds the curve radiuses for each block that would make    *
 * the mobile platform to avoid collision with the block.                            *
 *************************************************************************************/
RadiusPair findMinMaxRadius(vector<Corner> corners, Point2f initLoc){
    //float minMaxRad[] = {0, 0};
    float rad1 = 0; //This is radius on top of the block if possible
    float rad2 = 0;
    bool noMinRad = false; //This is set to true if it is not possible to generate a circular path on top of the block
    for (unsigned int i = 0; i < corners.size(); i++) {
        float newRadius;
        ///////////////////////////////////////////////////////////////
        //If the corner is on top of the block and it is still unproven that a path on top of it is possible...
        if (corners.at(i).isTop && !noMinRad) {
            if (corners.at(i).cornerLoc.x - initLoc.x < 0){
                newRadius = radiusForTangency(Point2f(initLoc.x-PathPlaner::HALF_VEHICLE_WIDTH), corners.at(i).cornerLoc);
                newRadius += PathPlaner::HALF_VEHICLE_WIDTH;
            }
            else {
                newRadius = radiusForTangency(Point2f(initLoc.x+PathPlaner::HALF_VEHICLE_WIDTH), corners.at(i).cornerLoc);
                newRadius -= PathPlaner::HALF_VEHICLE_WIDTH;
            }
            
            if ((rad1 < 0 && newRadius > 0) || (rad1 > 0 && newRadius < 0)) {
                    //If the radiuses have different sign, then it is not possible to have a min rad
                    noMinRad = true;
                    continue;
            }
            if(abs(newRadius) > abs(rad1))
                rad1 = newRadius;
        }
        ///////////////////////////////////////////////////////////////
        else if (!corners.at(i).isTop){
            if (corners.at(i).cornerLoc.x - initLoc.x < 0){
                newRadius = radiusForTangency(Point2f(initLoc.x+PathPlaner::HALF_VEHICLE_WIDTH), corners.at(i).cornerLoc);
                newRadius -= PathPlaner::HALF_VEHICLE_WIDTH;
            }
            else {
                newRadius = radiusForTangency(Point2f(initLoc.x-PathPlaner::HALF_VEHICLE_WIDTH), corners.at(i).cornerLoc);
                newRadius += PathPlaner::HALF_VEHICLE_WIDTH;
            }
            
            if (!noMinRad) { //If there is a minimum and maximum radius
                //First check if the current corner unproves this
                if ((rad2 < 0 && newRadius > 0) || (rad2 > 0 && newRadius < 0)) {
                    //If the radiuses have different sign, then it is not possible to have a min rad
                    noMinRad = true;
                    rad1 = newRadius;
                }
                else if (abs(newRadius) < abs(rad2))
                    rad1 = newRadius;
            }
            else if (noMinRad){
                if ((rad1 < 0 && newRadius < 0) || (rad1 > 0 && newRadius > 0)) {
                    if (abs(newRadius) < abs(rad1))
                        rad1 = newRadius;
                }
                else if ((rad2 < 0 && newRadius < 0) || (rad2 > 0 && newRadius > 0)) {
                    if (abs(newRadius) < abs(rad2))
                        rad2 = newRadius;
                }
            }
        }
    }
    RadiusPair radiuses;
    radiuses.radius1 = rad1;
    radiuses.radius2 = rad2;
    return radiuses;
}


/*************************************************************
 * Handy method for comparing the sign of two float values.  *
 *************************************************************/
bool areDifferentSign(float val1, float val2){
    if (val1 + val2 != abs(val1) + abs(val2)) {
        return true;
    }
    else return false;
}


/******************************************************************************
 * Method for inserting radius pairs into a vector. They are sorted according *
 * to their "radius1" component from minimum to maximum.                      *
 ******************************************************************************/
void sortedInsert(vector<RadiusPair> sortedPairs, RadiusPair newRadiusPair){
    for (unsigned int j = 0; j < sortedPairs.size(); j++) {
        if (newRadiusPair.radius2 > sortedPairs.at(j).radius2) {
            if (j < sortedPairs.size()-1) {
                if (newRadiusPair.radius2 < sortedPairs.at(j+1).radius2) {
                    vector<RadiusPair>::iterator it = sortedPairs.begin() + j + 1;
                    sortedPairs.insert(it, newRadiusPair);
                    return;
                }
            }
            else {
                sortedPairs.push_back(newRadiusPair);
                return;
            }
        }
        else if (newRadiusPair.radius2 < sortedPairs.at(j).radius2){
            if (j == 0) {
                vector<RadiusPair>::iterator it = sortedPairs.begin();
                sortedPairs.insert(it, newRadiusPair);
                return;
            }
        }
    }
}

/*********************************************************************************************
 * This function reveives the radiuses to avoid each block and selects the ones that do not  *
 * intersect radiuses from other blocks. This means to ensure that the robot fits in a path. *
 *********************************************************************************************/

vector<RadiusPair> findValidRadiusesRanges(vector<RadiusPair> tangentRadiusesPairs){
    int totalRadiusPairs = (signed int) tangentRadiusesPairs.size();
    vector<RadiusPair> sortedLeftRadiuses;
    vector<RadiusPair> sortedRightRadiuses;
    vector<RadiusPair> validPairs;
    RadiusPair noMinRadCase;
    for (int i = 0; i < totalRadiusPairs; i++) {
        RadiusPair currentRadPair = tangentRadiusesPairs.at(i);
        if (areDifferentSign(currentRadPair.radius1, currentRadPair.radius2)) {
            noMinRadCase = currentRadPair;
            continue;
        }
        if (currentRadPair.radius1 < 0)
            sortedInsert(sortedLeftRadiuses, currentRadPair);
        else if (currentRadPair.radius1 > 0)
            sortedInsert(sortedRightRadiuses, currentRadPair);
    }
    
    if (noMinRadCase.radius1 != 0 && noMinRadCase.radius2 != 0) {
        RadiusPair pair1, pair2;
        if (noMinRadCase.radius1 < 0) {
            pair1.radius1 = noMinRadCase.radius1;
            pair2.radius1 = noMinRadCase.radius2;
        }
        else {
            pair1.radius1 = noMinRadCase.radius2;
            pair2.radius1 = noMinRadCase.radius1;
        }
        sortedLeftRadiuses.push_back(pair1);
        sortedRightRadiuses.push_back(pair2);
    }
    else {
        RadiusPair pair;
        pair.radius1 = sortedLeftRadiuses.at(sortedLeftRadiuses.size()-1).radius1;
        pair.radius2 = sortedRightRadiuses.at(sortedRightRadiuses.size()-1).radius1;
        validPairs.push_back(pair);
    }
    
    for (unsigned int i = 0; i < sortedLeftRadiuses.size(); i++) {
        if (i < sortedLeftRadiuses.size()-1) {
            if (abs(sortedLeftRadiuses.at(i).radius2) < abs(sortedLeftRadiuses.at(i+1).radius2)) {
                RadiusPair newValidRadiusRange;
                newValidRadiusRange.radius1 = sortedLeftRadiuses.at(i).radius1;
                newValidRadiusRange.radius2 = sortedLeftRadiuses.at(i+1).radius2;
                validPairs.push_back(newValidRadiusRange);
            }
        }
    }
    for (unsigned int i = 0; i < sortedRightRadiuses.size(); i++) {
        if (i < sortedRightRadiuses.size()-1) {
            if (abs(sortedRightRadiuses.at(i).radius2) < abs(sortedRightRadiuses.at(i+1).radius2)) {
                RadiusPair newValidRadiusRange;
                newValidRadiusRange.radius1 = sortedRightRadiuses.at(i).radius1;
                newValidRadiusRange.radius2 = sortedRightRadiuses.at(i+1).radius2;
                validPairs.push_back(newValidRadiusRange);
            }
        }
    }
    return validPairs;
}

float PathPlaner::findAvoidancePath(ObstacleScenario scenario, float initialCurveRadius, Mat &display, int squarePixelSize){
    //If the initial curve radius is greather than 1000, the curve is considered a straight line
    pixelsPerMeter = (int)squarePixelSize/scenario.squareSize;
    squareSize = scenario.squareSize;
    scen = scenario.scenario;
    int scenX = (int)scen.size();
    int scenY = (int)scen.at(0).size();
    Point viewerLoc = Point(scenX/2, scenY);
    Point2f viewerLocMeters = Point2f(viewerLoc.x*squareSize, viewerLoc.y*squareSize);
    vector<RadiusPair> tangentRadiuses;
    
    for (int i = 1; i<scenX-1; i++) {
        for (int j = 1; j<scenY-1; j++) {
            if (scen.at(i).at(j) == 1) {
                vector<Corner> corners;
                neigbourExpand(Point(i, j), corners);
                RadiusPair minMaxRads = findMinMaxRadius(corners, viewerLocMeters);
                tangentRadiuses.push_back(minMaxRads);
            }
        }
    }
    
    
    vector<RadiusPair> goodRadiusRanges = findValidRadiusesRanges(tangentRadiuses);
    
    drawAvoidancePaths(display, goodRadiusRanges, viewerLocMeters);

    ///////provisional////////////
    float bestRadius = goodRadiusRanges.at(0).radius1;
    //////////////////////////////
    return bestRadius;
}
