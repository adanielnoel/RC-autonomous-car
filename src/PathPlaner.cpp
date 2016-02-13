/*
 * PathPlaner.cpp
 *
 *  Created on: Jul 26, 2014
 *     Author: Alejandro Daniel Noel
 *     Page: http://futuretechmaker.com
 */

#include "PathPlaner.h"


const float PathPlaner::WAYPOINT_DIST = 0.3;
const float PathPlaner::MINIMUM_RADIUS = 0.1;
const float PathPlaner::HALF_VEHICLE_WIDTH = 0.28   ;
const Scalar PathPlaner::COLOR_PATH = Scalar(30, 150, 30);
const Scalar PathPlaner::COLOR_PATH_LIMITS = Scalar(0, 150, 33);
const Scalar PathPlaner::COLOR_CORNERS = Scalar(30, 30, 150);

/*************************************************************
 * Handy method for comparing the sign of two float values.  *
 *************************************************************/
bool areDifferentSign(float val1, float val2){
    return val1*val2<0?true:false;
}

PathPlaner::PathPlaner() {
	// TODO Auto-generated constructor stub
}

PathPlaner::~PathPlaner() {
	// TODO Auto-generated destructor stub
}

void PathPlaner::drawAvoidancePaths(cv::Mat &display, vector<RadiusPair> radiusRanges, Point2f viewerLoc){
    
    for (unsigned int i = 0; i < radiusRanges.size(); i++) {
        float minRad = radiusRanges.at(i).radius1;
        float maxRad = radiusRanges.at(i).radius2;
        //cout << "Min rad " << i << ": " << minRad << endl;
        //cout << "Max rad " << i << ": " << maxRad << endl;
        Point curveCenter;
        //Scalar curveColor = Scalar((i%2)*200,((i*2 % 3)*100)%255,(100+30*i)%255);
        Scalar curveColor = COLOR_PATH_LIMITS;
        
        //Draw the minimum radius
        if (minRad == 0) continue;
        curveCenter = Point((viewerLoc.x+minRad)*pixelsPerMeter, display.rows);
        minRad = abs(minRad);
        circle(display, curveCenter, minRad*pixelsPerMeter, curveColor, 2);
        
        //Draw the maximum radius
        if (maxRad == 0) continue;
        curveCenter = Point((viewerLoc.x+maxRad)*pixelsPerMeter, display.rows);
        maxRad = abs(maxRad);
        circle(display, curveCenter, maxRad*pixelsPerMeter, curveColor, 2);
    }
}

/*****************************************************************************************
 * Calculate the radius of the smaller circumference tangent to both the initLoc and the *
 * tangentPoint.                                                                         *
 * 28/10/2014: Debugged and working as expected.                                         *
 *****************************************************************************************/
/////////////////////////////////HERE!!!!
float radiusForTangency(Point2f tangentPoint){
    float relX = tangentPoint.x;
    float relY = tangentPoint.y;
    if (relX == 0) return 10000.0; //Values greather than 1000 are considered a straight path
    float radiusSign = relX/abs(relX); //will be eigther 1 or -1
    //calculate curve radius to tangency
    float alpha = atan(relY / abs(relX));
    float beta = M_PI/2-alpha;
    float cathetus = sqrt((relX*relX) + (relY*relY));
    return ((cathetus/abs(sin(beta)))/2)*radiusSign;
}

/*******************************************************************
 * This function recursively finds the outher corner of a block.   *
 * 12/11/2014: Debugged and working as expected                    *
 *******************************************************************/
void PathPlaner::neighborExpand(Point currentSquare, Point viewerLoc, vector<Corner> &corners){
    int maxX = (unsigned)scen.size()-1;
    int maxY = (unsigned)scen.at(0).size()-1;
    int x = currentSquare.x;
    int y = currentSquare.y;
    int relX = currentSquare.x - viewerLoc.x;
    int relY = currentSquare.y - viewerLoc.y;
    //Mark the current square as already visited
    scen.at(x).at(y) = -1;
    
    //Define an array to store the neighbor types and variables to store neighbor location
    int neighborType[8];
    int nX, nY;
    
    //Top neighbor
    nX = x; nY = y-1;
    neighborType[0] = ((nX<0 || nY<0)||(nX>maxX || nY>maxY))? 0 : scen.at(nX).at(nY);
    //Top-right neighbor
    nX = x+1, nY = y-1;
    neighborType[1] = ((nX<0 || nY<0)||(nX>maxX || nY>maxY))? 0 : scen.at(nX).at(nY);
    //Right neighbor
    nX = x+1; nY = y;
    neighborType[2] = ((nX<0 || nY<0)||(nX>maxX || nY>maxY))? 0 : scen.at(nX).at(nY);
    //Bottom-right neighbor
    nX = x+1; nY = y+1;
    neighborType[3] = ((nX<0 || nY<0)||(nX>maxX || nY>maxY))? 0 : scen.at(nX).at(nY);
    //Bottom neighbor
    nX = x; nY = y+1;
    neighborType[4] = ((nX<0 || nY<0)||(nX>maxX || nY>maxY))? 0 : scen.at(nX).at(nY);
    //Bottom-left neighbor
    nX = x-1; nY = y+1;
    neighborType[5] = ((nX<0 || nY<0)||(nX>maxX || nY>maxY))? 0 : scen.at(nX).at(nY);
    //Left neighbor
    nX = x-1; nY = y;
    neighborType[6] = ((nX<0 || nY<0)||(nX>maxX || nY>maxY))? 0 : scen.at(nX).at(nY);
    //Top-left neighbor
    nX = x-1; nY = y-1;
    neighborType[7] = ((nX<0 || nY<0)||(nX>maxX || nY>maxY))? 0 : scen.at(nX).at(nY);

    if (neighborType[0] == 0 && neighborType[1] == 0 && neighborType[2] == 0){  //Top-right corner is free
        Corner corner;
        corner.cornerLoc = Point2f((relX+1)*squareSize, abs(relY*squareSize));
        corner.isTop = true;
        corner.isLeft = false;
        corners.push_back(corner);
    }
    if (neighborType[2] == 0 && neighborType[3] == 0 && neighborType[4] == 0){  //Bottom-right corner is free
        Corner corner;
        corner.cornerLoc = Point2f((relX+1)*squareSize, abs((relY+1)*squareSize));
        corner.isTop = false;
        corner.isLeft = false;
        corners.push_back(corner);
    }
    if (neighborType[4] == 0 && neighborType[5] == 0 && neighborType[6] == 0){  //Bottom-left corner is free
        Corner corner;
        corner.cornerLoc = Point2f(relX*squareSize, abs((relY+1)*squareSize));
        corner.isTop = false;
        corner.isLeft = true;
        corners.push_back(corner);
    }
    if (neighborType[6] == 0 && neighborType[7] == 0 && neighborType[0] == 0){  //Top-left corner is free
        Corner corner;
        corner.cornerLoc = Point2f(relX*squareSize, abs(relY*squareSize));
        corner.isTop = true;
        corner.isLeft = true;
        corners.push_back(corner);
    }
    
    if (neighborType[0] == 1)   neighborExpand(Point(x  , y-1), viewerLoc, corners);
    if (neighborType[1] == 1)   neighborExpand(Point(x+1, y-1), viewerLoc, corners);
    if (neighborType[2] == 1)   neighborExpand(Point(x+1,   y), viewerLoc, corners);
    if (neighborType[3] == 1)   neighborExpand(Point(x+1, y+1), viewerLoc, corners);
    if (neighborType[4] == 1)   neighborExpand(Point(x  , y+1), viewerLoc, corners);
    if (neighborType[5] == 1)   neighborExpand(Point(x-1, y+1), viewerLoc, corners);
    if (neighborType[6] == 1)   neighborExpand(Point(x-1,   y), viewerLoc, corners);
    if (neighborType[7] == 1)   neighborExpand(Point(x-1, y-1), viewerLoc, corners);
}


/****************************************************************************************
 *   FUNCTION: findMinMaxRadius                                                         *
 * Finds the curve radiuses that would make the vehicle pass tangentially to a block.   *
 * There are always two possible radiuses, normally one that passes above the block and *
 * one that passes below. But in the case that the block is in front of the vehicle's   *
 * bounding area, both possible curves are below the block. In this case, the variable  *
 * noCurveAvoveBlock is set to true. This implies that a straight path is not possible. *
 *                                                                                      *
 * 25/11/2014: Debugged and working with some problems when a block is in front.        *
 ****************************************************************************************/

RadiusPair findMinMaxRadius(vector<Corner> corners, Point2f initLoc){
    bool noCurveAboveBlock = false; //This is set to true if it is not possible to generate a circular path above the block.
    float lastCornerX = corners.at(0).cornerLoc.x;
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////STEP 1: Find out if a path above the block is possible. /////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    for (unsigned int i = 0; i < corners.size(); i++) {
        if (areDifferentSign(lastCornerX, corners.at(i).cornerLoc.x)) {
            //If the block crosses in front of the vehicle, its relative x positions change their sign
            noCurveAboveBlock = true;
            break;
        }
        if (corners.at(i).cornerLoc.x > -PathPlaner::HALF_VEHICLE_WIDTH && corners.at(i).cornerLoc.x < PathPlaner::HALF_VEHICLE_WIDTH) {
            //If the block has one of its corners intercepting the vehicle's width in front of it.
            noCurveAboveBlock = true;
            break;
        }
        lastCornerX = corners.at(i).cornerLoc.x;
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////STEP 2: Assign initial values to the two tangent radiuses.        /////////////////
    ///////////////////        rad2 is always bottom and rad1 can be both top or bottom. /////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    float rad1 = noCurveAboveBlock == true ? 10000 : 0; //This is radius on top of the block if possible, otherwise it is the one at the right of the block below it
    float rad2 = 10000;  //Set it to a big number because we are taking the smallest one and we compare new radiuses with this value

    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////STEP 3: Compute the two trajectory radiuses with which the vehicle  ////////////////
    ///////////////////        passes tangent to the block. The trajectory is given from   ////////////////
    ///////////////////        the center of the vehicle, but the tangent would be from    ////////////////
    ///////////////////        the side of the vehicle. The algorithm conveniently adds or ////////////////
    ///////////////////        subtracts half of the vehicle's width to center the path.   ////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    for (unsigned int i = 0; i < corners.size(); i++) {
        Corner currentCorner = corners.at(i);
        float newRadius;    //The radius of the trajectory from the center of the vehicle.
        ////////Discard the corner if it is on top of the corner but noCurveAboveBlock is checked as true///////
        if (currentCorner.isTop && noCurveAboveBlock)
            continue;
        
        if (currentCorner.isTop) {
            if (currentCorner.cornerLoc.x < -PathPlaner::HALF_VEHICLE_WIDTH) {
                newRadius = radiusForTangency(Point2f(currentCorner.cornerLoc.x + PathPlaner::HALF_VEHICLE_WIDTH, currentCorner.cornerLoc.y));
                newRadius -= PathPlaner::HALF_VEHICLE_WIDTH;    //As the radius returned will be negative, this increases its absolute value.
                if (abs(newRadius) > abs(rad1))
                    rad1 = newRadius; //Always take the greather radius, to ensure it is only tangent at one point.
            }
            else if (currentCorner.cornerLoc.x > PathPlaner::HALF_VEHICLE_WIDTH){
                newRadius = radiusForTangency(Point2f(currentCorner.cornerLoc.x - PathPlaner::HALF_VEHICLE_WIDTH, currentCorner.cornerLoc.y));
                newRadius += PathPlaner::HALF_VEHICLE_WIDTH;
                if (abs(newRadius) > abs(rad1))
                    rad1 = newRadius; //Always take the greather radius, to ensure it is only tangent at one point.
            }
        }
        if (!currentCorner.isTop) {
            if (currentCorner.cornerLoc.x < -PathPlaner::HALF_VEHICLE_WIDTH){
                newRadius = radiusForTangency(Point2f(currentCorner.cornerLoc.x - PathPlaner::HALF_VEHICLE_WIDTH, currentCorner.cornerLoc.y));
                newRadius += PathPlaner::HALF_VEHICLE_WIDTH; //As the returned radius will be negative, this decreases its absolute value.
                if (abs(newRadius) < abs(rad2)) {
                    rad2 = newRadius; //Always take the smaller radius, to ensure it is only tangent at one point.
                }
            }
            else if (currentCorner.cornerLoc.x > PathPlaner::HALF_VEHICLE_WIDTH){
                newRadius = radiusForTangency(Point2f(currentCorner.cornerLoc.x + PathPlaner::HALF_VEHICLE_WIDTH, currentCorner.cornerLoc.y));
                newRadius -= PathPlaner::HALF_VEHICLE_WIDTH; //As the returned radius will be positive, this decreases its absolute value.
                if (!noCurveAboveBlock) {
                    if (abs(newRadius) < abs(rad2))
                        rad2 = newRadius; //Always take the smaller radius, to ensure it is only tangent at one point.
                }
                else if (noCurveAboveBlock) {
                    if (abs(newRadius) < abs(rad1))
                        rad1 = newRadius; //Always take the smaller radius, to ensure it is only tangent at one point.
                }
            }
            else if (currentCorner.cornerLoc.x >= -PathPlaner::HALF_VEHICLE_WIDTH && currentCorner.cornerLoc.x <= PathPlaner::HALF_VEHICLE_WIDTH){
                if (currentCorner.isLeft) {
                    newRadius = radiusForTangency(Point2f(currentCorner.cornerLoc.x - PathPlaner::HALF_VEHICLE_WIDTH, currentCorner.cornerLoc.y));
                    newRadius += PathPlaner::HALF_VEHICLE_WIDTH;
                    if (abs(newRadius) < abs(rad2))
                        rad2 = newRadius;
                }
                else{
                    newRadius = radiusForTangency(Point2f(currentCorner.cornerLoc.x + PathPlaner::HALF_VEHICLE_WIDTH, currentCorner.cornerLoc.y));
                    newRadius -= PathPlaner::HALF_VEHICLE_WIDTH;
                    if (abs(newRadius) < abs(rad1))
                        rad1 = newRadius;
                }
            }
        }
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////STEP 4: Pack both radiuses into a RadiusPair object and return it. ////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    RadiusPair radiuses;
    radiuses.radius1 = rad1;
    radiuses.radius2 = rad2;
    return radiuses;
}


bool radiusContainedInRange(float radius, RadiusPair range){
    float minRad = abs(range.radius1) < abs(range.radius2) ? abs(range.radius1) : abs(range.radius2);
    float maxRad = abs(range.radius1) < abs(range.radius2) ? abs(range.radius2) : abs(range.radius1);

    if (areDifferentSign(range.radius1, range.radius2)) {
        if (abs(radius) > minRad) return true;
    }
    else if (!areDifferentSign(radius, range.radius1) && (abs(radius) > minRad && abs(radius) < maxRad)) return true;
    
    return false;
}

/*====================================================================================*\
|  Returns the left-most and right-most radiuses among a collection of radius pairs    |
\*====================================================================================*/

RadiusPair fusionRadiusRanges(vector<RadiusPair> radRanges) {
    float leftRad =   0.00001;    // Initial value used for starting comparison
    float rightRad = -0.00001;         // Initial value used for starting comparison
    
    for (int i = 0; i < radRanges.size(); i++) {
        float r1 = radRanges.at(i).radius1;
        float r2 = radRanges.at(i).radius2;
        if      (leftRad > 0 && r1 < 0) leftRad = r1;
        else if (leftRad > 0 && r2 < 0) leftRad = r2;
        if      (rightRad < 0 && r1 > 0) rightRad = r1;
        else if (rightRad < 0 && r2 > 0) rightRad = r2;
        
        if (areDifferentSign(leftRad, rightRad)) {
            if (r1 < 0 && abs(r1) < abs(leftRad)) leftRad = r1;
            if (r2 < 0 && abs(r2) < abs(leftRad)) leftRad = r2;
            if (r1 > 0 && abs(r1) < abs(rightRad)) rightRad = r1;
            if (r2 > 0 && abs(r2) < abs(rightRad)) rightRad = r2;
        }
        else if (leftRad < 0){  // rightRad will also be < 0
            if (abs(r1) < abs(leftRad)) leftRad = r1;
            if (abs(r2) < abs(leftRad)) leftRad = r2;
            if (abs(r1) > abs(rightRad)) rightRad = r1;
            if (abs(r2) > abs(rightRad)) rightRad = r2;
        }
        else if (leftRad > 0) {  // rightRad will also be > 0
            if (abs(r1) > abs(leftRad)) leftRad = r1;
            if (abs(r2) > abs(leftRad)) leftRad = r2;
            if (abs(r1) < abs(rightRad)) rightRad = r1;
            if (abs(r2) < abs(rightRad)) rightRad = r2;
        }
    }
    RadiusPair returnPair;
    returnPair.radius1 = leftRad; returnPair.radius2 = rightRad;
    return returnPair;
}

/*===========================================================================================*
 * FUNCTION: findValidRadiusesRanges                                                         *
 * This function receives the radiuses to avoid each block and selects the ones that do not  *
 * intersect radiuses from other blocks. This means to ensure that the robot fits in a path. *
 * 25/11/2014: Debugged and working as expected.                                             *
 *===========================================================================================*/

vector<RadiusPair> findValidRadiusesRanges(vector<RadiusPair> &tangentRadiusesPairs){
    vector<RadiusPair> validRanges;
    vector<RadiusPair> trp = tangentRadiusesPairs;
    
    for (int i = 0; i < trp.size(); i++) {
        vector<RadiusPair> interceptedRanges;
        vector<RadiusPair> temp_trp;
        for (int j = 0; j < trp.size(); j++) {
            if (j == i) continue;
            
            if (radiusContainedInRange(trp.at(i).radius1, trp.at(j))) {
                interceptedRanges.push_back(trp.at(j));
            }
            else if (radiusContainedInRange(trp.at(i).radius2, trp.at(j))) {
                interceptedRanges.push_back(trp.at(j));
            }
            else temp_trp.push_back(trp.at(j));
        }
        
        if (!interceptedRanges.empty()) {
            interceptedRanges.push_back(trp.at(i));
            vector<RadiusPair> new_trp;
            new_trp.push_back(fusionRadiusRanges(interceptedRanges));
            new_trp.insert(new_trp.end(), temp_trp.begin(), temp_trp.end());
            trp = new_trp;
        }
    }
    return trp;
}



float PathPlaner::findAvoidancePath(ObstacleScenario &scenario, float initialCurveRadius, Mat &display, int squarePixelSize){
    //If the initial curve radius is greather than 1000, the curve is considered a straight line
    pixelsPerMeter = (int)squarePixelSize/scenario.squareSize;
    squareSize = scenario.squareSize;
    scen = scenario.scenario;
    int scenWidth = (int)scen.size();
    int scenDepth = (int)scen.at(0).size();
    Point viewerLoc = Point(scenWidth/2, scenDepth);
    Point2f viewerLocMeters = Point2f(viewerLoc.x*squareSize, viewerLoc.y*squareSize);
    vector<RadiusPair> tangentRadiuses;
    
    for (int i = 0; i<scenWidth; i++) {
        for (int j = 0; j<scenDepth; j++) {
            if (scen.at(i).at(j) == 1) {
                vector<Corner> corners;
                neighborExpand(Point(i, j), viewerLoc, corners); //Corner location is relative to viewer's location
                if (corners.size() == 0) cout << "ERROR: no corners detected!" << endl;
                RadiusPair minMaxRads = findMinMaxRadius(corners, viewerLocMeters);
                tangentRadiuses.push_back(minMaxRads);
                ///////////Drawing code./////////////////
                /*Point2f curveCenter = Point((int)display.cols/2+minMaxRads.radius1*pixelsPerMeter, display.rows);
                float maxRad = abs(minMaxRads.radius1);
                circle(display, curveCenter, maxRad*pixelsPerMeter, Scalar(200, 50, 50), 2);
                curveCenter = Point((int)display.cols/2+minMaxRads.radius2*pixelsPerMeter, display.rows);
                maxRad = abs(minMaxRads.radius2);
                circle(display, curveCenter, maxRad*pixelsPerMeter, Scalar(200, 50, 50), 2);*/
                ////////////////////////////////////////
            }
        }
    }
    if (tangentRadiuses.empty()) {
        cout << "No obstacles where detected" << endl;
        return 0.0;
    }
  // imshow("Display", display);
  //  waitKey(0);
  //  return 0;
    
    vector<RadiusPair> goodRadiusRanges = findValidRadiusesRanges(tangentRadiuses);
    
    drawAvoidancePaths(display, goodRadiusRanges, viewerLocMeters);

    ///////provisional////////////
    float bestRadius = goodRadiusRanges.at(0).radius1;
    //////////////////////////////
    return bestRadius;
}
