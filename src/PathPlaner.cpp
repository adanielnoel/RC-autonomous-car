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
        Scalar curveColor = Scalar((i%2)*200,((i*2 % 3)*100)%255,(100+30*i)%255);
        //Draw the minimum radius
        if (minRad == 0) continue;
        else if (minRad > 1000){
            line(display, Point(viewerLoc.x*pixelsPerMeter, 0), Point(viewerLoc.x*pixelsPerMeter, display.rows), curveColor, 2);
            continue;
        }
        curveCenter = Point((viewerLoc.x+minRad)*pixelsPerMeter, display.rows);
        minRad = abs(minRad);
        circle(display, curveCenter, minRad*pixelsPerMeter, curveColor, 2);
        cout << "Min rad " << i << ": " << minRad << endl;
        
        //Draw the maximum radius
        if (maxRad == 0) continue;
        else if (maxRad > 1000){
            line(display, Point(viewerLoc.x*pixelsPerMeter, 0), Point(viewerLoc.x*pixelsPerMeter, display.rows), curveColor, 2);
            continue;
        }
        curveCenter = Point((viewerLoc.x+maxRad)*pixelsPerMeter, display.rows);
        maxRad = abs(maxRad);
        circle(display, curveCenter, maxRad*pixelsPerMeter, curveColor, 2);
        cout << "Max rad " << i << ": " << maxRad << endl;
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
    return ((cathetus/abs(sin(beta)))/2)*radiusSign; //Negative because it is at the left side of the
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
    
    //Define an array to store the neighbor types and variable to store neighbor location
    int neighborType[8];
    int nX = x, nY = y-1;
    
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
        if (lastCornerX*corners.at(i).cornerLoc.x < 0) {
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
    float rad1 = noCurveAboveBlock == true ? 1000 : 0; //This is radius on top of the block if possible, otherwise it is the one at the right of the block below it
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
            else if (currentCorner.cornerLoc.x > -PathPlaner::HALF_VEHICLE_WIDTH && currentCorner.cornerLoc.x < PathPlaner::HALF_VEHICLE_WIDTH){
                if (currentCorner.isLeft) {
                    newRadius = radiusForTangency(Point2f(currentCorner.cornerLoc.x - PathPlaner::HALF_VEHICLE_WIDTH, currentCorner.cornerLoc.y));
                    newRadius -= PathPlaner::HALF_VEHICLE_WIDTH;
                    if (abs(newRadius) < abs(rad2))
                        rad2 = newRadius;
                }
                else{
                    newRadius = radiusForTangency(Point2f(currentCorner.cornerLoc.x + PathPlaner::HALF_VEHICLE_WIDTH, currentCorner.cornerLoc.y));
                    newRadius -= PathPlaner::HALF_VEHICLE_WIDTH;
                    rad1 = newRadius;
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


/*************************************************************
 * Handy method for comparing the sign of two float values.  *
 *************************************************************/
bool areDifferentSign(float val1, float val2){
    return val1*val2<0?true:false;
}

/*********************************************************************************************
 * FUNCTION: findValidRadiusesRanges                                                         *
 * This function receives the radiuses to avoid each block and selects the ones that do not  *
 * intersect radiuses from other blocks. This means to ensure that the robot fits in a path. *
 * 25/11/2014: Debugged and working as expected.                                             *
 *********************************************************************************************/

vector<RadiusPair> findValidRadiusesRanges(vector<RadiusPair> tangentRadiusesPairs){
    vector<RadiusPair> validRanges;
    
    ///////////////////////////////////////////////////////////////////////////////////
    //////////STEP 1: Find the left-most block.                          //////////////
    ///////////////////////////////////////////////////////////////////////////////////
    int firstBlockIndex = -1;
    float firstRad = 10000.0;
    for (unsigned int i = 0; i < tangentRadiusesPairs.size(); i++) {
        if (tangentRadiusesPairs.at(i).radius2 < 0){
            if (abs(tangentRadiusesPairs.at(i).radius2) < abs(firstRad)) {
                firstRad = tangentRadiusesPairs.at(i).radius2;
                firstBlockIndex = i;
            }
        }
    }
    if (firstBlockIndex == -1) {
        firstRad = 0;
        for (int i = 0; i < tangentRadiusesPairs.size(); i++) {
            if (tangentRadiusesPairs.at(i).radius1 > 0){
                if (tangentRadiusesPairs.at(i).radius1 > firstRad) {
                    firstRad = tangentRadiusesPairs.at(i).radius1;
                    firstBlockIndex = i;
                }
            }
        }
    }
    
    /////////////////////////////////////////////////////////////////////////////////////////
    //////////STEP 2: Add the first radius range, from -0.0001 to the block. ////////////////
    /////////////////////////////////////////////////////////////////////////////////////////
    RadiusPair newValidRange;
    newValidRange.radius1 = -0.0001;
    newValidRange.radius2 = firstRad;
    validRanges.push_back(newValidRange);
    if (tangentRadiusesPairs.size() == 1) {
        if (firstRad < 0)
            newValidRange.radius1 = tangentRadiusesPairs.at(firstBlockIndex).radius1;
        else if(firstRad > 0)
            newValidRange.radius1 = tangentRadiusesPairs.at(firstBlockIndex).radius2;
        newValidRange.radius2 = 0.0001;
        validRanges.push_back(newValidRange);
        return validRanges;
    }
    
    /////////////////////////////////////////////////////////////////////////////////////////
    //////////STEP 3: Find the next block and check the validity of the radius //////////////
    //////////        range in between.                                        //////////////
    /////////////////////////////////////////////////////////////////////////////////////////

    int currentIdx = firstBlockIndex;
    while (true) {
        if (currentIdx == tangentRadiusesPairs.size()-1)
            break;
        float r01 = tangentRadiusesPairs.at(currentIdx).radius1;
        float r02 = tangentRadiusesPairs.at(currentIdx).radius2;
        float r0 = r02 < 0? r01 : r02;  //r0 is the right-most radius of the current block
        float smallestR1Diff = 100000;
        int nextIdx = -1;
        for (int i = currentIdx; i < tangentRadiusesPairs.size(); i++) {
            if (i == currentIdx) continue;
            float r1 = tangentRadiusesPairs.at(i).radius1;
            float r2 = tangentRadiusesPairs.at(i).radius2;
            if (r0 < 0) {
                if (r1 > 0 && nextIdx > currentIdx) break;
                if (abs(r1)-abs(r0) < smallestR1Diff) {
                    smallestR1Diff = abs(r1)-abs(r0);
                    nextIdx = i;
                }
            }
            else if (r0 > 0){
                if (r0 - r2 < smallestR1Diff) {
                    smallestR1Diff = r0 - r2;
                    nextIdx = i;
                }
            }
        }
        if (nextIdx == -1) {     //Only in the case that a range goes from negative to positive
            for (int i = 0; i < tangentRadiusesPairs.size(); i++) {
                if (i == currentIdx) continue;
                float r2 = tangentRadiusesPairs.at(i).radius2;
                if (r0 < 0) {
                    if (r2 < smallestR1Diff) {
                        smallestR1Diff = r2;
                        nextIdx = i;
                    }
                }
            }
        }
        if (nextIdx == -1) {
            cout << "Something went wrong!!!" << endl;
        }
        float nr1 = tangentRadiusesPairs.at(nextIdx).radius1;
        float nr2 = tangentRadiusesPairs.at(nextIdx).radius2;
        RadiusPair nextRadRange;
        nextRadRange.radius1 = r0;
        if (nr2 < 0){
            if (abs(nr2) - abs(r0) > 0){
                nextRadRange.radius2 = nr2;
                validRanges.push_back(nextRadRange);
            }
        }
        else{
            if (r0 - nr1 > 0 || areDifferentSign(r0, nr1)){
                nextRadRange.radius2 = nr1;
                validRanges.push_back(nextRadRange);
            }
        }
        currentIdx = nextIdx;
    }
    RadiusPair lastRadRange;
    float r01 = tangentRadiusesPairs.at(currentIdx).radius1;
    float r02 = tangentRadiusesPairs.at(currentIdx).radius2;
    lastRadRange.radius1 = r02 < 0 ? r01 : r02;
    lastRadRange.radius2 = 0.0001;
    validRanges.push_back(lastRadRange);
    return validRanges;
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
    
    for (int i = 0; i<scenX; i++) {
        for (int j = 0; j<scenY; j++) {
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
