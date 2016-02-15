/*
 * Simulator.cpp
 *
 *  Created on: Jul 26, 2014
 *     Author: Alejandro Daniel Noel
 *     Page: http://futuretechmaker.com
 */

#include "Simulator.h"

struct MouseData{
	Point p;
	int mouseEventType;
};

void CallBackFunc(int event, int x, int y, int flags, void* userdata){
	MouseData *md = (MouseData*)userdata;
	md->p.x = x;
	md->p.y = y;
	md->mouseEventType = event;
}

const Scalar Simulator::COLOR_EMPTY    = Scalar(255, 255, 255);
const Scalar Simulator::COLOR_OCCUPIED = Scalar(50, 50, 50);
const Scalar Simulator::COLOR_POSITION = Scalar(50, 180, 50);
const Scalar Simulator::COLOR_TARGET   = Scalar(50, 50, 180);
const Scalar Simulator::COLOR_SHADOW   = Scalar(255, 255, 255);
const Scalar Simulator::COLOR_GRID = Scalar(215, 215, 215);
const Scalar Simulator::COLOR_SEPARATOR_LINE = Scalar(50, 120, 200);


/*=========================================================================================*\
|   Initialization                                                                          |
\*=========================================================================================*/

Simulator::Simulator(float width, float height, float squareSize, int windowWidth, string _dataDirectory) {
    dataDirectory = _dataDirectory;
    XSquares = ceil(width/squareSize);
    YSquares = ceil(height/squareSize);
    squareRealSize = squareSize;

    //Make XSquares an even number so the car appears centered.
    if (XSquares % 2 != 0){
        XSquares += 1;
        width += squareSize;
    }

    // Initialise an empty scenario
    scenario = ObstacleScenario(width, height, squareSize);
    
    //Create window content
    ///Grid dimensions

    squarePixelSize = ceil(windowWidth/XSquares);
    
    windowSize = Size(squarePixelSize*XSquares, squarePixelSize*YSquares);
    display = Mat(windowSize, CV_8UC3, COLOR_EMPTY);
    drawGrid();
}



/*=========================================================================================*\
|   Private utility methods                                                                 |
\*=========================================================================================*/

void Simulator::markSquare(int markType, Point2i square, bool isShadow = false){
	//Calculate square corners
	Point corner0 = Point(square.x*squarePixelSize, square.y*squarePixelSize);
    //Subtract 1 so that the grid lines are not painted
	Point corner1 = Point(corner0.x + squarePixelSize-1, corner0.y + squarePixelSize-1);
    
	scenario.scenario.at(square.x).at(square.y) = markType;
	
    switch(markType){
        case -1:
            rectangle(display, corner0, corner1, COLOR_SHADOW, CV_FILLED);
            break;
        case 0:
            if (isShadow)
                rectangle(display, corner0, corner1, COLOR_SHADOW, CV_FILLED);
            else
                rectangle(display, corner0, corner1, COLOR_EMPTY, CV_FILLED);
            break;
        case 1:
            rectangle(display, corner0, corner1, COLOR_OCCUPIED, CV_FILLED);
            break;
        case 2:
            rectangle(display, corner0, corner1, COLOR_POSITION, CV_FILLED);
            break;
        case 3:
            rectangle(display, corner0, corner1, COLOR_TARGET, CV_FILLED);
            break;
    }
}

void Simulator::clearColumn(int column){
    for (unsigned int i = 0; i<scenario.scenario.at(i).size(); i++) {
        markSquare(0, Point2i(column, i));
    }
}

void Simulator::drawShadow(Point2i square){
    bool drawSadow = true;
    for (int j = 0; j<square.y; j++) {
        if (scenario.scenario.at(square.x).at(j) == 0) {
            markSquare(0, Point2i(square.x, j), drawSadow);
        }
    }
}

void Simulator::drawGrid(){
    int verticalLines = XSquares-1;
    int horizontalLines = YSquares-1;
    for(int i=0; i<=verticalLines; i++){
        int XOffset = (i+1)*squarePixelSize;
        line(display, Point(XOffset, 0), Point(XOffset, display.rows-1), COLOR_GRID);
    }
    for(int i=0; i<=horizontalLines; i++){
        int YOffset = (i+1)*squarePixelSize;
        line(display, Point(0, YOffset), Point(display.cols-1, YOffset), COLOR_GRID);
    }
    
    //Draw vertical separator line
    //line(display, Point(ceil(display.cols/2), 0), Point(ceil(display.cols/2), display.rows-1), COLOR_SEPARATOR_LINE, 2);
    //Draw FOV lines
    int halfDisplay = ceil(display.cols/2);
    float alpha = ((180-fov)*M_PI) / 360;
    int h = ceil(tan(alpha)*halfDisplay);
    line(display, Point(halfDisplay, display.rows), Point(0, display.rows-h), COLOR_SEPARATOR_LINE, 1);
    line(display, Point(halfDisplay, display.rows), Point(display.cols, display.rows-h), COLOR_SEPARATOR_LINE, 1);
    
    ////////Draw a car////////////
    if (!dataDirectory.empty()) {
        Mat carImg = imread(dataDirectory + "car_top.png", CV_LOAD_IMAGE_COLOR);
        resize(carImg, carImg, Size(60, 120));
        Rect roi(0, 0, carImg.cols,carImg.rows/2-5);
        Mat image_roi = carImg(roi);
        image_roi.copyTo(carImg);
        Mat displayROI(display, Rect(display.cols/2-carImg.cols/2, display.rows-carImg.rows, carImg.cols, carImg.rows));
        carImg.copyTo(displayROI);
    }
    //////////////////////////////
}

/*=========================================================================================*\
|   Avoidance simulator                                                                     |
\*=========================================================================================*/

void Simulator::runSimulation(bool autoEraseColumns){
    /////////Create a window object and set up mouse events/////////////
    namedWindow("Avoidance simulator", 1);
    MouseData mouse;
    setMouseCallback("Avoidance simulator", CallBackFunc, &mouse);
    ////////////////////////////////////////////////////////////////////
    
    imshow("Avoidance simulator", display);
    
    while(1){
        //Get current key pressed, if
        int keyPressed = int(char(waitKey(10)));
        
        /////////////Get the current square under the cursor/////////////////
        Point currentPixel = mouse.p;
        int sqX = ceil(currentPixel.x/squarePixelSize);
        int sqY = ceil(currentPixel.y/squarePixelSize);
        Point2i currentSquare = Point2i(sqX, sqY);
        ////////////////////////////////////////////////////////////////////

        //Mark as occupied (if keyPressed is 'a') or empty (if keyPressed is 's')
        if(keyPressed == 115){	//ASCII 97 = 's'
            //Mark square as empty
            markSquare(0, currentSquare);
            if(autoEraseColumns) clearColumn(currentSquare.x);
        }
        else if(keyPressed == 97){	//ASCII 97 = 'a'
            //Mark square as occupied
            if(autoEraseColumns) clearColumn(currentSquare.x);
            markSquare(1, currentSquare);
            drawShadow(currentSquare);
        }
        drawGrid();
        
        // Draw available commands on the bottom-left corner of the window
        string message = "a + move cursor: Paint obstacles";
        putText(display, message, Point(10, display.rows - 70), FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 123, 47), 1, 4);
        message = "s + move cursor: Erase obstacles";
        putText(display, message, Point(10, display.rows - 50), FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 123, 47), 1, 4);
        message = "ENTER: Generate avoidance path";
        putText(display, message, Point(10, display.rows - 30), FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 123, 47), 1, 4);
        message = "ESC: Close mode";
        putText(display, message, Point(10, display.rows - 10), FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 123, 47), 1, 4);
        
        //Update the simulator display
        imshow("Avoidance simulator", display);

        
        if( keyPressed == 27)	break;          //If keypressed is ESC exit simulation

        else if(keyPressed == 13 || keyPressed == 10){   //If keypressed is ENTER send scenario to path planner
            ObstacleScenario obstacleScenario = scenario;
            obstacleScenario.squareSize = squareRealSize;
            PathPlaner planer;
            float pathRadius;
            pathRadius = planer.findAvoidancePath(obstacleScenario, 10000, display, squarePixelSize);
            ////////Draw a car////////////
            if (!dataDirectory.empty()) {
                Mat carImg = imread(dataDirectory + "car_top.png", CV_LOAD_IMAGE_COLOR);
                resize(carImg, carImg, Size(60, 120));
                Rect roi(0, 0, carImg.cols,carImg.rows/2-5);
                Mat image_roi = carImg(roi);
                image_roi.copyTo(carImg);
                Mat displayROI(display, Rect(display.cols/2-carImg.cols/2, display.rows-carImg.rows, carImg.cols, carImg.rows));
                carImg.copyTo(displayROI);
            }
            //////////////////////////////
            message = "Press any key to continue";
            putText(display, message, Point(10, display.rows - 90), FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 123, 47), 1, 4);
            imshow("Avoidance simulator", display);
            waitKey();
            // Restart the simulator
            scenario.clearScenario();
            display = Mat(display.rows, display.cols, CV_8UC3, COLOR_EMPTY);
            drawGrid();
        }
    }
    destroyWindow("Avoidance simulator");
    for(int i = 0; i < 10; i++) waitKey(1); // In some systems, if this is not included windows may becmome unresponsive.
}

Mat Simulator::drawScenario(vector<Point2f> points, float fov){

    display = Mat(windowSize, CV_8UC3, COLOR_EMPTY);
    # pragma omp parallel for
    for (int i = 0; i<XSquares; i++) {
        for (int j = 0; j < YSquares; j++) {
            if (scenario.scenario.at(i).at(j) == 1) {
                markSquare(1, Point2i(i, j));
            }
        }
    }
    drawGrid();
    
    if (!points.empty()) {
        # pragma omp parallel for
        for (int i = 0; i < points.size(); i++) {
            Point pt_scrnCoords = Point(points.at(i).x*(squarePixelSize/scenario.squareSize), points.at(i).y*(squarePixelSize/scenario.squareSize));
            circle(display, pt_scrnCoords, 1, CV_RGB(250, 20, 20));
        }
    }
    
    if (fov > 0) {
        //Draw FOV lines
        int halfDisplay = ceil(display.cols/2);
        float alpha = ((180-fov)*M_PI) / 360;
        int h = ceil(tan(alpha)*halfDisplay);
        line(display, Point(halfDisplay, display.rows), Point(0, display.rows-h), COLOR_SEPARATOR_LINE, 1);
        line(display, Point(halfDisplay, display.rows), Point(display.cols, display.rows-h), COLOR_SEPARATOR_LINE, 1);
    }
    
    //Update the simulator display
    return display;
}
