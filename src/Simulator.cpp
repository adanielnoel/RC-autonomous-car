/*
 * Simulator.cpp
 *
 *  Created on: Sep 1, 2014
 *      Author: alejandro
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
const int Simulator::TYPE_AVOIDANCE = 0;
const int Simulator::TYPE_NAVIGATION = 1;
const int Simulator::TYPE_NONE = 2;

/*******************************************************************************************\
|   Initialization                                                                          |
\*******************************************************************************************/

Simulator::Simulator(){
    squarePixelSize = 30;
    simulatorType = TYPE_NONE;
}

Simulator::Simulator(float data1, float data2, int type, float squareSize, Size _windowSize) {
    windowSize = _windowSize;
    simulatorType = type;
    squareRealSize = squareSize;
    if (type == TYPE_AVOIDANCE) {       //Avoidance simulator
        //Calculate scenario dimensions
        depth = data1;
        fov = data2;
        float alpha = ((180-fov)/2)*M_PI / 180;
        XSquares = (2*(depth * cos(alpha)))/squareSize; //Calculate the scenario width
        YSquares = depth/squareSize;
        if (XSquares%2 != 0) {          //Make XSquares an even number
            XSquares++;
        }
    }
    
    else if (type == TYPE_NAVIGATION){  //Navigation simulator
        XSquares = data1/squareSize;
        YSquares = data2/squareSize;
    }
    
    // Initialise an empty scenario
    scenario = initEmptyScen(XSquares, YSquares);
    
    //Create window content
    ///Grid dimensions

    squarePixelSize = ceil(windowSize.width/XSquares);
    
    int windowWidth = squarePixelSize*XSquares;
    int windowHeight = squarePixelSize*YSquares;
    display = Mat(windowHeight, windowWidth, CV_8UC3, COLOR_EMPTY);
    drawGrid();
}

vector<vector<int> > Simulator::initEmptyScen(int XSquares, int YSquares){
    vector<vector<int> > scenario;
    for(int x=0; x<XSquares; x++){
        vector<int> newRow;
        for(int y=0; y<YSquares; y++){
            newRow.push_back(0);        //Initialize square with value 0 (empty)
        }
        scenario.push_back(newRow);
    }
    return scenario;
}

/*******************************************************************************************\
|   Private utility methods                                                                 |
\*******************************************************************************************/

void Simulator::markSquare(int markType, Point2i square, bool isShadow = false){
	//Calculate square corners
	Point rect0 = Point(square.x*squarePixelSize, square.y*squarePixelSize);
	Point rect1 = Point(square.x*squarePixelSize + squarePixelSize-1, square.y*squarePixelSize + squarePixelSize-1);
    
	scenario.at(square.x).at(square.y) = markType;
	
    switch(markType){
        case -1:
            rectangle(display, rect0, rect1, COLOR_SHADOW, CV_FILLED);
            break;
        case 0:
            if (isShadow)
                rectangle(display, rect0, rect1, COLOR_SHADOW, CV_FILLED);
            else
                rectangle(display, rect0, rect1, COLOR_EMPTY, CV_FILLED);
            break;
        case 1:
            rectangle(display, rect0, rect1, COLOR_OCCUPIED, CV_FILLED);
            break;
        case 2:
            rectangle(display, rect0, rect1, COLOR_POSITION, CV_FILLED);
            break;
        case 3:
            rectangle(display, rect0, rect1, COLOR_TARGET, CV_FILLED);
            break;
    }
}

void Simulator::clearColumn(int column){
    for (unsigned int i = 0; i<scenario.at(i).size(); i++) {
        markSquare(0, Point2i(column, i));
    }
}

void Simulator::drawShadow(Point2i square){
    bool drawSadow = true;
    for (int j = 0; j<square.y; j++) {
        if (scenario.at(square.x).at(j) == 0) {
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
    if (simulatorType == TYPE_AVOIDANCE) {
        //Draw vertical separator line
        //line(display, Point(ceil(display.cols/2), 0), Point(ceil(display.cols/2), display.rows-1), COLOR_SEPARATOR_LINE, 2);
        //Draw FOV lines
        int halfDisplay = ceil(display.cols/2);
        float alpha = ((180-fov)*M_PI) / 360;
        int h = ceil(tan(alpha)*halfDisplay);
        line(display, Point(halfDisplay, display.rows), Point(0, display.rows-h), COLOR_SEPARATOR_LINE, 1.5);
        line(display, Point(halfDisplay, display.rows), Point(display.cols, display.rows-h), COLOR_SEPARATOR_LINE, 1.5);
        
        ////////Draw a car////////////
        Mat carImg = imread("/Users/alejandrodanielnoel/Documents/XCode projects/Autonomous_Car/src/car_top.png");
        resize(carImg, carImg, Size(60, 120));
        Rect roi(0, 0, carImg.cols,carImg.rows/2-5);
        Mat image_roi = carImg(roi);
        image_roi.copyTo(carImg);
        Mat displayROI(display, Rect(display.cols/2-carImg.cols/2, display.rows-carImg.rows, carImg.cols, carImg.rows));
        carImg.copyTo(displayROI);
        //////////////////////////////
    }
}

/*******************************************************************************************\
|   Avoidance simulator                                                                     |
\*******************************************************************************************/

void Simulator::avoidanceSimulator(bool autoEraseColumns = false){
    /////////Create a window object and set up mouse events/////////////
    namedWindow("My Window", 1);
    MouseData mouse;
    setMouseCallback("My Window", CallBackFunc, &mouse);
    ////////////////////////////////////////////////////////////////////
    
    printf("Press 'a' and move cursor to occupy squares.\nPress 's' and move cursor to empty occupied squares.\n");
    printf("Press ENTER to generate a path.\n");
    //Show simulator display
    imshow("My Window", display);
    
    while(1){
        //Get current key pressed, if
        int keyPressed = waitKey(1);
        
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
            drawGrid();
            //Update the simulator display
            imshow("My Window", display);
        }
        else if(keyPressed == 97){	//ASCII 97 = 'a'
            //Mark square as occupied
            if(autoEraseColumns) clearColumn(currentSquare.x);
            markSquare(1, currentSquare);
            drawShadow(currentSquare);
            drawGrid();
            //Update the simulator display
            imshow("My Window", display);
        }
        
        if( keyPressed == 27)	break;          //If keypressed is ESC exit simulation

        else if(keyPressed == 13){              //If keypressed is ENTER send scenario to path planner
            ObstacleScenario obstacleScenario;
            obstacleScenario.scenario = scenario;
            obstacleScenario.squareSize = squareRealSize;
            PathPlaner planer;
            float pathRadius;
            pathRadius = planer.findAvoidancePath(obstacleScenario, 10000, display, squarePixelSize);
            ////////Draw a car////////////
            Mat carImg = imread("/Users/alejandrodanielnoel/Documents/XCode projects/Autonomous_Car/src/car_top.png");
            resize(carImg, carImg, Size(60, 120));
            Rect roi(0, 0, carImg.cols,carImg.rows/2-5);
            Mat image_roi = carImg(roi);
            image_roi.copyTo(carImg);
            Mat displayROI(display, Rect(display.cols/2-carImg.cols/2, display.rows-carImg.rows, carImg.cols, carImg.rows));
            carImg.copyTo(displayROI);
            //////////////////////////////
            imshow("My Window", display);
            int key = waitKey(0);
            if (key == 114) {
                scenario = initEmptyScen(XSquares, YSquares);
                display = Mat(display.rows, display.cols, CV_8UC3, COLOR_EMPTY);
                drawGrid();
                avoidanceSimulator();
            }
            if(pathRadius == 0) cout << "***NO SUITABLE PATH FOUND***" << endl;
            else {
                if(pathRadius > 1000) cout << "Found a straight path." << endl;
                else cout << "Found a path with radius: " << pathRadius << endl;
            }
            break;
        }
    }
}

/*******************************************************************************************\
|   Navigation simulator                                                                    |
\*******************************************************************************************/

void Simulator::navigationSimulator(){
    Point2i initialLocation = Point2i(-1, -1);  //Negative values indicate that this has not been set up
    Point2i targetLocation = Point2i(-1, -1);
    
    /////////Create a window object and set up mouse events/////////////
    namedWindow("My Window", 1);
    MouseData mouse;
    setMouseCallback("My Window", CallBackFunc, &mouse);
    ////////////////////////////////////////////////////////////////////
    
    printf("Press 'a' and move cursor to occupy squares.\nPress 's' and move cursor to empty occupied squares.\n");
    printf("Click to place the initial square.\nClick again to place the target square.\n");
    printf("Press ENTER to generate a path.\n");
    //Show simulator display
    imshow("My Window", display);
    
    while(1){
        //Get current key pressed, if
        int keyPressed = waitKey(1);
        
        /////////////Get the current square under the cursor/////////////////
        Point currentPixel = mouse.p;
        int sqX = ceil(currentPixel.x/squarePixelSize);
        int sqY = ceil(currentPixel.y/squarePixelSize);
        Point2i currentSquare = Point2i(sqX, sqY);
        ////////////////////////////////////////////////////////////////////
        
        if(mouse.mouseEventType == EVENT_LBUTTONDOWN){
            if(initialLocation.x < 0 || initialLocation.y < 0){
                initialLocation = currentSquare;
                markSquare(2, currentSquare);
                drawGrid();
                //Update the simulator display
                imshow("My Window", display);
             }
             else if((targetLocation.x < 0 || targetLocation.y < 0) && (scenario.at(sqX).at(sqY) != 2)){
                 targetLocation = currentSquare;
                 markSquare(3, currentSquare);
                 drawGrid();
                 //Update the simulator display
                 imshow("My Window", display);
             }
            continue;
        }
        //Mark as occupied (if keyPressed is 'a') or empty (if keyPressed is 's')
        if(keyPressed == 115){	//ASCII 97 = 's'
            //Mark square as empty
            markSquare(0, currentSquare);
            drawGrid();
            //Update the simulator display
            imshow("My Window", display);
        }
        else if(keyPressed == 97){	//ASCII 97 = 'a'
            //Mark square as occupied
            markSquare(1, currentSquare);
            drawGrid();
            //Update the simulator display
            imshow("My Window", display);
        }
        
        if( keyPressed == 27)	break;          //If keypressed is ESC exit simulation
        
        else if(keyPressed == 13){              //If keypressed is ENTER send scenario to path planner
            ObstacleScenario obstacleScenario;
            obstacleScenario.scenario = scenario;
            PathPlaner planer;
            bool success;
            success = planer.findNavigationPath(obstacleScenario, initialLocation, targetLocation);
            if(!success) cout << "***NO SUITABLE PATH FOUND***" << endl;
            break;
        }
    }
}

/*******************************************************************************************\
|   Simulator auto-choose                                                                   |
\*******************************************************************************************/
void Simulator::runSimulation(){
    if (simulatorType == TYPE_AVOIDANCE)
        avoidanceSimulator();
    else if (simulatorType == TYPE_NAVIGATION)
        navigationSimulator();
}

void Simulator::displayScenario(ObstacleScenario &obstacleScen, bool runPathPlanner){
    bool obstaclesPresent = false;
    simulatorType = TYPE_AVOIDANCE;
    fov = 90;
    namedWindow("Simulator", 1);
    this->scenario = obstacleScen.scenario;
    XSquares = scenario.size();
    YSquares = scenario.at(1).size();
    int windowWidth = squarePixelSize*XSquares;
    int windowHeight = squarePixelSize*YSquares;
    display = Mat(windowHeight, windowWidth, CV_8UC3, COLOR_EMPTY);
    initEmptyScen(XSquares, YSquares);
    
    for (int i = 0; i<XSquares; i++) {
        for (int j = 0; j < YSquares; j++) {
            if (scenario.at(i).at(j) == 1) {
                markSquare(1, Point2i(i, j));
                obstaclesPresent = true;
            }
        }
    }
    //cout << rand() << endl;
    drawGrid();
    
    if (runPathPlanner && obstaclesPresent) {
        PathPlaner planer;
        float pathRadius;
        pathRadius = planer.findAvoidancePath(obstacleScen, 10000, display, squarePixelSize);
        imshow("Simulator", display);
        waitKey();
    }
    //Update the simulator display
    imshow("Simulator", display);
}

Simulator::~Simulator() {
	// TODO Auto-generated destructor stub
}
