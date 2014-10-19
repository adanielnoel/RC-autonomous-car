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

const Scalar Simulator::colorEmpty = Scalar(255, 255, 255);
const Scalar Simulator::colorOccupied = Scalar(50, 50, 50);
const Scalar Simulator::colorPosition = Scalar(50, 180, 50);
const Scalar Simulator::colorTarget = Scalar(50, 50, 180);
const Scalar Simulator::colorShadow = Scalar(230, 230, 230);
const Scalar Simulator::colorGrid = Scalar(150, 150, 150);
const Scalar Simulator::colorSeparatorLine = Scalar(20, 20, 220);
const Scalar Simulator::colorMessage = Scalar(20, 20, 100);
const int Simulator::TYPE_AVOIDANCE = 0;
const int Simulator::TYPE_NAVIGATION = 1;


Simulator::Simulator(float data1, float data2, int type, float squareSize, Size _windowSize) {
    windowSize = _windowSize;
    simulatorType = type;
    if (type == TYPE_AVOIDANCE) {      //Avoidance simulator
        //Calculate scenario dimensions
        float depth = data1;
        float fov = data2;
        XSquares = (2*(depth * cos((180-fov)/2)))/squareSize; //Calculate the scenario width
        YSquares = depth/squareSize;
        if (XSquares%2 != 0) {  //Make XSquares an even number
            XSquares++;
        }
    }
    
    else if (type == TYPE_NAVIGATION){    //Navigation simulator
        XSquares = data1/squareSize;
        YSquares = data2/squareSize;
    }
    // Initialise an empty scenario
    ObstacleScenario obstacleScenario;

    
    // Create a vector grid
//    vector< vector<int> > scenario;
    for(int x=0; x<XSquares; x++){
        vector<int> newRow;
        for(int y=0; y<YSquares; y++){
            newRow.push_back(0);	//Initialize square with value 0 (empty)
        }
        scenario.push_back(newRow);
    }
    
    //Create window content
    ///Grid dimensions
    cout<< "XSquares: " << XSquares << " | YSquares: " << YSquares << endl;
    int verticalLines = XSquares-1;
    int horizontalLines = YSquares-1;
    squarePixelSize = ceil(windowSize.width/XSquares);
    
    int windowWidth = squarePixelSize*XSquares + verticalLines;
    int windowHeight = squarePixelSize*YSquares + horizontalLines;
    display = Mat(windowHeight, windowWidth, CV_8UC3, colorEmpty);
    
    for(int i=0; i<=verticalLines; i++){
        int XOffset = (i+1)*squarePixelSize+i;
        line(display, Point(XOffset, 0), Point(XOffset, windowHeight-1), colorGrid);
    }
    for(int i=0; i<=horizontalLines; i++){
        int YOffset = (i+1)*squarePixelSize+i;
        line(display, Point(0, YOffset), Point(windowWidth-1, YOffset), colorGrid);
    }
    if (type == TYPE_AVOIDANCE) {
        line(display, Point(ceil(display.cols/2), 0), Point(ceil(display.cols/2), windowHeight-1), colorSeparatorLine, 2);
    }
}

void Simulator::markSquare(int markType, Point2i square){
	//Calculate square corners
	Point rect0 = Point(square.x*squarePixelSize + square.x, square.y*squarePixelSize + square.y);
	Point rect1 = Point(square.x*squarePixelSize + square.x + squarePixelSize-1, square.y*squarePixelSize + square.y + squarePixelSize-1);
    
	scenario.at(square.x).at(square.y) = markType;
	
    switch(markType){
        case -1:
            rectangle(display, rect0, rect1, colorShadow, CV_FILLED);
            break;
        case 0:
            rectangle(display, rect0, rect1, colorEmpty, CV_FILLED);
            break;
        case 1:
            rectangle(display, rect0, rect1, colorOccupied, CV_FILLED);
            break;
        case 2:
            rectangle(display, rect0, rect1, colorPosition, CV_FILLED);
            break;
        case 3:
            rectangle(display, rect0, rect1, colorTarget, CV_FILLED);
            break;
    }
}

void Simulator::clearColumn(int column){
    for (unsigned int i = 0; i<scenario.at(i).size(); i++) {
        markSquare(0, Point2i(column, i));
    }
}

void    Simulator::drawShadow(Point2i square){
    for (int i = 0; i<square.y; i++) {
        markSquare(-1, Point2i(square.x, i));
    }
}

void Simulator::avoidanceSimulator(){
    /////////Create a window object and set up mouse events/////////////
    namedWindow("My Window", 1);
    MouseData mouse;
    //set the callback function for any mouse event
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
        int sqX = ceil((currentPixel.x+1)/(squarePixelSize+1));
        int sqY = ceil((currentPixel.y+1)/(squarePixelSize+1));
        
        Point2i currentSquare = Point2i(sqX, sqY);
        ////////////////////////////////////////////////////////////////////

        //Mark as occupied (if keyPressed is 'a') or empty (if keyPressed is 's')
        if(keyPressed == 115){	//ASCII 97 = 's'
            //Mark square as empty
            markSquare(0, currentSquare);
            clearColumn(currentSquare.x);
            //Update the simulator display
            imshow("My Window", display);
        }
        else if(keyPressed == 97){	//ASCII 97 = 'a'
            //Mark square as occupied
            clearColumn(currentSquare.x);
            markSquare(1, currentSquare);
            drawShadow(currentSquare);
            //Update the simulator display
            imshow("My Window", display);
        }
        
        if( keyPressed == 27)	break;					//If keypressed is ESC exit simulation
        
        else if(keyPressed == 13){//If keypressed is ENTER send scenario to path planner
            ObstacleScenario obstacleScenario;
            obstacleScenario.scenario = scenario;
            PathPlaner planer;
            bool success;
            success = planer.findAvoidancePath(obstacleScenario, display, squarePixelSize);
            if(!success) cout << "***NO SUITABLE PATH FOUND***" << endl;
            break;
        }
    }
}

void Simulator::navigationSimulator(){
    Point2i initialLocation = Point2i(-1, -1); //Negative values indicate that this has not been set up
    Point2i targetLocation = Point2i(-1, -1);
    
    /////////Create a window object and set up mouse events/////////////
    namedWindow("My Window", 1);
    MouseData mouse;
    //set the callback function for any mouse event
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
        int sqX = ceil((currentPixel.x+1)/(squarePixelSize+1));
        int sqY = ceil((currentPixel.y+1)/(squarePixelSize+1));
        
        Point2i currentSquare = Point2i(sqX, sqY);
        ////////////////////////////////////////////////////////////////////
        if(mouse.mouseEventType == EVENT_LBUTTONDOWN){
            if(initialLocation.x < 0 || initialLocation.y < 0){
                 initialLocation = currentSquare;
                 markSquare(2, currentSquare);
                //Update the simulator display
                imshow("My Window", display);
             }
             else if((targetLocation.x < 0 || targetLocation.y < 0) && (scenario.at(sqX).at(sqY) != 2)){
                 targetLocation = currentSquare;
                 markSquare(3, currentSquare);
                 //Update the simulator display
                 imshow("My Window", display);
             }
            continue;
        }
        //Mark as occupied (if keyPressed is 'a') or empty (if keyPressed is 's')
        if(keyPressed == 115){	//ASCII 97 = 's'
            //Mark square as empty
            markSquare(0, currentSquare);
            //Update the simulator display
            imshow("My Window", display);
        }
        else if(keyPressed == 97){	//ASCII 97 = 'a'
            //Mark square as occupied
            markSquare(1, currentSquare);
            //Update the simulator display
            imshow("My Window", display);
        }
        
        if( keyPressed == 27)	break;					//If keypressed is ESC exit simulation
        
        else if(keyPressed == 13){//If keypressed is ENTER send scenario to path planner
            ObstacleScenario obstacleScenario;
            obstacleScenario.scenario = scenario;
            PathPlaner planer;
            bool success;
            success = planer.findAvoidancePath(obstacleScenario, display, squarePixelSize);
            if(!success) cout << "***NO SUITABLE PATH FOUND***" << endl;
            break;
        }
    }
}


void Simulator::runSimulation(){
    if (simulatorType == TYPE_AVOIDANCE)
        avoidanceSimulator();
    else if (simulatorType == TYPE_NAVIGATION)
        navigationSimulator();
}

Simulator::~Simulator() {
	// TODO Auto-generated destructor stub
}
