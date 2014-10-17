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

Simulator::Simulator(float _depth, float fov, float squareSize, Size _windowSize) {

	//Calculate scenario dimensions
	XSquares = (2*(_depth * cos((180-fov)/2)))/squareSize; //Calculate the scenario width
	YSquares = _depth/squareSize;
	windowSize = _windowSize;

	colorEmpty = Scalar(255, 255, 255);
	colorOccupied = Scalar(50, 50, 50);
	colorPosition = Scalar(50, 180, 50);
	colorTarget = Scalar(50, 50, 180);
	colorGrid = Scalar(150, 150, 150);
	colorMessage = Scalar(20, 20, 100);
}

void Simulator::markSquare(int markType, Point2i square, vector< vector<int> > &scenario, int sqPixSize, Mat &display){
	//Calculate square corners
	Point rect0 = Point(square.x*sqPixSize + square.x, square.y*sqPixSize + square.y);
	Point rect1 = Point(square.x*sqPixSize + square.x + sqPixSize-1, square.y*sqPixSize + square.y + sqPixSize-1);
	scenario.at(square.x).at(square.y) = markType;
	switch(markType){
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

void Simulator::runSimulation(){
	// Initialise an empty scenario
	ObstacleScenario obstacleScenario;
	Point2i initialLocation = Point2i(-1, -1); //Negative values indicate that this has not been set up
	Point2i targetLocation = Point2i(-1, -1);

	// Create a vector grid
	vector< vector<int> > newScenario;
	for(int x=0; x<XSquares; x++){
		vector<int> newRow;
		for(int y=0; y<YSquares; y++){
			newRow.push_back(0);	//Initialize square with value 0 (empty)
		}
		newScenario.push_back(newRow);
	}

	//Create window content
	///Grid dimensions
	cout<< "XSquares: " << XSquares << " | YSquares: " << YSquares << endl;
	int verticalLines = XSquares-1;
	int horizontalLines = YSquares-1;
	int squarePixelSize = ceil(windowSize.width/XSquares);

	int windowWidth = squarePixelSize*XSquares + verticalLines;
	int windowHeight = squarePixelSize*YSquares + horizontalLines;
	Mat scenarioDisplay(windowHeight, windowWidth, CV_8UC3, colorEmpty);
	Mat displayCopy;

	for(int i=0; i<=verticalLines; i++){
		int XOffset = (i+1)*squarePixelSize+i;
		line(scenarioDisplay, Point(XOffset, 0), Point(XOffset, windowHeight-1), colorGrid);
	}
	for(int i=0; i<=horizontalLines; i++){
		int YOffset = (i+1)*squarePixelSize+i;
		line(scenarioDisplay, Point(0, YOffset), Point(windowWidth-1, YOffset), colorGrid);
	}

	/////////Create a window object and set up mouse events/////////////
	namedWindow("My Window", 1);
	MouseData mouse;
	Point2i lastSquare;
	//set the callback function for any mouse event
	setMouseCallback("My Window", CallBackFunc, &mouse);
	////////////////////////////////////////////////////////////////////

	printf("Press 'a' and move cursor to occupy squares.\nPress 's' and move cursor to empty occupied squares.\n");
	printf("Click to place the initial square.\nClick again to place the target square.\n");
	printf("Press ENTER to generate a path.\n");

	while(1){
		//Get current key pressed, if
		int keyPressed = waitKey(1);

		/////////////Get the current square under the cursor/////////////////
		Point currentPixel = mouse.p;
		int mx = currentPixel.x; int my = currentPixel.y;
		int sqX = ceil((mx+1)/(squarePixelSize+1));
		int sqY = ceil((my+1)/(squarePixelSize+1));
		Point2i currentSquare = Point2i(sqX, sqY);
		////////////////////////////////////////////////////////////////////
		if(mouse.mouseEventType == EVENT_LBUTTONDOWN){
			/*if(newScenario.at(sqX).at(sqY) == 2){
				cout << "1" << endl;
				initialLocation = Point2i(-1, -1);
				markSquare(0, currentSquare, newScenario, squarePixelSize, scenarioDisplay);
				continue;
			}
			else if(newScenario.at(sqX).at(sqY) == 3 && keyPressed == 100){
				cout << "2" << endl;
				targetLocation = Point2i(-1, -1);
				markSquare(0, currentSquare, newScenario, squarePixelSize, scenarioDisplay);
				continue;
			}
			else*/ if(initialLocation.x < 0 || initialLocation.y < 0){
				initialLocation = currentSquare;
				markSquare(2, currentSquare, newScenario, squarePixelSize, scenarioDisplay);
			}
			else if((targetLocation.x < 0 || targetLocation.y < 0) && (newScenario.at(sqX).at(sqY) != 2)){
				targetLocation = currentSquare;
				markSquare(3, currentSquare, newScenario, squarePixelSize, scenarioDisplay);
			}
			continue;
		}
		//Mark as occupied (if keyPressed is 'a') or empty (if keyPressed is 's')
		if(keyPressed == 115){	//ASCII 97 = 's'
			//Mark square as empty
			markSquare(0, currentSquare, newScenario, squarePixelSize, scenarioDisplay);
		}
		else if(keyPressed == 97){	//ASCII 97 = 'a'
			//Mark square as occupied
			markSquare(1, currentSquare, newScenario, squarePixelSize, scenarioDisplay);
		}

		//Save current square for next iteration
		lastSquare = Point(sqX, sqY);

		//Update the simulator display
		imshow("My Window", scenarioDisplay);

		if( keyPressed == 27)	break;					//If keypressed is ESC exit simulation

		else if(keyPressed == 13){//If keypressed is ENTER send scenario to path planner
			obstacleScenario.scenario = newScenario;
			PathPlaner planer;
			bool success;
			success = planer.findPath(obstacleScenario, initialLocation, targetLocation);
			if(!success) cout << "***NO SUITABLE PATH FOUND***" << endl;
			break;
		}
	}
}

Simulator::~Simulator() {
	// TODO Auto-generated destructor stub
}
