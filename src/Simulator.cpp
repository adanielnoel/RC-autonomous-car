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
	colorShadow = Scalar(235, 235, 235);
	colorGrid = Scalar(150, 150, 150);
	colorMessage = Scalar(20, 20, 100);
	colorTargetDirection = Scalar(50, 155, 150);
}

void Simulator::runSimulation(){
	// Initialise an empty scenario
	ObstacleScenario obstacleScenario;
	float targetDirection;

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
	for(int i=0; i<=verticalLines; i++){
		int XOffset = (i+1)*squarePixelSize+i;
		line(scenarioDisplay, Point(XOffset, 0), Point(XOffset, windowHeight-1), colorGrid);
	}
	for(int i=0; i<=horizontalLines; i++){
		int YOffset = (i+1)*squarePixelSize+i;
		line(scenarioDisplay, Point(0, YOffset), Point(windowWidth-1, YOffset), colorGrid);
	}
	 //Create a window
	namedWindow("My Window", 1);
	MouseData mouse;
	Point2i lastClick;
	//set the callback function for any mouse event
	setMouseCallback("My Window", CallBackFunc, &mouse);

	Mat displayCopy;
	int selectionMode = 0; //0 to select squares, 1 to select direction

	while(1){
		imshow("My Window", scenarioDisplay);
		switch(selectionMode){
			case 0:		//Select obstacle squares
				if(mouse.mouseEventType == EVENT_LBUTTONDOWN){
					Point clickedPixel = mouse.p;
					if(clickedPixel.x != lastClick.x && clickedPixel.y != lastClick.y){
						//cout << "New x: " << clickedPixel.x << " | New y: " << clickedPixel.y << endl;
						int mx = clickedPixel.x; int my = clickedPixel.y;
						int sqX = ceil((mx+1)/(squarePixelSize+1));
						int sqY = ceil((my+1)/(squarePixelSize+1));
						newScenario.at(sqX).at(sqY) = 1;
						Point rect0 = Point(sqX*squarePixelSize + sqX, sqY*squarePixelSize + sqY);
						Point rect1 = Point(sqX*squarePixelSize + sqX + squarePixelSize-1, sqY*squarePixelSize + sqY + squarePixelSize-1);
						rectangle(scenarioDisplay, rect0, rect1, colorOccupied, CV_FILLED);
						for(int j=0; j<sqY; j++){
							Point srect0 = Point(sqX*squarePixelSize + sqX, j*squarePixelSize + j);
							Point srect1 = Point(sqX*squarePixelSize + sqX + squarePixelSize - 1, j*squarePixelSize + j + squarePixelSize - 1);
							rectangle(scenarioDisplay, srect0, srect1, colorShadow, CV_FILLED);
						}
						cout << "sqX: " << sqX << "  |  sqY: " << sqY << endl;
						lastClick = clickedPixel;
					}
				}
			break;

			case 1:		//Select target direction
			{
				if(mouse.mouseEventType == EVENT_LBUTTONDOWN)	selectionMode = 2;
				Mat dirSelect = displayCopy.clone();
				Point C = Point(windowWidth/2, windowHeight-1);
				float deltaY = abs(mouse.p.y-C.y);
				float deltaX = mouse.p.x - windowWidth/2;
				targetDirection = atan(deltaY/deltaX); //Return angle in radians
				if(targetDirection < 0) targetDirection += M_PI;
				float lineLength = 300;
				deltaX = windowWidth/2 + lineLength*cos(targetDirection);
				deltaY = windowHeight - lineLength*sin(targetDirection);
				targetDirection *=  180 / M_PI; //convert to degrees
				Point L = Point(deltaX, deltaY);
				line(dirSelect, C, L, colorTargetDirection, 3);
				if(mouse.mouseEventType == EVENT_LBUTTONDOWN){
					displayCopy = dirSelect.clone();
					selectionMode = 2;
				}
				else scenarioDisplay = dirSelect;
			}
			break;

			case 2:		//Display prompt
				Mat displayMessage = displayCopy.clone();
				putText(displayMessage, "Press ENTER to generate path", Point(80, windowHeight-50), FONT_HERSHEY_DUPLEX, 1, colorMessage);
				scenarioDisplay = displayMessage;
			break;
		}

		// Wait until user press some key
		int keyPressed = waitKey(30);
		if( keyPressed == 27)	break;					//If keypressed is ESC exit simulation
		else if( keyPressed == 100){					//If keypressed is 'd' enter select direction mode
			selectionMode = 1;
			displayCopy = scenarioDisplay.clone();
		}
		else if( selectionMode == 2 && keyPressed == 13){//If keypressed is ENTER send scenario to path planner
			obstacleScenario.scenario = newScenario;
			PathPlaner planer;
			bool success;
			success = planer.findPath(obstacleScenario, targetDirection);
			if(!success) cout << "***NO SUITABLE PATH FOUND***" << endl;
			break;
		}
	}
}

Simulator::~Simulator() {
	// TODO Auto-generated destructor stub
}
