//
//
//  Created on: Jul 26, 2014
//     Author: Alejandro Daniel Noel
//     Page: http://futuretechmaker.com
//

#define WIDTH	640
#define HEIGHT	480
#define FPS		30
#define DUO3D

//#include "stdafx.h" //To compile on Windows
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/opencv.hpp"
#include <stdio.h>
#include <sys/stat.h>
// Include DUO API header file
#include "DUOLib.h"
//#include "cv.h"   //To compile on Linux???
#include "StereoPair.h"
#include "Odometry.h"
#include "Simulator.h"
#include "arduino_serial_lib.c"
#include "Globals.h"

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
    // cout << getBuildInformation() << endl; // Print OpenCV build info
    //performTests();
    //return 0;
    StereoPair stereoCam = initStereo();
    float scenWidth = 3.0;  // meters
    float scenDepth = 2.0;  // meters
    float squareSize = 0.1; // meters
    //ObstacleScenario obstacleScenario(scenWidth, scenDepth, squareSize);
    
    Simulator simulator = Simulator(scenWidth, scenDepth, squareSize, Simulator::TYPE_AVOIDANCE, 1200);
    simulator.scenario.regionOfInterest = Rect(20, 200, WIDTH-20, 5);  //Region of the disparity map to convert into a grid obstacle map
    
	while(DO_MAIN_LOOP){
        ////////////////Camera update////////////////////
        stereoCam.updateImages(true /*rectify*/);
        stereoCam.updateDisparityImg();
        stereoCam.updateImage3D();
        /////////////Obstacle map update/////////////////
        Mat image3D = stereoCam.image3D;
        //imshow("image3D", image3D);
        //waitKey(0);
        bool obstaclesDetected = false;
        simulator.scenario.populateScenario(image3D, obstaclesDetected);
        ///////////Display internal update///////////////
        Mat display = simulator.drawScenario();
        ////////////Avoidance path update////////////////
        float newCurveRadius = 10000.0; // In meters. 10000 means infinity (straight path)
        if (obstaclesDetected) {
            PathPlaner planer;
            newCurveRadius = planer.findAvoidancePath(simulator.scenario, 10000, display, simulator.squarePixelSize);
        }
        ///////////////Display update////////////////////
        imshow("Simulator", display);
        
        //////////////Display point cloud////////////////
        //if(obstaclesDetected){
            //stereoCam.run3DVisualizer();
        //}
        
        //Program stops when user presses ESC key
        //If no window is open, this won't work
        int keyPressed = waitKey(30);
        if( keyPressed== 27)
        {
            cout << "Exiting program" << endl;
            break;
        }
	}
    
	cout << "\n*****FINNISHED*****" << endl;
    return 0;
}


void realTimeApp(){
}
