//#include "stdafx.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/opencv.hpp"
#include <stdio.h>
#include "cv.h"
#include "StereoPair.h"
#include "Odometry.h"


using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{

	/*******************************************************************************************************
	 * Main parameters                                                                                     *
	 *******************************************************************************************************/

	// File and folder paths
	string outputFolder		= "/home/alejandro/Desktop/";
	string calibrationFile	= "/home/alejandro/Desktop/Stereo1/calibration_24-Jul-2014.txt";
	string calOutput		= "/home/alejandro/Desktop/Stereo1/calibrationRectified.txt";

	//Stereo camera parameters
	int leftCamID = 2;
	int rightCamID = 1;
	int frameRate = 10;

	//Odometry parameters
	int framesToInitOdometry = 4;


	/********************************************************************************************************
	 * Initialization of the StereoPair object                                                              *
	 *                                                                                                      *
	 * Variable initStereoSuccess will be set to false if the method fails to load the cameras.             *
	 * It is important to check if the camera indexes and frame rate are correct.                           *
	 ********************************************************************************************************/
	bool initStereoSuccess;
	StereoPair stereo(leftCamID, rightCamID, frameRate, initStereoSuccess);
	if(!initStereoSuccess){
		cout << "\n**********FINNISHED WITH CAMERA INITIALIZATION ERROR*********" << endl;
		return 1;
	}
	//TODO: check if folder path is valid
	stereo.setupRectification(calibrationFile, "");	// Init rectification. The second parameter is for storing the calibration output. Will be ignored if empty.

	/*Uncomment following lines if you need to save non calibrated frames or calibrated frames to file*/
	//stereo.saveCalibrationFrames(outputFolder);
	//stereo.saveCalibratedImages(outputFolder);


	/*********************************************************************************************************
	 * Initialization of the Odometry object                                                                 *
	 *                                                                                                       *
	 * The initOdometry method requires the number of iterations required to initialize the reference        *
	 * coordinate system.                                                                                    *
	 * Only the features that survive all the iterations will be used to calculate the first 3D points.      *
	 * A minimum of 3 points is needed for a successful initialization. Setting the iteration count too      *
	 * high may result in very few points.                                                                   *
	 *********************************************************************************************************/

	Odometry odometry(stereo);
	odometry.initOdometry(framesToInitOdometry, Odometry::PROMPT_REPEAT); //If init fails to find enough points, it will prompt if you wish to repeat it or abort


	/*********************************************************************************************************
	 * Main loop                                                                                             *
	 *********************************************************************************************************/

	cout << "press ESC to exit" << endl;
	bool doLoop = false;
	while(doLoop){
		if(stereo.updateRectifiedPair()){ //function returns true if both frames are received correctly
			odometry.updateQueue(false);
		}

		//Program stops when user presses ESC key
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
