//#include "stdafx.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/opencv.hpp"
#include <stdio.h>
#include <sys/stat.h>
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
	string outputFolder		= "/home/alejandro/Documents/eclipse_workspace/cv_1/Data/";
	string calibrationFile	= "/home/alejandro/Documents/eclipse_workspace/cv_1/Data/stereo_calibration_parameters.yml";
	string calOutput		= "/home/alejandro/Desktop/Stereo1/calibrationRectified.txt";

	//Stereo camera parameters
	int leftCamID = 2;
	int rightCamID = 1;
	int frameRate = 0;

	//Odometry parameters
	int framesToInitOdometry = 4;


	/********************************************************************************************************
	 * Initialization of the StereoPair object                                                              *
	 *                                                                                                      *
	 * Variable initStereoSuccess will be set to false if the method fails to load the cameras.             *
	 * It is important to check if the camera indexes and frame rate are correct.                           *
	 ********************************************************************************************************/
	bool initStereoSuccess;
	StereoPair stereo(leftCamID, rightCamID, frameRate, calibrationFile, initStereoSuccess);
	if(!initStereoSuccess){
		cout << "\n**********FINNISHED WITH CAMERA INITIALIZATION ERROR*********" << endl;
		return 1;
	}
	//TODO: check if folder path is valid

	/*Uncomment following lines to use these useful tools*/
	//stereo.RectificationViewer();
	//stereo.calibrate(true);
	//stereo.saveUncalibratedStereoImages(outputFolder);
	//stereo.saveCalibratedStereoImages(outputFolder);
	//stereo.displayImagePairAndDepthMap(false);



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

	/*Uncomment following lines to use these useful tools*/
	odometry.showLRMatches();

	/*********************************************************************************************************
	 * Main loop                                                                                             *
	 *********************************************************************************************************/

	cout << "press ESC to exit" << endl;
	bool doLoop = false; //For current tests the main loop is not used
	while(doLoop){
		if(stereo.updateRectifiedPair()){ //function returns true if both frames are received correctly
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
