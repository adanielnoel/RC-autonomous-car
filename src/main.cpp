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
	// Parameters
	string outputFolder		= "/home/alejandro/Desktop/";
	string calibrationFile	= "/home/alejandro/Desktop/Stereo1/calibration_24-Jul-2014.txt";
	string calOutput		= "/home/alejandro/Desktop/Stereo1/calibrationRectified.txt";
	int leftImageID = 2;
	int rightImageID = 1;
	int frameRate = 20;

	StereoPair stereo(leftImageID, rightImageID, frameRate);
	stereo.setupRectification(calibrationFile, "");	// Init rectification. The second parameter is for storing the calibration output. Will be ignored if empty.
	//stereo.calibrateCoefs();
//////Uncomment next line if you need to save chess board calibration frames///////
	//stereo.saveCalibrationFrames(outputFolder);
	//stereo.saveCalibratedImages(outputFolder);
	Odometry odometry = Odometry(stereo);
	odometry.initOdometry(11);



	cout << "press ESC to exit" << endl;

	while(1){
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

    return 0;
}
