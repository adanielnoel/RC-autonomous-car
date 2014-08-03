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
using namespace sp;

int main(int argc, char* argv[])
{
	// Parameters
	string outputFolder		= "/home/alejandro/Desktop/Debug/";
	string calibrationFile	= "/home/alejandro/Desktop/Stereo1/calibration_24-Jul-2014.txt";
	string calOutput		= "/home/alejandro/Desktop/Stereo1/calibrationRectified.txt";
	int leftImageID = 1;
	int rightImageID = 2;
	int frameRate = 20;

	StereoPair stereo(leftImageID, rightImageID, frameRate);
	Odometry odometry = Odometry();
	// Init rectification
	stereo.setupRectification(calibrationFile);

//////Uncomment next line if you need to save chess board calibration frames///////
	//stereo.saveCalibrationFrames(outputFolder);

	cout << "press ESC to exit" << endl;

	while(1){

		if(stereo.updateRectifiedPair()){ //function returns true if both frames are received correctly
			odometry.updateQueue(stereo.getMainImg(), true);
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
