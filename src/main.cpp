//#include "stdafx.h" //To compile on Windows
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/opencv.hpp"
#include <stdio.h>
#include <sys/stat.h>
//#include "cv.h"   //To compile on Linux???
#include "StereoPair.h"
#include "Odometry.h"
#include "Simulator.h"


using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
//Contraseña ordenador on-board: adaspwd
	/*******************************************************************************************************
	 * Main parameters                                                                                     *
	 *******************************************************************************************************/

	// File and folder paths
	string OUTPUT_FOLDER    = "/home/alejandro/Documents/eclipse_workspace/cv_1/Data/";
	string CALIBRATION_FILE = "/home/alejandro/Documents/eclipse_workspace/cv_1/Data/stereo_calibration_parameters.yml";

	//Main options
	bool DO_LOOP = false; //This enables/disables the main loop

	//Stereo camera parameters
	int STEREOCAM_LEFT_ID = 2;
	int STEREOCAM_RIGHT_ID = 1;
	int STEREOCAM_FRAME_RATE = 20;

	//Stereo camera options
	bool STEREOCAM_INIT = false;
	bool STEREOCAM_CALIBRATE = false;
	bool STEREOCAM_SHOW_RECTIFICATION = false;
	bool STEREOCAM_SAVE_UNCALIBRATED_PAIRS = false;
	bool STEREOCAM_SAVE_CALIBRATED_PAIRS = false;
	bool STEREOCAM_SHOW_DISPARITY_MAP = false;

	//Odometry options
	bool ODOMETRY_INIT = false;
	bool ODOMETRY_SHOW_MATCHES = true;

	//Path planning simulator options
	bool PATHSIM_INIT = true;
    bool PATHSIM_RUN_AVOIDANCE = true;
    bool PATHSIM_RUN_NAVIGATION = false;


	/********************************************************************************************************
	 * Initialization of the StereoPair object                                                              *
	 *                                                                                                      *
	 * Variable initStereoSuccess will be set to false if the method fails to load the cameras.             *
	 * It is important to check if the camera indexes and frame rate are correct.                           *
	 ********************************************************************************************************/
	StereoPair stereo;
	if(STEREOCAM_INIT){
		bool initStereoSuccess;
		stereo = StereoPair(STEREOCAM_LEFT_ID, STEREOCAM_RIGHT_ID, STEREOCAM_FRAME_RATE, CALIBRATION_FILE, initStereoSuccess);
		if(!initStereoSuccess){
			cout << "\n**********FINNISHED WITH CAMERA INITIALIZATION ERROR*********" << endl;
			return 1;
		}

		if(STEREOCAM_SHOW_RECTIFICATION)        stereo.RectificationViewer(OUTPUT_FOLDER);
		if(STEREOCAM_CALIBRATE)                 stereo.calibrate(true, OUTPUT_FOLDER);
		if(STEREOCAM_SAVE_UNCALIBRATED_PAIRS)   stereo.saveUncalibratedStereoImages(OUTPUT_FOLDER);
		if(STEREOCAM_SAVE_CALIBRATED_PAIRS)     stereo.saveCalibratedStereoImages(OUTPUT_FOLDER);
		if(STEREOCAM_SHOW_DISPARITY_MAP)        stereo.displayDisparityMap(false, OUTPUT_FOLDER);
	}


	/*********************************************************************************************************
	 * Initialization of the Odometry object                                                                 *
	 *                                                                                                       *
	 * The initOdometry method requires the number of iterations required to initialize the reference        *
	 * coordinate system.                                                                                    *
	 * Only the features that survive all the iterations will be used to calculate the first 3D points.      *
	 * A minimum of 3 points is needed for a successful initialization. Setting the iteration count too      *
	 * high may result in very few points.                                                                   *
	 *********************************************************************************************************/
	Odometry odometry;
	if(ODOMETRY_INIT){
		odometry = Odometry(stereo);
		if(ODOMETRY_SHOW_MATCHES) odometry.showLRMatches();
	}

	/*********************************************************************************************************
	 * Path planning simulator                                                                               *
	 ********************************************************************************************************/
	if(PATHSIM_INIT){
        if (PATHSIM_RUN_AVOIDANCE) {
            float depth = 4;       //In meters
            float fov = 130;        //In meters
            float squareSize = 0.25; //In meters
            Size windowSize(800, 0);//This is orientative and only the with will be considered
            Simulator simulator(depth, fov, Simulator::TYPE_AVOIDANCE, squareSize, windowSize);
            simulator.runSimulation();
        }
        else if (PATHSIM_RUN_NAVIGATION){
            float width = 20;       //In meters
            float depth = 10;       //In meters
            float squareSize = 0.8; //In meters
            Size windowSize(800, 0);//This is orientative and only the with will be considered
            Simulator simulator(width, depth, Simulator::TYPE_NAVIGATION, squareSize, windowSize);
            simulator.runSimulation();
        }
	}

	/*********************************************************************************************************
	 * Main loop                                                                                             *
	 *********************************************************************************************************/

	while(DO_LOOP){
		if(ODOMETRY_INIT)
			odometry.updateOdometry();

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
