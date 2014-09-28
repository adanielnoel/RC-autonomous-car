//#include "stdafx.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/opencv.hpp"
#include "boost/filesystem.hpp"
#include <stdio.h>
#include <sys/stat.h>
#include "cv.h"
#include "StereoPair.h"
#include "Odometry.h"
#include "Simulator.h"


using namespace cv;
using namespace std;
using namespace boost::filesystem;

int main(int argc, char* argv[])
{
//Contraseña ordenador on-board: adaspwd
	/*******************************************************************************************************
	 * Main parameters                                                                                     *
	 *******************************************************************************************************/

	// File and folder paths
	path outputFolder		= "/home/alejandro/Documents/eclipse_workspace/cv_1/Data/";
	path calibrationFile	= "/home/alejandro/Documents/eclipse_workspace/cv_1/Data/stereo_calibration_parameters.yml";

	//Stereo camera parameters
	int leftCamID = 2;
	int rightCamID = 1;
	int frameRate = 20;

	//Main options
	bool doLoop = false; //This enables/disables the main loop

	//Stereo camera options
	bool init_stereo_camera = false;
	bool calibrate_camera = false;
	bool show_rectification = false;
	bool save_uncalibrated_stereo_pairs = false;
	bool save_calibrated_stereo_pairs = false;
	bool show_disparity_map = false;

	//Odometry options
	bool init_odometry = false;
	bool show_Left_Right_matches = true;

	//Path planning simulator options
	bool init_simulator = true;
	bool run_simulation = true;

	/********************************************************************************************************
	 * Initial checks                                                                                       *
	 ********************************************************************************************************/

	if(!exists(calibrationFile)){
		cout << "WARNING: The given calibration file path doesn't exist. File must end with .yml extension." << endl;
		cout << "Would you like to calibrate the camera (y/n)?" << endl;
		string response;
		cin >> response;
		if(response != "n" || response != "N") calibrate_camera = true;
	}
	else if(!is_regular_file(calibrationFile)){
		if(is_directory(calibrationFile)){
			cout << "ERROR: the given calibration file path is a directory" << endl;
		}
		else cout << "ERROR: the given calibration file path is invalid" << endl;
		return 1;
	}

	if(!exists(outputFolder)) cout << "ERROR: Output folder path doesn't exist." << endl;
	else if(!is_directory(outputFolder)) cout << "ERROR: Output folder must be a directory" << endl;

	/********************************************************************************************************
	 * Initialization of the StereoPair object                                                              *
	 *                                                                                                      *
	 * Variable initStereoSuccess will be set to false if the method fails to load the cameras.             *
	 * It is important to check if the camera indexes and frame rate are correct.                           *
	 ********************************************************************************************************/
	StereoPair stereo;
	if(init_stereo_camera){
		bool initStereoSuccess;
		stereo = StereoPair(leftCamID, rightCamID, frameRate, calibrationFile.string(), initStereoSuccess);
		if(!initStereoSuccess){
			cout << "\n**********FINNISHED WITH CAMERA INITIALIZATION ERROR*********" << endl;
			return 1;
		}

		if(show_rectification)				stereo.RectificationViewer();
		if(calibrate_camera)				stereo.calibrate(true);
		if(save_uncalibrated_stereo_pairs)	stereo.saveUncalibratedStereoImages(outputFolder.string());
		if(save_calibrated_stereo_pairs)	stereo.saveCalibratedStereoImages(outputFolder.string());
		if(show_disparity_map)				stereo.displayDisparityMap();
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
	if(init_odometry){
		odometry = Odometry(stereo);
		if(show_Left_Right_matches)	odometry.showLRMatches();
	}

	/*********************************************************************************************************
	 * Path planning simulator                                                                               *
	 ********************************************************************************************************/
	if(init_simulator){
		float depth = 10;
		float fov = 130;
		float squareSize = 0.8;
		Size windowSize(800, 0); //This is orientative and only the with will be considered
		Simulator simulator(depth, fov, squareSize, windowSize);

		if(run_simulation){
			simulator.runSimulation();
		}
	}

	/*********************************************************************************************************
	 * Main loop                                                                                             *
	 *********************************************************************************************************/

	while(doLoop){
		if(init_odometry)
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
