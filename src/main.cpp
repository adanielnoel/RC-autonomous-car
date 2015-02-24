//
//
//  Created on: Jul 26, 2014
//     Author: Alejandro Daniel Noel
//     Page: http://futuretechmaker.com
//


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
#include <DUOLib.h>
//#include "cv.h"   //To compile on Linux???
#include "StereoPair.h"
#include "Odometry.h"
#include "Simulator.h"
#include "arduino_serial_lib.c"

#define WIDTH	640
#define HEIGHT	480
#define FPS		30

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
 //	printf("DUOLib Version:       v%s\n", GetLibVersion());
//Contraseña ordenador on-board: adaspwd
	/*******************************************************************************************************
	 * Main parameters                                                                                     *
	 *******************************************************************************************************/
    
    /*ArduinoInterface ardu;
    ardu.sendData();
    return 0;*/
	// File and folder paths
	const string OUTPUT_FOLDER    = "/Users/alejandrodanielnoel/Documents/XCode projects/Autonomous_Car/data/";
	const string CALIBRATION_FILE = "/Users/alejandrodanielnoel/Documents/XCode projects/Autonomous_Car/data/stereo_calibration_parameters.xml";
    const string ARDUINO_SERIAL_PORT = "/dev/tty.usbmodem411";
    const int ARDUINO_BAUDRATE = 9600;

	//Main options
	const bool DO_LOOP = false; //This enables/disables the main loop

	//Stereo camera parameters
	const int STEREOCAM_LEFT_ID = 2;
	const int STEREOCAM_RIGHT_ID = 1;
	const int STEREOCAM_FRAME_RATE = 10;

	//Stereo camera options
	const bool STEREOCAM_INIT = false;
    const bool STEREOCAM_DUO3D = true;
    const bool STEREOCAM_RECTIFY_IMAGES = true;
	const bool STEREOCAM_CALIBRATE = false;
	const bool STEREOCAM_SHOW_RECTIFICATION = false;
	const bool STEREOCAM_SAVE_UNCALIBRATED_PAIRS = false;
	const bool STEREOCAM_SAVE_CALIBRATED_PAIRS = false;
	const bool STEREOCAM_SHOW_DISPARITY_MAP = false;

	//Odometry options
	const bool ODOMETRY_INIT = false;
	const bool ODOMETRY_SHOW_MATCHES = true;

	//Path planning simulator options
	const bool PATHSIM_INIT = false;
    const bool PATHSIM_RUN_AVOIDANCE = true;
    const bool PATHSIM_RUN_NAVIGATION = false;
    
    //Arduino communication options
    const bool ARDUINO_INIT_SERIAL_COMMUNICATION = true;
    const bool ARDUINO_PERFORM_HAND_SHAKE = true;


	/********************************************************************************************************
	 * Initialization of the StereoPair object                                                              *
	 *                                                                                                      *
	 * Variable initStereoSuccess will be set to false if the method fails to load the cameras.             *
	 * It is important to check if the camera indexes and frame rate are correct.                           *
	 ********************************************************************************************************/
	StereoPair stereoCam;
	if(STEREOCAM_INIT){
		bool initStereoSuccess = false;
        if (STEREOCAM_DUO3D) {
            stereoCam = StereoPair(0, 0, 640, 480, STEREOCAM_FRAME_RATE, initStereoSuccess);
        }
        else {
            stereoCam = StereoPair(STEREOCAM_LEFT_ID, STEREOCAM_RIGHT_ID, 640, 480, STEREOCAM_FRAME_RATE, initStereoSuccess);
        }
		if(!initStereoSuccess){
			cout << "\n**********FINNISHED WITH CAMERA INITIALIZATION ERROR*********" << endl;
			return 1;
		}
        bool showImages;
        if(STEREOCAM_CALIBRATE)                 stereoCam.calibrate(CALIBRATION_FILE, OUTPUT_FOLDER);
        if(STEREOCAM_RECTIFY_IMAGES)            stereoCam.setupRectification(CALIBRATION_FILE);
        if(STEREOCAM_SHOW_RECTIFICATION)        stereoCam.rectificationViewer(OUTPUT_FOLDER);
		if(STEREOCAM_SAVE_UNCALIBRATED_PAIRS)   stereoCam.saveUncalibratedStereoImages(OUTPUT_FOLDER);
		if(STEREOCAM_SAVE_CALIBRATED_PAIRS)     stereoCam.saveCalibratedStereoImages(OUTPUT_FOLDER);
		if(STEREOCAM_SHOW_DISPARITY_MAP)        stereoCam.displayDisparityMap(showImages = false, OUTPUT_FOLDER, STEREOCAM_RECTIFY_IMAGES);
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
		odometry = Odometry(stereoCam);
		if(ODOMETRY_SHOW_MATCHES) odometry.showLRMatches();
	}

	/*********************************************************************************************************
	 * Path planning simulator                                                                               *
	 ********************************************************************************************************/
	if(PATHSIM_INIT){
        if (PATHSIM_RUN_AVOIDANCE) {
            float depth = 2;       //In meters
            float fov = 90;        //In meters
            float squareSize = 0.15; //In meters
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
     * Arduino serial communication                                                                          *
     *********************************************************************************************************/
    if (ARDUINO_INIT_SERIAL_COMMUNICATION) {
        int fd = -1;
        fd = serialport_init(ARDUINO_SERIAL_PORT.c_str(), ARDUINO_BAUDRATE);
        if( fd==-1 ){
            cout << "ARDUINO ERROR: Couldn't open port" << endl;
            exit(EXIT_FAILURE);
        }
        serialport_flush(fd);
        
        if (ARDUINO_PERFORM_HAND_SHAKE) {
            // Sending a string
            string helloString = "Hello Arduino!";
            int buf_max = 256;
            char buf[buf_max];
            sprintf(buf, "%s\n", helloString.c_str());
            cout << "- Writing to arduino...";
            int rc = serialport_write(fd, buf);
            if(rc==-1) cout << " error writing" << endl;
            else cout << " done" << endl;
  
            // Receiving a string
            memset(buf,0,buf_max); //Set all buffer to 0
            cout << "- Reading from arduino..." << endl;
            const int timeout = 1000;
            serialport_read_until(fd, buf, '\n', buf_max, timeout);
            string message = buf;
            cout << "Message: " << message << endl;
        }
    }

	/*********************************************************************************************************
	 * Main loop                                                                                             *
	 *********************************************************************************************************/

    float scenWidth = 3.0;
    float scenDepth = 2.0;
    float squareSize = 0.1;
    ObstacleScenario obstacleScenario(scenWidth, scenDepth, squareSize);
    obstacleScenario.stereoPair = stereoCam;
    obstacleScenario.regionOfInterest = Rect(0, 0, 80, 80);
    Simulator simulator = Simulator();
    
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
        stereoCam.updateRectifiedPair();
        stereoCam.updateDisparityImg(0.5, true);
        obstacleScenario.populateScenario(stereoCam.getDisparityImg());
        simulator.displayScenario(obstacleScenario, true);
	}
	cout << "\n*****FINNISHED*****" << endl;
    return 0;
}
