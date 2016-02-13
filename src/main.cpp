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




//  3rd party libraries
#include <boost/filesystem.hpp>
#include "opencv2/opencv.hpp"
//#include "cv.h"   //To compile on Linux???
//#include "stdafx.h" //To compile on Windows
#include "DUOLib.h"
//  Project files
#include "StereoPair.h"
#include "Odometry.h"

//#include "DUO3D_camera.h"
#include "Simulator.h"
#include "arduino_serial_lib.c"

//  Standard libraries
#include <stdio.h>

// File and folder paths
#ifdef _WIN32 // Includes 32 and 64 bit versions

#else
const string DATA_DIRECTORY = "/usr/local/var/lib/autonomousCar/";
const string ARDUINO_SERIAL_PORT = "/dev/tty.usbmodem411";
#endif

const int ARDUINO_BAUDRATE = 9600;


void testArduino() {
    //  Opening communication
    int fd = -1;
    fd = serialport_init(ARDUINO_SERIAL_PORT.c_str(), ARDUINO_BAUDRATE);
    if( fd==-1 ){
        cout << "ARDUINO ERROR: Couldn't open port" << endl;
        exit(EXIT_FAILURE);
    }
    serialport_flush(fd);
    
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


void mainLoop(StereoPair &stereoCam) {
    float scenWidth = 3.0;  // meters
    float scenDepth = 2.0;  // meters
    float squareSize = 0.1; // meters
    //ObstacleScenario obstacleScenario(scenWidth, scenDepth, squareSize);
    
    Simulator simulator = Simulator(scenWidth, scenDepth, squareSize, Simulator::TYPE_AVOIDANCE, 1200, DATA_DIRECTORY);
    simulator.scenario.regionOfInterest = Rect(20, 200, WIDTH-20, 5);  //Region of the disparity map to convert into a grid obstacle map
    
    while(true){
        ////////////////Camera update////////////////////
        stereoCam.updateImages();
        stereoCam.updateDisparityImg();
        stereoCam.updateImage3D();
        /////////////Obstacle map update/////////////////
        Mat image3D = stereoCam.image3D;
        //imshow("image3D", image3D);
        //waitKey(0);
        bool obstaclesDetected = false;
        simulator.scenario.populateScenario(image3D, obstaclesDetected);
        ///////////Display internal update///////////////
        Mat display = simulator.drawScenario(simulator.scenario.points);
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
        int keyPressed = waitKey(10);
        if( keyPressed== 27) {
            destroyWindow("Simulator");
            waitKey(1);
            return;
        }
    }
}


using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
    StereoPair stereoCam(WIDTH, HEIGHT, FPS, DATA_DIRECTORY);
    bool quit = false;
    // Print options menu
    cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
    cout << "Please write the desired command and press the ENTER key." << endl;
    cout << "The following is a list of commands and the part of the program they activate:\n" << endl;
    cout << "sr  : Show rectified images" << endl;
    cout << "su  : Show unrectified images" << endl;
    cout << "in  : invert camera" << endl;
    cout << "at  : autotune exposure" << endl;
    cout << "cal : calibrate stereo camera" << endl;
    cout << "sd  : Show disparity map" << endl;
    cout << "sim : Launch path planner simulator" << endl;
    cout << "odo : Launch feature tracking (current progress in odometry)" << endl;
    cout << "loop: Start main loop" << endl;
    cout << "q   : Exit" << endl;
    while (!quit) {
        bool commandRecognised = false;
        while(!commandRecognised) {
            cout << "\rCommand: ";
            string command;
            cin >> command;
            if (command == "sr") {
                stereoCam.rectifyImages(true);
                stereoCam.displayImages(true /*drawLines*/);
                commandRecognised = true;
            }
            else if (command == "su") {
                stereoCam.rectifyImages(false);
                stereoCam.displayImages(true /*drawLines*/);
                commandRecognised = true;
            }
            else if (command == "in") {
                stereoCam.flipUpsideDown();
                commandRecognised = true;
            }
            else if (command == "at") {
                stereoCam.autoTuneExposure();
                commandRecognised = true;
            }
            else if (command == "cal") {
                stereoCam.calibrate();
                commandRecognised = true;
            }
            else if (command == "sd") {
                stereoCam.displayDisparityMap();
                commandRecognised = true;
            }
            else if (command == "sim") {
                float width = 3.0;       //In meters
                float depth = 2.0;       //In meters
                float squareSize = 0.1; //In meters
                int windowWidth = 800;
                Simulator simulator(width, depth, squareSize, Simulator::TYPE_AVOIDANCE, windowWidth, DATA_DIRECTORY);
                simulator.runSimulation();
                commandRecognised = true;
            }
            else if (command == "odo") {
                Odometry odometry(stereoCam);
                odometry.showLRMatches();
                commandRecognised = true;
            }
            else if (command == "loop"){
                mainLoop(stereoCam);
            }
            else if (command == "q") {
                quit = true;
                commandRecognised = true;
            }
            else {
                cout << "\nThe entered command was not recognised." << endl;
            }
        }
    }
    // cout << getBuildInformation() << endl; // Print OpenCV build info

    return 0;
}
