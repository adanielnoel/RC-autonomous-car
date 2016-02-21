//
//
//  Created on: Jul 26, 2014
//     Author: Alejandro Daniel Noel
//     Page: http://futuretechmaker.com
//

#define WIDTH	640
#define HEIGHT	480
#define FPS		30
#define FOV     70
#define DUO3D
#define MAX_DEPTH 2.0
// Scale fator for adjusting the scale of the 3D reconstruction
#define SCALE_FACTOR_3D_RECONSTRUCTION 50.0



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
#include <unistd.h>

// File and folder paths
#ifdef _WIN32 // Includes 32 and 64 bit versions

#elif __APPLE__
const string INSTALL_DIRECTORY = "/usr/local/var/lib/autonomousCar/";
const string DATA_DIRECTORY = "/users/" + string(getlogin()) + "/autonomousCar/"; // user home directory
const string ARDUINO_SERIAL_PORT = "/dev/tty.usbmodem411";
#else
const string INSTALL_DIRECTORY = "/usr/local/var/lib/autonomousCar/";
const string DATA_DIRECTORY = "/home/" + string(getlogin()) + "/autonomousCar/"; // user home directory
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


void launchAvoidanceSimulator(){
    float width = 3.0;       //In meters
    float depth = 2.0;       //In meters
    float squareSize = 0.1; //In meters
    int windowWidth = 800;
    Simulator simulator(width, depth, squareSize, windowWidth, DATA_DIRECTORY);
    simulator.runSimulation();
}


void launchOdometry(StereoPair stereoCam){
    Odometry odometry(stereoCam);
    odometry.showLRMatches();
}


void mainLoop(StereoPair &stereoCam) {
    float scenWidth = 2.0;  // meters
    float scenDepth = 1.0;  // meters
    float squareSize = 0.08; // meters
    int iteration = 0;
    float fieldOfView = -1;
    Simulator simulator = Simulator(scenWidth, scenDepth, squareSize, 1200, DATA_DIRECTORY);
    simulator.scenario.minY = 0.0;
    simulator.scenario.maxY = 0.1;
    simulator.scenario.scaleFactor = SCALE_FACTOR_3D_RECONSTRUCTION;
    
    while(true){
        ////////////////Camera update////////////////////
        stereoCam.updateImages();
        stereoCam.updateDisparityImg();
        stereoCam.updateImage3D();
        if (iteration == 0) fieldOfView = stereoCam.computeFieldOfView();
        /////////////Obstacle map update/////////////////
        Mat image3D = stereoCam.image3D;
        bool obstaclesDetected = false;
        simulator.scenario.populateScenario(image3D, obstaclesDetected);
        ///////////Display internal update///////////////
        Mat display = simulator.drawScenario(simulator.scenario.points, fieldOfView);
        ////////////Avoidance path update////////////////
        float newCurveRadius = 10000.0; // In meters. 10000 means infinity (straight path)
        if (obstaclesDetected) {
            PathPlaner planer;
            newCurveRadius = planer.findAvoidancePath(simulator.scenario, 10000, display, simulator.squarePixelSize);
        }
        ///////////////Display update////////////////////
        imshow("Simulator", display);
        
        int keyPressed = int(char(waitKey(10)));
        if( keyPressed== 27) {
            destroyWindow("Simulator");
            for(int i = 0; i < 10; i++) waitKey(1);
            return;
        }
        iteration++;
    }
}


using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
    boost::filesystem::path dir(DATA_DIRECTORY);
    if(boost::filesystem::create_directory(dir)) {
        std::cout << "Created data folder: " + DATA_DIRECTORY << endl;
        boost::filesystem::copy(boost::filesystem::path(INSTALL_DIRECTORY + "car_top.png"), boost::filesystem::path(DATA_DIRECTORY + "car_top.png"));
        boost::filesystem::copy(boost::filesystem::path(INSTALL_DIRECTORY + "semiglobal_block_match_parameters.xml"), boost::filesystem::path(DATA_DIRECTORY + "semiglobal_block_match_parameters.xml"));
        boost::filesystem::copy(boost::filesystem::path(INSTALL_DIRECTORY + "stereo_calibration_parameters.xml"), boost::filesystem::path(DATA_DIRECTORY + "stereo_calibration_parameters.xml"));
    }
    
    cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
    // Detect the number of threads available
    int threadCount = 0;
    #pragma omp parallel
    {threadCount++;}
    cout << "Number of threads: " << threadCount << endl << endl;
    
    StereoPair stereoCam(WIDTH, HEIGHT, FPS, DATA_DIRECTORY);
    stereoCam.maximumDepth = MAX_DEPTH;
    
    // Print options menu
    
    cout << "Note: windows cannot be closed, ESC key must pe pressed instead." << endl << endl;
    cout << "Please write the desired command and press the ENTER key." << endl;
    cout << "The following is a list of commands and the part of the program they activate:\n" << endl;
    cout << "0  : Show rectified images" << endl;
    cout << "1  : Show unrectified images" << endl;
    cout << "2  : Show disparity map" << endl;
    cout << "3  : Show point cloud" << endl;
    cout << "4  : invert camera" << endl;
    cout << "5  : autotune exposure" << endl;
    cout << "6  : calibrate stereo camera" << endl;
    cout << "7  : Launch path planner simulator" << endl;
    cout << "8  : Launch feature tracking" << endl;
    cout << "9  : Start main loop" << endl;
    cout << "10 : Exit\n" << endl;
    
    bool commandRecognised = false;
    while(!commandRecognised) {
        bool commandRecognised = true;
        cout << "Command: ";
        int command;
        cin >> command;
        switch (command) {
            case 0: //  Show rectified images
                stereoCam.rectifyImages(true);
                stereoCam.displayImages(true /*drawLines*/);
                break;
            case 1:
                stereoCam.rectifyImages(false);
                stereoCam.displayImages(true /*drawLines*/);
                break;
            case 2:
                stereoCam.displayDisparityMap();
                break;
            case 3:
                stereoCam.displayImage3D();
                break;
            case 4:
                stereoCam.flipUpsideDown();
                break;
            case 5:
                stereoCam.autoTuneExposure();
                break;
            case 6:
                stereoCam.calibrate();
                break;
            case 7:
                launchAvoidanceSimulator();
                break;
            case 8:
                launchOdometry(stereoCam);
                break;
            case 9:
                mainLoop(stereoCam);
                break;
            case 10:
                return 0;
                break;
                
            default:
                commandRecognised = false;
                cout << "\nThe entered command was not recognised." << endl;
                break;
        }
    }
    // cout << getBuildInformation() << endl; // Print OpenCV build info

    return 0;
}
