//
//  Globals.h
//  Autonomous_RC_Car
//
//  Created by Alejandro Daniel Noel on 20/05/15.
//
//

#ifndef Autonomous_RC_Car_Globals_h
#define Autonomous_RC_Car_Globals_h

// File and folder paths
const string OUTPUT_FOLDER    = "/Users/alejandrodanielnoel1/Desktop/";
const string CALIBRATION_FILE = "/Users/alejandrodanielnoel1/Documents/XCode projects/Autonomous_Car/data/stereo_calibration_parameters.xml";
const string ARDUINO_SERIAL_PORT = "/dev/tty.usbmodem411";
const int ARDUINO_BAUDRATE = 9600;

//Main options
const bool DO_LOOP = false; //This enables/disables the main loop

//Stereo camera parameters
const int STEREOCAM_LEFT_ID = 2;
const int STEREOCAM_RIGHT_ID = 1;
const int STEREOCAM_FRAME_RATE = 10;

//Stereo camera options
const bool STEREOCAM_INIT = true;
const bool STEREOCAM_CAMERA_IS_INVERTED = true;
const bool STEREOCAM_DUO3D = true;
const bool STEREOCAM_RECTIFY_IMAGES = true;
const bool STEREOCAM_CALIBRATE = false;
const bool STEREOCAM_SHOW_RECTIFICATION = true;
const bool STEREOCAM_SAVE_UNCALIBRATED_PAIRS = false;
const bool STEREOCAM_SAVE_CALIBRATED_PAIRS = false;
const bool STEREOCAM_SHOW_DISPARITY_MAP = true;

//Odometry options
const bool ODOMETRY_INIT = false;
const bool ODOMETRY_SHOW_MATCHES = true;

//Path planning simulator options
const bool PATHSIM_INIT = false;
const bool PATHSIM_RUN_AVOIDANCE = true;
const bool PATHSIM_RUN_NAVIGATION = false;

//Arduino communication options
const bool ARDUINO_INIT_SERIAL_COMMUNICATION = false;
const bool ARDUINO_PERFORM_HAND_SHAKE = true;

StereoPair initStereo(){
    bool initStereoSuccess = false;
    StereoPair sp;
    if (STEREOCAM_DUO3D) {
        // If left and right IDs are equal, then the camera is the DUO3D
        sp = StereoPair(0, 0, 640, 480, STEREOCAM_FRAME_RATE, initStereoSuccess);
    }
    else {
        sp = StereoPair(STEREOCAM_LEFT_ID, STEREOCAM_RIGHT_ID, 640, 480, STEREOCAM_FRAME_RATE, initStereoSuccess);
    }
    if(!initStereoSuccess){
        cout << "\n**********FINNISHED WITH CAMERA INITIALIZATION ERROR*********" << endl;
        exit(EXIT_FAILURE);
    }
    
    if(STEREOCAM_RECTIFY_IMAGES)            sp.setupRectification(CALIBRATION_FILE);
    return sp;
}

void performTests() {
    StereoPair stereoCam;
    if(STEREOCAM_INIT){
        stereoCam = initStereo();
        bool showImages;
        if(STEREOCAM_CAMERA_IS_INVERTED)        stereoCam.cameraIsUpsideDown = true;
        if(STEREOCAM_CALIBRATE)                 stereoCam.calibrate(CALIBRATION_FILE, OUTPUT_FOLDER);
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
            float width = 3.0;       //In meters
            float depth = 2.0;       //In meters
            float squareSize = 0.1; //In meters
            int windowWidth = 800;
            Simulator simulator(width, depth, squareSize, Simulator::TYPE_AVOIDANCE, windowWidth);
            simulator.runSimulation();
        }
        else if (PATHSIM_RUN_NAVIGATION){
            float width = 20;       //In meters
            float depth = 10;       //In meters
            float squareSize = 0.8; //In meters
            int windowWidth = 800;
            Simulator simulator(width, depth, squareSize, Simulator::TYPE_NAVIGATION, windowWidth);
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
}

#endif
