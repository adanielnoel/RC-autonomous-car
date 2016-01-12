/*
 * StereoPair.h
 *
 *  Created on: Jul 26, 2014
 *      Author: Alejandro Daniel Noel
 *   Objective:	This class will group all image based computations, such as:
 *   - Image rectification
 *   - Depth map generation
 */

#ifndef STEREOPAIR_H_
#define STEREOPAIR_H_

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/opencv.hpp"


#include "pcl/common/common_headers.h"
#include "pcl/io/pcd_io.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "boost/thread/thread.hpp"


#include <stdio.h>
#include <DUOLib.h>
//#include "cv.h"


using namespace cv;
using namespace std;

struct Rectification
{
	Mat rmap[2][2];
	Mat K;
	double B;
};

class StereoPair {
private:
	//Atributes
    # ifdef DUO3D
    static DUOInstance duoCam;      // DUO3D camera instance
    static PDUOFrame   duoFrame;      // DUO3D stereo frame
    # else
    struct Webcam {
        VideoCapture	left;
        VideoCapture	right;
    } webcam;
    # endif
    bool    useColorImages = false;
    bool    flipped = false;
    bool    swapped = false;
    bool    canRectify = false;
    
    
	Rectification	rectification;			// Rectification maps
	StereoSGBM		sgbm;			// Disparity computation method
	Mat				dsp;			// Disparity map (not normalized)
	Mat				img3D;			// Depth map
	Mat				Q;              // camera matrix from stereoRectify(..., Q, ...);
    string          outputDirectory;
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3D;
    
public:
    string          calibration_parametersFile = "/Users/alejandrodanielnoel1/Documents/XCode projects/Autonomous_Car/data/stereo_calibration_parameters.xml";
    Size            calibration_boardSize = Size(9, 6);
    float           calibration_squareSize = 0.022;
    int             calibration_numberOfImages = 16;
    ///////Public class constants/////////
    bool            cameraIsUpsideDown;
    Mat				leftImage;			// Rectified left image
    Mat				rightImage;			// Rectified right image
    
	//Constructors and destructors
	StereoPair();			//TODO: default constructor does nothing!

    // New stuff
    void updateImages(bool rectify);
    void displayImages(bool rectified, bool drawLines);
    
	//Initialization methods
	void setupRectification(String calibrationFile = "");
	void setupDisparity();

	//Functions
//	bool updateRectifiedPair();
    void updateDisparityImg(float scaleFactor);
	void updateImg3D();
    void resizeImages(float scaleFactor);
	Mat glueTwoImagesHorizontal(Mat Img1, Mat Img2);
	Mat glueTwoImagesVertical(Mat Img1, Mat Img2);

	//Utilities
	void saveUncalibratedStereoImages(string outputFolder);		//on 's' key press saves stereo images. Useful to get chess board images.
	void saveCalibratedStereoImages(string outputFolder);		//on 's' key press saves rectified images.
	void displayDisparityMap(bool showImages = false, string outputFolder = "", bool useRectifiedImages = true);
	void calibrate(String outputFile, string outputFolder = "");	//Calibrate camera intrinsics and extrinsics
    Point3f getPixel3Dcoords(int pixX, int pixY, double disp);
    Mat reprojectTo3D(Mat disp);
    void run3DVisualizer();

	//Get methods
	Mat getDisparityImg();
	Mat getImg3D();
	Mat getDisparityImgNormalised();
};


#endif /* STEREOPAIR_H_ */
