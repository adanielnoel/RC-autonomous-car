/*
 * StereoPair.h
 *
 *  Created on: Jul 26, 2014
 *      Author: Alejandro Daniel Noel
 *   Objective:	This class will group all image based computations, such as:
 *   - Image rectification
 *   - Depth map generation
 */

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

#ifndef STEREOPAIR_H_
#define STEREOPAIR_H_


using namespace cv;
using namespace std;

struct Rectification
{
	Mat rmap[2][2];
	Mat K;
	double B;
};

class StereoPair {

	//Atributes
    int             width;
    int             height;
    int             fps;
    bool            defaultCamera;  // If true, uses the OpenCV VideoCapture class. Otherwise uses the DUO3D API.
	Rectification	recti;			// Rectification maps
	StereoSGBM		sgbm;			// Disparity computation method
	Mat				imgl;			// Rectified left image
	Mat				imgr;			// Rectified right image
	Mat				dsp;			// Disparity map (not normalized)
	Mat				img3D;			// Depth map
	Mat				Q;              // camera matrix from stereoRectify(..., Q, ...);
	String			calibrationFile;//File path to the intrinsic and extrinsic parameters
    bool            rectificationCorrect;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3D;
    
public:
    ///////Public class constants/////////
    static const bool USE_CUSTOM_REPROJECTION_METHOD;
    static const bool USE_OPENCV_REPROJECTION_METHOD;
    bool              cameraIsUpsideDown;
    
    ////////Web-cam///////
    VideoCapture	camL;			// Left camera
    VideoCapture	camR;			// right camera
    /////DUO3D camera/////
    static DUOInstance duoCam;      // DUO3D camera instance
    static PDUOFrame   duoFrame;      // DUO3D stereo frame
    //////////////////////
    
	//Constructors and destructors
	StereoPair();			//TODO: default constructor does nothing!
	StereoPair(int lCamId, int rCamId, int _width, int _height, int camFPS, bool & success);

	//Initialization methods
	void setupRectification(String _calibrationFile = "");
	void setupDisparity();

	//Functions
	Mat rectifyImage(const Mat& unrectifiedImage, const Rectification& recti, bool left);
    bool updateUnrectifiedPair();
	bool updateRectifiedPair();
    void updateDisparityImg(float scaleFactor);
	void updateImg3D(bool useCustomMethod);
    void resizeImages(float scaleFactor);
	Mat glueTwoImagesHorizontal(Mat Img1, Mat Img2);
	Mat glueTwoImagesVertical(Mat Img1, Mat Img2);

	//Utilities
	void saveUncalibratedStereoImages(string outputFolder);		//on 's' key press saves stereo images. Useful to get chess board images.
	void saveCalibratedStereoImages(string outputFolder);		//on 's' key press saves rectified images.
	void displayDisparityMap(bool showImages = false, string outputFolder = "", bool useRectifiedImages = true);
	void rectificationViewer(string outputFolder = "");			//Shows rectified images side to side with horizontal lines.
	void calibrate(String outputFile, string outputFolder = "");	//Calibrate camera intrinsics and extrinsics
    Point3f getPixel3Dcoords(int pixX, int pixY, double disp);
    Mat reprojectTo3D(Mat disp);
    void run3DVisualizer();

	//Get methods
	Mat getDisparityImg();
	Mat getImg3D();
	Mat getDisparityImgNormalised();

	const Mat& getImgr() const {
		return imgr;
	}

	const Mat& getImgl() const {
		return imgl;
	}

};


#endif /* STEREOPAIR_H_ */
