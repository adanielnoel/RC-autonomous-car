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

#define DUO3D

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
    int     imageWidth;
    int     imageHeight;
    
	Rectification	rectification;			// Rectification maps
	StereoSGBM		semiGlobalBlobMatch;	// Disparity computation method
	Mat				Q;                      // camera matrix from stereoRectify(..., Q, ...);
    
    string          calibration_ParametersFileName = "stereo_calibration_parameters.xml";
    string          sgbm_ParametersFileName = "semiglobal_block_match_parameters.xml";
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3D;
    
public:
    ///////Public objects and variables/////////
    string          outputDirectory = "/Users/alejandrodanielnoel1/Documents/0 Projects/1 BRAIN/CV/SDC Code/data/";
    // Calibration parameters
    Size            calibration_boardSize = Size(9, 6);
    float           calibration_squareSize = 0.022;
    int             calibration_numberOfImages = 16;
    
    Mat				leftImage;			// Rectified left image
    Mat				rightImage;			// Rectified right image
	Mat				disparityMap;       // Disparity map (not normalized)
	Mat				image3D;            // Depth map
    
	// Constructors and destructors
    StereoPair();
	StereoPair(int width, int height, int fps);

    // Updating and displaying
    void updateImages(bool rectify);
    void updateDisparityImg(float scaleFactor = 1.0);
    void updateImage3D();
    
    void displayImages(bool rectified, bool drawLines);
    void displayDisparityMap();
    void displayImage3D();
    
	// Initialization methods
	void setupRectification();
    void setupDisparityParameters();
    void saveDisparityParameters();

	// Utilities
	void calibrate(String outputFile, string outputFolder = "");	//Calibrate camera intrinsics and extrinsics
    Point3f getPixel3Dcoords(int pixX, int pixY, double disp);      // TODO: implement
    void resizeImages(float scaleFactor);
    Mat glueTwoImagesHorizontal(Mat Img1, Mat Img2);
    Mat glueTwoImagesVertical(Mat Img1, Mat Img2);
    bool saveImage(Mat image, string imageName, string outputDirectory);
    void flipUpsideDown();

	// Get methods
	Mat getDisparityImageNormalised();
};


#endif /* STEREOPAIR_H_ */
