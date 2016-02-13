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


#include "opencv2/opencv.hpp"
#include "pcl/common/common_headers.h"
#include "pcl/io/pcd_io.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "boost/thread/thread.hpp"
#include "DUOLib.h"

//#include "cv.h"

#ifndef DUO3D
#define DUO3D
#endif

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
    int exposure;       // From 0 to 100
    int ledIntensity;   // From 0 to 100
    # else
    struct Webcam {
        VideoCapture	left;
        VideoCapture	right;
    } webcam;
    # endif
    bool    useColorImages;
    bool    flippedUpsideDown;
    bool    swapped;
    bool    canRectify;
    bool    rectify;
    int     imageWidth;
    int     imageHeight;
    
	Rectification	rectification;			// Rectification maps
	StereoSGBM		semiGlobalBlobMatch;	// Disparity computation method
	Mat				Q;                      // camera matrix from stereoRectify(..., Q, ...);
    
    string          calibration_ParametersFileName;
    string          sgbm_ParametersFileName;
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3D;
    
public:
    ///////Public objects and variables/////////
    string          dataDirectory;
    // Calibration parameters
    Size            calibration_boardSize;
    float           calibration_squareSize;
    int             calibration_numberOfImages;
    
    Mat				leftImage;			// Rectified left image
    Mat				rightImage;			// Rectified right image
	Mat				disparityMap;       // Disparity map (not normalized)
	Mat				image3D;            // Depth map
    
	// Constructors and destructors
    StereoPair();
	StereoPair(int width, int height, int fps, string _dataDirectory);

    // Updating and displaying
    void rectifyImages(bool doRectify);
    void updateImages();
    void updateDisparityImg(float scaleFactor = 1.0);
    void updateImage3D();
    
    void displayImages(bool drawLines);
    void displayDisparityMap();
    void displayImage3D();
    
	// Initialization methods
	void setupRectification();
    void setupDisparityParameters();
    void saveDisparityParameters();

	// Utilities
	void calibrate();	//Calibrate camera intrinsics and extrinsics
    Point3f getPixel3Dcoords(int pixX, int pixY, double disp);      // TODO: implement
    void resizeImages(float scaleFactor);
    Mat glueTwoImagesHorizontal(Mat Img1, Mat Img2);
    Mat glueTwoImagesVertical(Mat Img1, Mat Img2);
    bool saveImage(Mat image, string imageName, string outputDirectory);
    void flipUpsideDown();
#ifdef DUO3D
    void autoTuneExposure();
#endif

	// Get methods
	Mat getDisparityImageNormalised();
};


#endif /* STEREOPAIR_H_ */
