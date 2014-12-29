/*
 * StereoPair.cpp
 *
 *  Created on: Jul 26, 2014
 *      Author: nicolau
 */


#include "StereoPair.h"
#include "DUO3D_camera.h"
#include <DUOLib.h>


StereoPair::StereoPair() {
	// TODO Auto-generated constructor stub

}

StereoPair::StereoPair(int lCamId, int rCamId, int _width, int _height, int camFPS, bool & success){
    width = _width;
    height = _height;
    fps = camFPS;
    if (lCamId != rCamId) {
        defaultCamera = true;
        //Open and configure cameras
        camL = VideoCapture();
        camR = VideoCapture();
        camL.open(lCamId);
        camL.set(CV_CAP_PROP_FPS, camFPS);
        camR.open(rCamId);
        camR.set(CV_CAP_PROP_FPS, camFPS);
        //Check and print camera info
        Size imageSize;
        if(!camL.isOpened()){ cout << "Cannot open left camera" << endl; success = false;}
        if(!camR.isOpened()){ cout << "Cannot open right camera" << endl; success = false;}
        if(camL.isOpened() && camR.isOpened()){ //print frame size
            imageSize.width = camL.get(CV_CAP_PROP_FRAME_WIDTH);
            imageSize.height = camL.get(CV_CAP_PROP_FRAME_HEIGHT);
            cout << "Left camera frame size : " << imageSize.width << " x " << imageSize.height << endl;
            imageSize.width = camR.get(CV_CAP_PROP_FRAME_WIDTH);
            imageSize.height = camR.get(CV_CAP_PROP_FRAME_HEIGHT);
            cout << "Right camera frame size : " << imageSize.width << " x " << imageSize.height << endl;
            success = true;
        }
    }
    else {
        defaultCamera = false;
        if(!OpenDUOCamera(width, height, fps))
        {
            printf("Could not open DUO camera\n");
            success = false;
        }
        // Create OpenCV windows
        cvNamedWindow("Left");
        cvNamedWindow("Right");
        
        // Set exposure and LED brightness
        SetExposure(100);
        SetLed(0);
    }
    
	//Setup Semi Global Block Matching object
	this->setupDisparity();
    success = true;
}

// ////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
// |||||||||||||||||||||||||||||||||||||setupDisparity|||||||||||||||||||||||||||||||||||||||||||
// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//////////////////////////////////////////////////

void StereoPair::setupDisparity(){
	sgbm = StereoSGBM();
	sgbm.SADWindowSize = 11;
	sgbm.numberOfDisparities = 18*16;
	sgbm.preFilterCap = 27;
	sgbm.minDisparity = 0;
	sgbm.uniquenessRatio = 10;
	sgbm.speckleWindowSize = 16;
	sgbm.speckleRange = 60;
	sgbm.disp12MaxDiff = -80;
	sgbm.fullDP = false;
	sgbm.P1 = 8*sgbm.SADWindowSize*sgbm.SADWindowSize;
	sgbm.P2 = 32*sgbm.SADWindowSize*sgbm.SADWindowSize;
}

// ////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
// |||||||||||||||||||||||||||||||||||||setupRectification|||||||||||||||||||||||||||||||||||||||
// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//////////////////////////////////////////////////

void StereoPair::setupRectification(String _calibrationFile)
{
    if(_calibrationFile.empty()){
        cout << "ERROR: Calibration file not found." << endl;
        return;
    }
    calibrationFile = _calibrationFile;
	Mat R1, R2, P1, P2, Q;
	Mat rmap[2][2];
	Rect validRoi[2];

	// Open calibration file
	FileStorage fs(calibrationFile, FileStorage::READ);

	// Read calibration file
	if(!fs.isOpened())
		cout << "Calibration file was not found" << endl;
	else{
		Size imageSize;
		Mat cameraMatrix0, cameraMatrix1, distCoeffs0, distCoeffs1, imgSize, RInitial, TInitial;
		fs["Size"] >> imageSize;
		fs["K1"] >> cameraMatrix0;
		fs["distCoeffs1"] >> distCoeffs0;
		fs["K2"] >> cameraMatrix1;
		fs["distCoeffs1"] >> distCoeffs1;
		fs["R"] >> RInitial;
		fs["T"] >> TInitial;

		// Compute rectification mappings
		stereoRectify(cameraMatrix0, distCoeffs0, cameraMatrix1, distCoeffs1, imageSize, RInitial, TInitial, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, 0, imageSize, &validRoi[0], &validRoi[1]);
		initUndistortRectifyMap(cameraMatrix0, distCoeffs0, R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
		initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

		//"dispToDepthMat" is used to convert disparity images into depth maps.
		dispToDepthMat = Q;

		// Save the rectification mappings
		recti.K = P1;
		recti.B = P2.at<double>(0, 3) / P2.at<double>(0, 0);
		for(int i=0; i<2; i++)
		for(int j=0; j<2; j++)
			recti.rmap[i][j] = rmap[i][j];
	}
}

// ////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
// ||||||||||||||||||||||||||||||||||||||||rectifyImage||||||||||||||||||||||||||||||||||||||||||
// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//////////////////////////////////////////////////

Mat StereoPair::rectifyImage(const Mat& unrectifiedImage, const Rectification& recti, bool left)
{
	Mat rectifiedImage;

	if(left)
		remap(unrectifiedImage, rectifiedImage, recti.rmap[0][0], recti.rmap[0][1], CV_INTER_LINEAR, BORDER_TRANSPARENT);
	else
		remap(unrectifiedImage, rectifiedImage, recti.rmap[1][0], recti.rmap[1][1], CV_INTER_LINEAR, BORDER_TRANSPARENT);

//	rectifiedImage = unrectifiedImage;
	return rectifiedImage;
}

// ////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
// ||||||||||||||||||||||||||||||||glueTwoImagesHorizontal|||||||||||||||||||||||||||||||||||||||
// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//////////////////////////////////////////////////

Mat StereoPair::glueTwoImagesHorizontal(Mat Img1, Mat Img2){
	Mat LR(Img1.rows, Img1.cols+Img2.cols, Img1.type());

	//Place each image horizontally adjacent to each other
	Mat left_roi(LR, Rect(0, 0, Img1.cols, Img1.rows));
	Img1.copyTo(left_roi);
	Mat right_roi(LR, Rect(Img1.cols, 0, Img2.cols, Img2.rows)); // Copy constructor
	Img2.copyTo(right_roi);

	return LR;
}

// ////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
// |||||||||||||||||||||||||||||||||glueTwoImagesVertical||||||||||||||||||||||||||||||||||||||||
// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//////////////////////////////////////////////////

Mat StereoPair::glueTwoImagesVertical(Mat Img1, Mat Img2){
	Mat LR(Img1.rows+Img2.rows, Img1.cols, Img1.type());

	//Place each image vertically adjacent to each other
	Mat top_roi(LR, Rect(0, 0, Img1.cols, Img1.rows));
	Img1.copyTo(top_roi);
	Mat bottom_roi(LR, Rect(0, Img1.rows, Img2.cols, Img2.rows)); // Copy constructor
	Img2.copyTo(bottom_roi);

	return LR;
}

// ////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
// |||||||||||||||||||||||||||||||||||rectificationViewer||||||||||||||||||||||||||||||||||||||||
// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//////////////////////////////////////////////////

void StereoPair::rectificationViewer(string outputFolder)
{
	cout << "Showing rectification" << endl;
	namedWindow("Rectification", CV_WINDOW_AUTOSIZE);
	int frameId = 0;
	while(1){
		this->updateRectifiedPair();
		Mat IL, IR;
        cvtColor(imgl, IL, COLOR_GRAY2BGR);
        cvtColor(imgr, IR, COLOR_GRAY2BGR);

		// set both images horizontally adjacent
		Mat LR = glueTwoImagesHorizontal(IL, IR);

		//draw lines
		for(int h=0; h<LR.rows; h+=25)
		{
			Point pt1(0, h);
			Point pt2(LR.cols, h);
			line(LR, pt1, pt2, CV_RGB(255, 0, 0), 1);
		}

		// Show the image
		imshow("Rectification", LR);

		int keyPressed = waitKey(20);

		if( keyPressed==83 || keyPressed==115)
		{
			cout << "Saving image pairs..." << endl;

			if(!outputFolder.empty()){
				// Create the file names for saving the images
				char fileName[256];
				sprintf(fileName, "%sRectified_LR_%d.png", outputFolder.c_str(), frameId);

				// Write the images
				try {
					imwrite(fileName, LR);
				}
				catch (runtime_error& ex) {
					fprintf(stderr, "Exception converting image to PNG format: %s \n", ex.what());
				}
			}

			// Increment the frame id
			frameId++;
		}
		// Exit if 'esc' key is pressed
        if( keyPressed==27)
		{
			cout << "ESC key is pressed by user" << endl;
			break;
		}
	}
	destroyWindow("Rectification");
}

// ////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
// |||||||||||||||||||||||||||||||||updateUnrectifiedPair||||||||||||||||||||||||||||||||||||||||
// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//////////////////////////////////////////////////

bool StereoPair::updateUnrectifiedPair(){
    Mat newFrameL, newFrameR;
    
    if (defaultCamera) {
        //get new (unrectified) frames
        if (!camL.read(newFrameL)) return false;
        if (!camR.read(newFrameR)) return false;
        //Convert to grey scale
        cvtColor(newFrameL,newFrameL,CV_RGB2GRAY);
        cvtColor(newFrameR,newFrameR,CV_RGB2GRAY);
    }
    else {
        // Capture DUO frame
        PDUOFrame pFrameData = GetDUOFrame();
        if(pFrameData == NULL) return false;
        IplImage *left = cvCreateImageHeader(cvSize(width, height), IPL_DEPTH_8U, 1);
        IplImage *right = cvCreateImageHeader(cvSize(width, height), IPL_DEPTH_8U, 1);
        // Set the image data
        left->imageData = (char*)pFrameData->leftData;
        right->imageData = (char*)pFrameData->rightData;
        newFrameL = left;
        newFrameR = right;
    }
    
    imgl = newFrameL;
    imgr = newFrameR;
    
    return true;
}

// ////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
// ||||||||||||||||||||||||||||||||||updateRectifiedPair|||||||||||||||||||||||||||||||||||||||||
// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//////////////////////////////////////////////////

bool StereoPair::updateRectifiedPair()
{
    if (not updateUnrectifiedPair()) return false;

    if (calibrationFile.empty()) {
        cout << "Images not rectified because camera calibration file was not found." << endl;
        return true;
    }
    
	// Rectify the frames
	imgl = rectifyImage(imgl, recti, true);
	imgr = rectifyImage(imgr, recti, false);

	return true;
}

// ////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
// ||||||||||||||||||||||||||||||||||||||||resizeImages||||||||||||||||||||||||||||||||||||||||||
// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//////////////////////////////////////////////////

void StereoPair::resizeImages(float scaleFactor){
    /*
     * RESIZE INTERPOLATION METHODS
     *
     * INTER_NEAREST - a nearest-neighbor interpolation
     * INTER_LINEAR - a bilinear interpolation (used by default)
     * INTER_AREA - resampling using pixel area relation. It may be a preferred method for image decimation, as it gives moire’-free results. But when the image is zoomed, it is similar to the INTER_NEAREST method.
     * INTER_CUBIC - a bicubic interpolation over 4x4 pixel neighborhood
     * INTER_LANCZOS4 - a Lanczos interpolation over 8x8 pixel neighborhood
     *
     */
    //RESIZE IMAGE FOR TESTING DIFFERENT RESOLUTIONS
     resize(imgl, imgl, Size(), scaleFactor, scaleFactor, INTER_CUBIC);
     resize(imgr, imgr, Size(), scaleFactor, scaleFactor, INTER_CUBIC);
}

// ////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
// |||||||||||||||||||||||||||||||||||updateDisparityImg||||||||||||||||||||||||||||||||||||||||||
// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//////////////////////////////////////////////////

void StereoPair::updateDisparityImg(float scaleFactor, bool useRectifiedImages){
	Mat imL, imR, rimgl, rimgr;
    if (useRectifiedImages) {
        imL = imgl;
        imR = imgr;
    }
    /*
     * RESIZE INTERPOLATION METHODS
     *
     * INTER_NEAREST - a nearest-neighbor interpolation
     * INTER_LINEAR - a bilinear interpolation (used by default)
     * INTER_AREA - resampling using pixel area relation. It may be a preferred method for image decimation, as it gives moire’-free results. But when the image is zoomed, it is similar to the INTER_NEAREST method.
     * INTER_CUBIC - a bicubic interpolation over 4x4 pixel neighborhood
     * INTER_LANCZOS4 - a Lanczos interpolation over 8x8 pixel neighborhood
     *
     */
	resize(imgl, rimgl, Size(), scaleFactor, scaleFactor, INTER_AREA);
	resize(imgr, rimgr, Size(), scaleFactor, scaleFactor, INTER_AREA);
	
    sgbm(rimgl, rimgr, dsp);
}

// ////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
// |||||||||||||||||||||||||||||||||||||||updateImg3D||||||||||||||||||||||||||||||||||||||||||||
// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//////////////////////////////////////////////////

void StereoPair::updateImg3D(){
	reprojectImageTo3D(dsp, img3D, dispToDepthMat, true, CV_32F);
}

// ////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
// ||||||||||||||||||||||||||||||||||||getDisparityImg|||||||||||||||||||||||||||||||||||||||||||
// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//////////////////////////////////////////////////

Mat StereoPair::getDisparityImg(){
	return dsp;
}

// ////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
// ||||||||||||||||||||||||||||||||getDisparityImgNormalised|||||||||||||||||||||||||||||||||||||
// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//////////////////////////////////////////////////

Mat StereoPair::getDisparityImgNormalised(){
	Mat dspn;
	normalize(dsp, dspn, 0, 255, CV_MINMAX, CV_8U);
	return dspn;
}

// ////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
// ||||||||||||||||||||||||||||||||||||||||getImg3D||||||||||||||||||||||||||||||||||||||||||||||
// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//////////////////////////////////////////////////

Mat StereoPair::getImg3D(){
	return img3D;
}

// ////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
// ||||||||||||||||||||||||||||||saveUncalibratedStereoImages||||||||||||||||||||||||||||||||||||
// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//////////////////////////////////////////////////

void StereoPair::saveUncalibratedStereoImages(string outputFolder)
{
	// Create visualization windows
	namedWindow("Uncalibrated stereo images", CV_WINDOW_NORMAL);

	int frameId = 0;
    Mat IL, IR;
    
	while(1)
    {
        this->updateUnrectifiedPair();
        cvtColor(imgl, IL, COLOR_GRAY2BGR);
        cvtColor(imgr, IR, COLOR_GRAY2BGR);
    	Mat LR = glueTwoImagesHorizontal(IL, IR);
        
        //draw lines
        for(int h=0; h<LR.rows; h+=25)
        {
            Point pt1(0, h);
            Point pt2(LR.cols, h);
            line(LR, pt1, pt2, CV_RGB(255, 0, 0), 1);
        }

    	imshow("Uncalibrated stereo images", LR);

		// Read key press
		int keyPressed = waitKey(10);

		// Save the images if 's' or 'S' key has been pressed
		if( keyPressed==83 || keyPressed==115)
		{
			cout << "Saving image pairs..." << endl;

			if(!outputFolder.empty()){
				// Create the file names for saving the images
				char fileName[256];
				char fileName2[256];
				sprintf(fileName, "%sCam0_%d.png", outputFolder.c_str(), frameId);
				sprintf(fileName2, "%sCam1_%d.png", outputFolder.c_str(), frameId);

				// Write the images
				try {
					imwrite(fileName, IL);
					imwrite(fileName2, IR);
				}
				catch (runtime_error& ex) {
					fprintf(stderr, "Exception converting image to PNG format: %s \n", ex.what());
				}
			}

			// Increment the frame id
			frameId++;
		}

		// Exit if 'esc' key is pressed
        if( keyPressed==27)
		{
			cout << "ESC key is pressed by user" << endl;
			break;
		}

    }

	// Close the windows
    destroyWindow("Left camera");
    destroyWindow("Right camera");
    printf("Saved %i calibration images \n", frameId);
}

// ////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
// |||||||||||||||||||||||||||||||saveCalibratedStereoImages|||||||||||||||||||||||||||||||||||||
// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//////////////////////////////////////////////////

void StereoPair::saveCalibratedStereoImages(string outputFolder){
	namedWindow("Calibrated stereo images", CV_WINDOW_NORMAL);

	int frameId = 0;
    while(1)
    {
    	this->updateRectifiedPair();
    	Mat LR = glueTwoImagesHorizontal(imgl, imgr);
    	imshow("Calibrated stereo images", LR);
		// Wait for key press
		int keyPressed = waitKey(20);

		// Save the images if required
		if( keyPressed== 83 || keyPressed==115)
		{
			char fileName[256];
			char fileName2[256];
			sprintf(fileName, "%sCam0_%d.png", outputFolder.c_str(), frameId);
			sprintf(fileName2, "%sCam1_%d.png", outputFolder.c_str(), frameId);

			try {
				imwrite(fileName, imgl);
				imwrite(fileName2, imgr);
			}
			catch (runtime_error& ex) {
				fprintf(stderr, "Exception converting image to PNG format: %s \n", ex.what());
			}
			frameId++;
		}

		// Exit when esc key is pressed
        if( keyPressed== 27) break;

    }

    //Close windows and show how many image pairs where saved
    destroyWindow("Left camera");
    destroyWindow("Right camera");
    printf("\nESC key pressed. Saved %i calibration images \n", frameId);
}

// ////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
// ||||||||||||||||||||||||||||||||||displayDisparityMap|||||||||||||||||||||||||||||||||||||||||
// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//////////////////////////////////////////////////

void StereoPair::displayDisparityMap(bool showImages, string outputFolder, bool useRectifiedImages){
	namedWindow("Disparity", CV_WINDOW_NORMAL);
	float scaleFactor = 0.5;
	int whiteThreshold = 200;
	int frameID = 0;
	while(1){
		if (useRectifiedImages) this->updateRectifiedPair();
        else                    this->updateUnrectifiedPair();
		this->updateDisparityImg(scaleFactor, useRectifiedImages);
		Mat d1, d2, dispNorm = getDisparityImgNormalised();

		if(showImages){
			d1 = glueTwoImagesHorizontal(imgl, imgr);
			d2 = glueTwoImagesHorizontal(d1, dispNorm);
		}
		else d2 = dispNorm;
        for( int i = 0; i < d2.rows; ++i){
            for( int j = 0; j < d2.cols; ++j ){
            	Scalar intensity = d2.at<uchar>(Point(j, i));
            	if(intensity.val[0] > whiteThreshold){
            		d2.at<uchar>(Point(j, i)) = 0;
            	}
            }
    	}
		//cvtColor(d2, d2, CV_GRAY2RGB );
		//cvtColor(d2, d2, CV_BGR2HSV );

		imshow("Disparity", d2);

		// Wait for key press
		int keyPressed = waitKey(20);

		// Save the images if required
		if( (keyPressed== 83 || keyPressed==115) && !outputFolder.empty())
		{
			char fileName[256];
			sprintf(fileName, "%sdepthMap_%d.png", outputFolder.c_str(), frameID);

			try {
				imwrite(fileName, d2);
			}
			catch (runtime_error& ex) {
				fprintf(stderr, "Exception converting image to PNG format: %s \n", ex.what());
			}
			frameID++;
		}

		// Exit when esc key is pressed
        if( keyPressed== 27) break;
	}
	destroyWindow("Disparity");
}

// ////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
// |||||||||||||||||||||||||||||||||||||||calibrate||||||||||||||||||||||||||||||||||||||||||||||
// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//////////////////////////////////////////////////

void StereoPair::calibrate(bool showResult, String outputFile, String outputFolder){

	///////////INITIAL PARAMETERS//////////////
	Size boardSize = Size(9, 6);	//Inner board corners
	float squareSize = 1.f;			//The actual square size, in any unit
	int nimages = 5;				//Number of images to take for calibration
    const int maxScale = 2;

    vector<vector<Point2f> > imagePoints[2];
    vector<vector<Point3f> > objectPoints;
    Size imageSize;

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    objectPoints.resize(nimages);
    vector<Mat> goodImages;			//Vector to store image pairs with calibration pattern correctly detected

	// Create visualization windows
    Mat frameLR;
    Mat cornerLR;
	namedWindow("Live calibration view", CV_WINDOW_NORMAL);
	namedWindow("Corners", CV_WINDOW_NORMAL);
	Size WindowResize;

	int i, j, k, frameID = 0;
	/*
	 * i: new image pair
	 * j: number of good image pairs
	 * k: image side. 0 = LEFT | 1 = RIGHT
	 */

    for( i = j = 0; j < nimages; i++ )
    {
    	vector<Mat> newStereoFrame;
    	Mat cimg1, cimg2;	//Corner images
    	///////////LIVE CALIBRATION WINDOW//////////////
    	while(1){

            this->updateUnrectifiedPair();
    		Mat niml, nimr;

        	frameLR = glueTwoImagesHorizontal(imgl, imgr);
        	float LiveViewScale = 0.7;
        	WindowResize = Size(frameLR.cols*LiveViewScale, frameLR.rows*LiveViewScale);
        	resize(frameLR, frameLR, WindowResize);
    		// Show images
        	imshow("Live calibration view", frameLR);
    		// Wait for key press
    		int keyPressed = waitKey(20);

    		// Take images if 'n' or 'N' keys have been pressed
    		if( keyPressed==78 || keyPressed==110)
            {
                if (imgl.channels() > 1) cvtColor(imgl, imgl,CV_RGB2GRAY);
                if (imgr.channels() > 1) cvtColor(imgr, imgr,CV_RGB2GRAY);

                newStereoFrame.push_back(imgl);
                newStereoFrame.push_back(imgr);
                cout << "Took a new image pair, " << nimages-(i+1) << " to end calibration" << endl;
                break;
    		}
    		// Save the images if required
    		if((keyPressed== 83 || keyPressed==115) && j>0)
    		{
    			cout << "Saving image pairs..." << endl;

    			if(!outputFolder.empty()){
    				// Create the file names for saving the images
    				char fileName[256];
    				char fileName2[256];
    				sprintf(fileName, "%sCam0_%d.png", outputFolder.c_str(), frameID);
    				sprintf(fileName2, "%sCam1_%d.png", outputFolder.c_str(), frameID);

    				// Write the images
    				try {
    					imwrite(fileName, frameLR);
    					imwrite(fileName2, cornerLR);
    				}
    				catch (runtime_error& ex) {
    					fprintf(stderr, "Exception converting image to PNG format: %s \n", ex.what());
    				}
    			}
    			frameID++;
    		}
    		else if(keyPressed==27){
    			destroyWindow("Live calibration view");
    			destroyWindow("Corners");
    			return;
    		}
    	}

    	//////////DETECT CHESSBOARD CORNERS AND DISPLAY THEM/////////////
        for( k = 0; k < 2; k++ )
        {
            //Mat img = (k==0?imgL.at(i):imgR.at(i));
        	Mat img = newStereoFrame.at(k);

            if(img.empty())
                break;


            if( imageSize == Size() )
                imageSize = img.size();

            bool found = false;
            vector<Point2f>& corners = imagePoints[k][j];
            for( int scale = 1; scale <= maxScale; scale++ )
            {
                Mat timg;
                if( scale == 1 )
                    timg = img;
                else
                    resize(img, timg, Size(), scale, scale);
                found = findChessboardCorners(timg, boardSize, corners,
                    CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
                if( found )
                {
                    if( scale > 1 )
                    {
                        Mat cornersMat(corners);
                        cornersMat *= 1./scale;
                    }
                    break;
                }
                else{
                	cout << "Calibration pattern must be inside the frame!" << endl;
                	break;
                }
            }

            //////////////Update corner view images///////////////////////
            string imageSide = (k==0? " left" : " right");
            cout << "Image: " << i << imageSide << endl;
            Mat cimg;
            cvtColor(img, cimg, COLOR_GRAY2BGR);
            drawChessboardCorners(cimg, boardSize, corners, found);
            double sf = 640./MAX(img.rows, img.cols);
            Mat cornerImg;
            resize(cimg, cornerImg, Size(), sf, sf);
            if(k==0)     cimg1 = cornerImg;
            else if(k==1)cimg2 = cornerImg;
            //////////////////////////////////////////////////////

            ///////////////IMPROVE CORNER ACCURACY////////////////
            cornerSubPix(img, corners, Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01));
        }

        /////////////Show corner images//////////////////
        cornerLR = glueTwoImagesHorizontal(cimg1, cimg2);
        resize(cornerLR, cornerLR, WindowResize);
        imshow("Corners", cornerLR);
        /////////////////////////////////////////////////

        //////ADD STEREO PAIR TO "goodImages" IF THE CHESSBOARD CORNERS WHERE FOUND ON BOTH IMAGES//////
        if( k == 2 )
        {
            goodImages.push_back(newStereoFrame.at(0));
            goodImages.push_back(newStereoFrame.at(1));
            j++;
        }
    }

    ////////FILL "objectPoints" WITH THE COORDINATES OF THE BOARD CORNERS////////
    for( i = 0; i < nimages; i++ )
    {
        for( j = 0; j < boardSize.height; j++ )
            for( k = 0; k < boardSize.width; k++ )
                objectPoints[i].push_back(Point3f(j*squareSize, k*squareSize, 0));
    }

    /////////////RUN STEREO CALIBRATION//////////////
    cout << "Running stereo calibration ...\n";

    Mat cameraMatrix[2], distCoeffs[2];
    cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
    cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
    Mat R, T, E, F;

    double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                    cameraMatrix[0], distCoeffs[0],
                    cameraMatrix[1], distCoeffs[1],
                    imageSize, R, T, E, F,
                    TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                    CV_CALIB_FIX_ASPECT_RATIO +
                    CV_CALIB_ZERO_TANGENT_DIST +
                    CV_CALIB_SAME_FOCAL_LENGTH +
                    CV_CALIB_RATIONAL_MODEL +
                    CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);
    cout << "done with RMS error=" << rms << endl;
/*
 * /////////CALIBRATION QUALITY CHECK//////////////
 * because the output fundamental matrix implicitly
 * includes all the output information,
 * we can check the quality of calibration using the
 * epipolar geometry constraint: m2^t*F*m1=0
 */
    double err = 0;
    int npoints = 0;
    vector<Vec3f> lines[2];
    for( i = 0; i < nimages; i++ )
    {
        int npt = (int)imagePoints[0][i].size();
        Mat imgpt[2];
        for( k = 0; k < 2; k++ )
        {
            imgpt[k] = Mat(imagePoints[k][i]);
            undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
            computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
        }
        for( j = 0; j < npt; j++ )
        {
            double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
                                imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
                           fabs(imagePoints[1][i][j].x*lines[0][j][0] +
                                imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
    cout << "average reprojection err = " <<  err/npoints << endl;

    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];

    stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

    //////////Save calibration parameters//////////////////
    calibrationFile = outputFile;
    FileStorage fs(calibrationFile.c_str(), CV_STORAGE_WRITE);
    if( fs.isOpened() )
    {
    	cout << "Saving parameters" << endl;
    	fs << "Size" << imageSize << "K1" << cameraMatrix[0] << "distCoeffs1" << distCoeffs[0] <<
    			"K2" << cameraMatrix[1] << "distCoeffs2" << distCoeffs[1] <<
    			"R" << R << "T" << T;
    	fs.release();
    	cout << "Parameters saved" << endl;
    }
    else cout << "CALIBRATION ERROR: Cannot save calibration results to file" << endl;

   //////////Once parameters are saved, reinitialize rectification from them////////////////
    this->setupRectification(outputFolder);

    if(showResult)this->rectificationViewer();
}
