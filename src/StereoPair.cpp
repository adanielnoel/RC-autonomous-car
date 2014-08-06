/*
 * StereoPair.cpp
 *
 *  Created on: Jul 26, 2014
 *      Author: nicolau
 */


#include "StereoPair.h"

StereoPair::StereoPair() {
	// TODO Auto-generated constructor stub

}

double StereoPair::depthCoef = 1;	//test values. NON REAL VALUES
double StereoPair::scaleCoef = 2;	//test values. NON REAL VALUES

StereoPair::StereoPair(int lCamId, int rCamId, int camFPS){
	camL.open(lCamId);
	camR.open(rCamId);
    camL.set(CV_CAP_PROP_FPS, camFPS);
    camR.set(CV_CAP_PROP_FPS, camFPS);
    if(!camL.isOpened()) cout << "Cannot open left camera" << endl;
	if(!camR.isOpened()) cout << "Cannot open right camera" << endl;
	if(camL.isOpened() && camR.isOpened()){ //print frame size
		double dWidth = camL.get(CV_CAP_PROP_FRAME_WIDTH);
		double dHeight = camL.get(CV_CAP_PROP_FRAME_HEIGHT);
		cout << "Left camera frame size : " << dWidth << " x " << dHeight << endl;
		dWidth = camR.get(CV_CAP_PROP_FRAME_WIDTH);
		dHeight = camR.get(CV_CAP_PROP_FRAME_HEIGHT);
		cout << "Right camera frame size : " << dWidth << " x " << dHeight << endl;
	}
	this->setupDisparity();
}

StereoPair::~StereoPair() {
	// TODO Auto-generated destructor stub
}

void StereoPair::setupDisparity(){
	sgbm = StereoSGBM();
	sgbm.SADWindowSize = 13;
	sgbm.numberOfDisparities = 96;
	sgbm.preFilterCap = 27;
	sgbm.minDisparity = 0;
	sgbm.uniquenessRatio = 0;
	sgbm.speckleWindowSize = 30;
	sgbm.speckleRange = 60;
	sgbm.disp12MaxDiff = -80;
	sgbm.fullDP = true;
	sgbm.P1 = 8*sgbm.SADWindowSize*sgbm.SADWindowSize;
	sgbm.P2 = 32*sgbm.SADWindowSize*sgbm.SADWindowSize;
}

void StereoPair::setupRectification(string calibrationFile, string calOutput)
{
	Mat R1, R2, P1, P2, Q;
	Mat rmap[2][2];
	Rect validRoi[2];

	// Open calibration file
	FileStorage fs(calibrationFile, FileStorage::READ);

	// Read calibration file
	if(!fs.isOpened())
		cout << "Calibration file was not found" << endl;
	else{
		Mat cameraMatrix0, cameraMatrix1, distCoeffs0, distCoeffs1, imgSize, RInitial, TInitial;
		fs["Size"] >> imgSize;
		fs["K1"] >> cameraMatrix0;
		fs["distCoeffs1"] >> distCoeffs0;
		fs["K2"] >> cameraMatrix1;
		fs["distCoeffs2"] >> distCoeffs1;
		fs["R"] >> RInitial;
		fs["T"] >> TInitial;
		Size imageSize;
		imageSize.width = imgSize.at<double>(0, 0);
		imageSize.height = imgSize.at<double>(1, 0);

		// Compute rectification mappings based on the calibration
		stereoRectify(cameraMatrix0, distCoeffs0, cameraMatrix1, distCoeffs1, imageSize, RInitial, TInitial, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, 0, imageSize, &validRoi[0], &validRoi[1]);
		initUndistortRectifyMap(cameraMatrix0, distCoeffs0, R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
		initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

		// Save the rectification mappings
		recti.K = P1;
		recti.B = P2.at<double>(0, 3) / P2.at<double>(0, 0);
		for(int i=0; i<2; i++)
		for(int j=0; j<2; j++)
			recti.rmap[i][j] = rmap[i][j];

		//write new calibration
		if(!calOutput.empty()){
			FileStorage fout(calOutput, FileStorage::WRITE);
			fout << "K" << recti.K << "T" << -recti.B;
			fout.release();
		}
	}
}



Mat StereoPair::rectifyImage(const Mat& I, const Rectification& recti, bool left)
{
	Mat O;

	if(left)
		remap(I, O, recti.rmap[0][0], recti.rmap[0][1], CV_INTER_LINEAR, BORDER_TRANSPARENT);
	else
		remap(I, O, recti.rmap[1][0], recti.rmap[1][1], CV_INTER_LINEAR, BORDER_TRANSPARENT);

	return O;
}

void StereoPair::RectificationViewer()
{
	Mat IL = imgl;
	Mat IR = imgr;
	// set both images horizontally adjacent
	Mat LR(IL.rows, IL.cols+IR.cols, IL.type());
	Mat left_roi(LR, Rect(0, 0, IL.cols, IL.rows)); // Copy constructor
	IL.copyTo(left_roi);
	Mat right_roi(LR, Rect(IL.cols, 0, IR.cols, IR.rows)); // Copy constructor
	IR.copyTo(right_roi);

	//Mat LR2;
	//cvtColor(LR, LR2, CV_GRAY2RGB);

	//draw lines
	for(int h=0; h<LR.rows; h+=20)
	{
		Point pt1(0, h);
		Point pt2(LR.cols, h);
		line(LR, pt1, pt2, CV_RGB(255, 0, 0), 2);
	}

	// Show the image
	imshow("Rectification", LR);
}

bool StereoPair::updateRectifiedPair()
{
	Mat newFrameL, newFrameR;

	//get new (unrectified) frames
	if (!camL.read(newFrameL)) return false;
	if (!camR.read(newFrameR)) return false;

	cvtColor(newFrameL,newFrameL,CV_RGB2GRAY);
	cvtColor(newFrameR,newFrameR,CV_RGB2GRAY);

	// Rectify the frames
	imgl = rectifyImage(newFrameL, recti, true);
	imgr = rectifyImage(newFrameR, recti, false);


	/*Size size(320, 240);
	resize(imgl, imgl, size);
	resize(imgr, imgr, size);
*/
	return true;
}

void StereoPair::updateDepthMap()
{
	sgbm(imgl, imgr, dmp);
}

Mat StereoPair::getMainImg(){
	return imgl;
}

Mat StereoPair::getDepthMap(){
	return dmp;
}

Mat StereoPair::getDepthMapNormalised(){
	Mat dmpn;
	normalize(dmp, dmpn, 0, 255, CV_MINMAX, CV_8U);
	return dmpn;
}

bool StereoPair::pixelToPoint(Mat& _dmp, Point2f pixel, Point3d& point){
	double depth = _dmp.at<uchar>(pixel);
	if(depth == 0)return false;
	double z = depth*depthCoef;
	double x = pixel.x * depth * scaleCoef;
	double y = pixel.y * depth * scaleCoef;
	point.x = x;
	point.y = y;
	point.z = z;
	return true;
}

void StereoPair::calibrateCoefs(){
	namedWindow("Calibrate coefficients", CV_WINDOW_AUTOSIZE);
	while(1){
		this->updateRectifiedPair();
		this->updateDepthMap();
		int closest;
		/*for(int x=0; x<dmp.cols; x++){
			for(int y=0; y<dmp.rows; y++){
				Scalar cd = dmp.at<uchar>(Point(x, y));
				int currentDepth = cd.val[0];
				closest = currentDepth>closest?currentDepth:closest;
			}
		}

		cvtColor(dmp, dmp, CV_GRAY2RGB);
		Vec3d color = Vec3d(60, 100, 200);
		for(int x=0; x<dmp.cols; x++){
			for(int y=0; y<dmp.rows; y++){
				Scalar cd = dmp.at<uchar>(Point(x, y));
				int currentDepth = cd.val[0];
				if(currentDepth == closest)
					dmp.at<Vec3b>(Point(x, y)) = color;
			}
		}*/
		imshow("Calibrate coefficients", dmp);
				// Wait for key press
		int keyPressed = waitKey(30);
		if( keyPressed== 27)
		{
			cout << "ESC key is pressed by user" << endl;
			destroyWindow("Calibrate coefficients");
			break;
		}
	}
}


void StereoPair::saveCalibrationFrames(string outputFolder)
{
	// Create visualization windows
	namedWindow("Left camera", CV_WINDOW_AUTOSIZE);
	namedWindow("Right camera", CV_WINDOW_AUTOSIZE);

	int frameId = 0;
	Mat newFrameL, newFrameR;
    while(1)
    {
		// Grab stereo images
    	if (!camL.grab() || !camR.grab()){
    		cout << "Error getting frames from camera" << endl << "Try reducing FPS or frame size" << endl;
    		break;
    	}

		// Retrieve images
    	camL.retrieve(newFrameL);
    	camR.retrieve(newFrameR);

		// Show images
    	imshow("Left camera", newFrameL);
    	imshow("Right camera", newFrameR);

		// Wait for key press
		int keyPressed = waitKey(30);		// TODO: Be carefull here. You are waiting 30ms

		// Save the images if required
		if( keyPressed==83 || keyPressed==115)
		{
			// Create the file names for saving the images
			char fileName[256];
			char fileName2[256];
			sprintf(fileName, "%sCam0_%d.png", outputFolder.c_str(), frameId);
			sprintf(fileName2, "%sCam1_%d.png", outputFolder.c_str(), frameId);

			// Write the images
			try {
				imwrite(fileName, newFrameL);
				imwrite(fileName2, newFrameR);
			}
			catch (runtime_error& ex) {
				fprintf(stderr, "Exception converting image to PNG format: %s \n", ex.what());
			}

			// Update the frame id
			frameId++;
		}

		// Exit when esc key is pressed
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

void StereoPair::saveCalibratedImages(string outputFolder){
	namedWindow("Left camera", CV_WINDOW_AUTOSIZE);
	namedWindow("Right camera", CV_WINDOW_AUTOSIZE);

	int frameId = 0;
    while(1)
    {
		// Get left image

    	this->updateRectifiedPair();
    	imshow("Left camera", imgl);
    	imshow("Right camera", imgr);
		// Wait for key press
		int keyPressed = waitKey(30);

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
        if( keyPressed== 27)
		{
			cout << "ESC key is pressed by user" << endl;
			break;
		}

    }
    destroyWindow("Left camera");
    destroyWindow("Right camera");
    printf("Saved %i calibration images \n", frameId);
}
