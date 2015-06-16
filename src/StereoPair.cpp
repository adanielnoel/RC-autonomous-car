/*
 * StereoPair.cpp
 *
 *  Created on: Jul 26, 2014
 *      Author: Alejandro Daniel Noel
 *      Page: http://futuretechmaker.com
 */


#include "StereoPair.h"
#include "DUO3D_camera.h"
#include <DUOLib.h>

float mapValue(float val, float min1, float max1, float min2, float max2){
    bool invertRet = false;
    if (min2 > max2) {
        float tmpMin = min2;
        max2 = min2;
        min2 = tmpMin;
        invertRet = true;
    }
    if (invertRet) return (max2 - (((val - min1) * max2) / max1) - min2);
    else return ((((val - min1) * max2) / max1) - min2);
}

float constrain(float val, float min, float max){
    if (min > max) {
        float tmp = max;
        max = min;
        min = tmp;
    }
    if (val < min) val = min;
    else if (val > max) val = max;
    return val;
}



const bool StereoPair::USE_OPENCV_REPROJECTION_METHOD = 0;
const bool StereoPair::USE_CUSTOM_REPROJECTION_METHOD = 1;

StereoPair::StereoPair() {
	// TODO Auto-generated constructor stub

}

StereoPair::StereoPair(int lCamId, int rCamId, int _width, int _height, int camFPS, bool & success){
    width = _width;
    height = _height;
    fps = camFPS;
    cameraIsUpsideDown = false;
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
        else success = true;
        // Create OpenCV windows
        cvNamedWindow("Left");
        cvNamedWindow("Right");
        
        // Set exposure and LED brightness
        SetExposure(9);
        SetLed(0);
    }
    
	//Setup Semi Global Block Matching object
	this->setupDisparity();
}

// ////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
// |||||||||||||||||||||||||||||||||||||setupDisparity|||||||||||||||||||||||||||||||||||||||||||
// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//////////////////////////////////////////////////

void StereoPair::setupDisparity(){
    if (defaultCamera) {
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
    
    else{
        sgbm.SADWindowSize = 5;
        sgbm.numberOfDisparities = 192;
        sgbm.preFilterCap = 9;
        sgbm.minDisparity = 5;
        sgbm.uniquenessRatio = 18;
        sgbm.speckleWindowSize = 83;
        sgbm.speckleRange = 95;
        sgbm.disp12MaxDiff = 25;
        sgbm.fullDP = true;
        sgbm.P1 = 240;
        sgbm.P2 = 2339;
    }
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
	Mat R1, R2, P1, P2;
	Mat rmap[2][2];
	Rect validRoi[2];

    rectificationCorrect = false; // initialize flag
    
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
        
        rectificationCorrect = true;
        
        /*
        calibrationFile = "/Users/alejandrodanielnoel1/Desktop/q.xml";
        FileStorage fs(calibrationFile.c_str(), CV_STORAGE_WRITE);
        if( fs.isOpened() )
        {
            cout << "Saving Q" << endl;
            fs << "Q" << Q;
            fs.release();
            cout << "Parameters saved" << endl;
        }
         */
        
        
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
			line(LR, pt1, pt2, CV_RGB(255, 123, 47), 1);
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
    
    if (cameraIsUpsideDown) {
        flip(newFrameL, newFrameL, 0);  // Flip vertically
        flip(newFrameR, newFrameR, 0);  // Flip vertically
        flip(newFrameL, newFrameL, 1);  // Flip horizontally
        flip(newFrameR, newFrameR, 1);  // Flip horizontally
    }
    
    // For some reason, DUO3D left frame is right and vice-versa!!!
    imgr = newFrameL;
    imgl = newFrameR;
    
    return true;
}

// ////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
// ||||||||||||||||||||||||||||||||||updateRectifiedPair|||||||||||||||||||||||||||||||||||||||||
// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//////////////////////////////////////////////////

bool StereoPair::updateRectifiedPair()
{
    //Get the images
    if (!updateUnrectifiedPair()){ // If the images where not retrieved, print error.
        cout << "Error: Could not get any image from the camera." << endl;
        return false;
    }
    if (!rectificationCorrect) {
        cout << "Images not rectified because camera calibration file was not found." << endl;
        return false;
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

void StereoPair::updateDisparityImg(float scaleFactor){
	Mat imL, imR, rimgl, rimgr;
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
    resize(dsp, dsp, Size(), 1/scaleFactor, 1/scaleFactor, INTER_AREA);
}


 // This function was used for testing and requires Point Cloud library.
 // It has been commented out to make distribution easier.
 
void StereoPair::run3DVisualizer(){
    //Create point cloud and fill it
    std::cout << "Creating Point Cloud..." <<std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    float minZ = 1000000, maxZ = 0;
    for(int i = 0; i < img3D.cols; i++){
        for(int j = 0; j < img3D.rows; j++){
            pcl::PointXYZRGB point;
            point.x = float(img3D.at<Vec3f>(j, i).val[0]*100.0);
            point.y = float(img3D.at<Vec3f>(j, i).val[1]*100.0);
            point.z = float(img3D.at<Vec3f>(j, i).val[2]*100.0);
            if(point.z < minZ) minZ = point.z;
            if (point.z > maxZ) maxZ = point.z;
            
            point_cloud_ptr->points.push_back(point);
            //cout << "X: " << point.x << "   Y: " << point.y << "   Z: " << point.z << endl;
        }
    }
    cout << "minZ: " << minZ << "     maxZ: " << maxZ << endl;
    for(unsigned int i = 0; i < point_cloud_ptr->size(); i++){
        float pz = point_cloud_ptr->at(i).z;
        uint8_t r(255 - constrain(mapValue(pz, minZ, maxZ, 0, 255), 0, 255));
        uint8_t g(constrain(mapValue(pz, minZ, maxZ, 0, 255), 0, 255));
        uint8_t b(15);
        uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                        static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
        point_cloud_ptr->at(i).rgb = *reinterpret_cast<float*>(&rgb);
    }
    point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (1, 1, 1);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr);
    viewer->addPointCloud<pcl::PointXYZRGB>(point_cloud_ptr, rgb, "reconstruction");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "reconstruction");
    viewer->addCoordinateSystem ( 1.0 );
    viewer->initCameraParameters ();
    
    while ( !viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds (500000));
        int keyPressed = waitKey(10);
        if (keyPressed==27){
            viewer->close();
        }
    }
}

// ////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
// |||||||||||||||||||||||||||||||||||||||updateImg3D||||||||||||||||||||||||||||||||||||||||||||
// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//////////////////////////////////////////////////

void StereoPair::updateImg3D(bool useCustomMethod){
    img3D = Mat(dsp.size(), CV_32FC3);
    if (useCustomMethod) {
        //Get the interesting parameters from Q
        double Q03, Q13, Q23, Q32, Q33;
        Q03 = Q.at<double>(0,3);    //cx
        Q13 = Q.at<double>(1,3);    //cy
        Q23 = Q.at<double>(2,3);    //f 
        Q32 = Q.at<double>(3,2);    //
        Q33 = Q.at<double>(3,3);
        
        double px, py, pz;
        Mat dspn = this->getDisparityImg();
        double minX = 10000000, maxX = 0;
        double minY = 10000000, maxY = 0;
        double minZ = 10000000, maxZ = 0;
        for (int i = 0; i < dspn.rows; i++)
        {
            uchar* disp_ptr = dspn.ptr<uchar>(i);

            for (int j = 0; j < dspn.cols; j++)
            {
                //Get 3D coordinates
                double d = static_cast<double>(disp_ptr[j]);
                if ( d == 0 ) continue; //Discard bad pixels
                double pw = 1.0 * d * Q32 + Q33;
                px = static_cast<double>(j) + Q03;
                py = static_cast<double>(i) + Q13;
                pz = Q23;
                
                px = px/pw;
                py = py/pw;
                pz = pz/pw;
                if (pz == 0) continue;
                if (pz > 0.3) continue;
                //if (px > 1.0 || px < 1.0) continue;
                //if (py > 1.0 || py < 1.0) continue;
                if(px < minX) minX = px;
                else if (px > maxX) maxX = px;
                if(py < minY) minY = py;
                else if (py > maxY) maxY = py;
                if(pz < minZ) minZ = pz;
                else if (pz > maxZ) maxZ = pz;
                img3D.at<Vec3f>(i, j).val[0] = px;
                img3D.at<Vec3f>(i, j).val[1] = -1*py;
                img3D.at<Vec3f>(i, j).val[2] = pz;
            }
        }
        //cout << "minX: " << minX << "   maxX: " << maxX << endl;
        //cout << "minY: " << minY << "   maxY: " << maxY << endl;
        //cout << "minZ: " << minZ << "   maxZ: " << maxZ << endl;
    }
    else {
        Mat depthImage = Mat(dsp.rows, dsp.cols, dsp.type());
        flip(dsp, depthImage, 1);  // reprojectImageTo3D() rotates the image, so we prevent this before
        flip(depthImage, depthImage, 0);
        reprojectImageTo3D(depthImage, img3D, Q);//, true, CV_32FC3);
    }
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
            line(LR, pt1, pt2, CV_RGB(255, 123, 47), 1);
        }

    	imshow("Uncalibrated stereo images", LR);

		// Read key press
		int keyPressed = waitKey(10);

		// Save the images if 's' or 'S' key has been pressed
		if( keyPressed==83 || keyPressed==115)
		{
			cout << "Saving uncalibrated image pairs..." << endl;

			if(!outputFolder.empty()){
				// Create the file names for saving the images
				char fileName[256];
				char fileName2[256];
				sprintf(fileName, "%sUncalibCam0_%d.png", outputFolder.c_str(), frameId);
				sprintf(fileName2, "%sUncalibCam1_%d.png", outputFolder.c_str(), frameId);

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
        
        //draw lines
        for(int h=0; h<LR.rows; h+=25)
        {
            Point pt1(0, h);
            Point pt2(LR.cols, h);
            line(LR, pt1, pt2, CV_RGB(255, 123, 47), 1);
        }
    	
        imshow("Calibrated stereo images", LR);
		// Wait for key press
		int keyPressed = waitKey(20);

		// Save the images if required
		if( keyPressed== 83 || keyPressed==115)
		{
            cout << "Saving calibrated image pairs..." << endl;
			char fileName[256];
			char fileName2[256];
			sprintf(fileName, "%sCalibCam0_%d.png", outputFolder.c_str(), frameId);
			sprintf(fileName2, "%sCalibCam1_%d.png", outputFolder.c_str(), frameId);

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
	namedWindow("Controls", CV_WINDOW_NORMAL);
    if (showImages) {
        namedWindow("Left — Right", CV_WINDOW_NORMAL);
    }
    createTrackbar("SADWindowSize", "Controls", &sgbm.SADWindowSize, 50);
  //  createTrackbar("numberOfDisparities", "Controls", &sgbm.numberOfDisparities, 1000);
    createTrackbar("preFilterCap", "Controls", &sgbm.preFilterCap, 100);
    createTrackbar("minDisparity", "Controls", &sgbm.minDisparity, 100);
    createTrackbar("uniquenessRatio", "Controls", &sgbm.uniquenessRatio, 100);
    createTrackbar("speckleWindowSize", "Controls", &sgbm.speckleWindowSize, 300);
    createTrackbar("speckleRange", "Controls", &sgbm.speckleRange, 100);
    createTrackbar("disp12MaxDiff", "Controls", &sgbm.disp12MaxDiff, 100);
    createTrackbar("P1", "Controls", &sgbm.P1, 3000);
    createTrackbar("P2", "Controls", &sgbm.P2, 10000);

	float scaleFactor = 0.50;
	int whiteThreshold = 255;
	int frameID = 0;
    
    cout << "Press \n's' to save disparity map\n'd' to launch the 3D visualizer\n'x' to print corrent SGBM parameters" << endl;
    
	while(1){
		this->updateRectifiedPair();
		this->updateDisparityImg(scaleFactor);

		if(showImages){
			Mat d1 = glueTwoImagesHorizontal(imgl, imgr);
            imshow("Left — Right", d1);
		}
		Mat dispNorm = getDisparityImgNormalised();
        for( int i = 0; i < dispNorm.rows; ++i){
            for( int j = 0; j < dispNorm.cols; ++j ){
            	Scalar intensity = dispNorm.at<uchar>(Point(j, i));
            	if(intensity.val[0] > whiteThreshold){
            		dispNorm.at<uchar>(Point(j, i)) = 0;
            	}
            }
    	}

        //resize(dispNorm, dispNorm, Size(), 1/scaleFactor, 1/scaleFactor, INTER_CUBIC);
		imshow("Disparity", dispNorm);

		// Wait for key press
		int keyPressed = waitKey(20);
        
        // Run point cloud visualizer y 'd' or 'D' keys are pressed
        if(keyPressed== 68 || keyPressed==100) {
            updateImg3D(USE_OPENCV_REPROJECTION_METHOD);
            run3DVisualizer();
        }
        
        else if(keyPressed==120 || keyPressed==88){
            cout << "sgbm.SADWindowSize = " << sgbm.SADWindowSize << ";" << endl;
            cout << "sgbm.numberOfDisparities = " << sgbm.numberOfDisparities << ";" << endl;
            cout << "sgbm.preFilterCap = " << sgbm.preFilterCap << ";" << endl;
            cout << "sgbm.minDisparity = " << sgbm.minDisparity << ";" << endl;
            cout << "sgbm.uniquenessRatio = " << sgbm.uniquenessRatio << ";" << endl;
            cout << "sgbm.speckleWindowSize = " << sgbm.speckleWindowSize << ";" << endl;
            cout << "sgbm.speckleRange = " << sgbm.speckleRange << ";" << endl;
            cout << "sgbm.disp12MaxDiff = " << sgbm.disp12MaxDiff << ";" << endl;
            cout << "sgbm.fullDP = " << (sgbm.fullDP==0?"false":"true") << ";" << endl;
            cout << "sgbm.P1 = " << sgbm.P1 << ";" << endl;
            cout << "sgbm.P2 = " << sgbm.P2 << ";" << endl;
        }
		// Save the images if required (press 's' or 'S')
		else if( (keyPressed== 83 || keyPressed==115) && !outputFolder.empty()) {
			char fileName[256];
			sprintf(fileName, "%sdepthMap_%d.png", outputFolder.c_str(), frameID);

			try {
				imwrite(fileName, dispNorm);
			}
			catch (runtime_error& ex) {
				fprintf(stderr, "Exception converting image to PNG format: %s \n", ex.what());
			}
			frameID++;
		}

		// Exit when esc key is pressed
        if( keyPressed== 27) break;
	}
    destroyAllWindows();
}

// ////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
// |||||||||||||||||||||||||||||||||||||||calibrate||||||||||||||||||||||||||||||||||||||||||||||
// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//////////////////////////////////////////////////

void StereoPair::calibrate(String outputFile, String outputFolder){

	///////////INITIAL PARAMETERS//////////////
	Size boardSize = Size(9, 6);	//Inner board corners
	float squareSize = 0.022;       //The actual square size, in any unit (meters prefered)
	int nimages = 9;				//Number of images to take for calibration
    int maxScale = 2;

    vector<vector<Point2f> > imagePoints[2];
    vector<vector<Point3f> > objectPoints;
    Size imageSize;

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    objectPoints.resize(nimages);
    vector<Mat> goodImages;			//Vector to store image pairs with calibration pattern correctly detected
    
    int itersSinceLastPattern = 1;

	// Create visualization windows
    float windowResize = 0.8;
    Mat calibDisplay;
	namedWindow("Live calibration view", CV_WINDOW_NORMAL);

	int i, j, k, frameID = 0;
	/*
	 * i: new image pair
	 * j: number of good image pairs
	 * k: image side. 0 = LEFT | 1 = RIGHT
	 */

    for( i = j = 0; j < nimages; i++ )
    {
    	while(1){
            this->updateUnrectifiedPair();
    		Mat niml = imgl, nimr = imgr;
            Mat cimg1, cimg2;	//Corner images
            bool found = false;
            if( imageSize == Size() ) imageSize = niml.size();
            
            //////////DETECT CHESSBOARD CORNERS AND DISPLAY THEM/////////////
            for( k = 0; k < 2; k++ ) {
                Mat img = k==0 ? niml : nimr;
                if(img.empty()) break;
                
                vector<Point2f>& corners = imagePoints[k][j];
                
                for( int scale = 1; scale <= maxScale; scale++ ) {
                    Mat timg;
                    if( scale == 1 )
                        timg = img;
                    else
                        resize(img, timg, Size(), scale, scale);
                    
                    found = findChessboardCorners(timg, boardSize, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
                    
                    if(found) {
                        if(scale > 1) {
                            Mat cornersMat(corners);
                            cornersMat *= 1./scale;
                        }
                        //////////////Update corner view images///////////////////////
                        Mat cimg;
                        cvtColor(img, cimg, COLOR_GRAY2BGR);
                        drawChessboardCorners(cimg, boardSize, corners, found);
                        double sf = 640./MAX(img.rows, img.cols);
                        Mat cornerImg;
                        resize(cimg, cornerImg, Size(), sf, sf);
                        if(k==0)     cimg1 = cornerImg;
                        else if(k==1)cimg2 = cornerImg;
                        
                        ///////////////IMPROVE CORNER ACCURACY////////////////
                        cornerSubPix(img, corners, Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01));
                        break;
                    }
                }
                
                if(!found) {
                    if (itersSinceLastPattern == 0) {
                        cout << "*** Alert! Calibration pattern must be detected in both frames!" << endl;
                    }
                    itersSinceLastPattern++;
                    break;
                }
            }
            
            if(k == 2 && found){
                calibDisplay = glueTwoImagesHorizontal(cimg1, cimg2);   // Update corner images
                resize(calibDisplay, calibDisplay, Size(), windowResize, windowResize);
                if(itersSinceLastPattern > 0){
                    cout << "*** Correctly detecting pattern!" << endl;
                    itersSinceLastPattern = 0;
                }
            } else {
                calibDisplay = glueTwoImagesHorizontal(imgl, imgr);
                resize(calibDisplay, calibDisplay, Size(), windowResize, windowResize);
            }
            imshow("Live calibration view", calibDisplay);
            
    		// Wait for key press
    		int keyPressed = waitKey(20);

    		// Take images if 'n' or 'N' keys have been pressed
    		if( keyPressed==78 || keyPressed==110)
            {
                if(k == 2){
                    if (imgl.channels() > 1) cvtColor(imgl, imgl, CV_RGB2GRAY);
                    if (imgr.channels() > 1) cvtColor(imgr, imgr, CV_RGB2GRAY);
                    goodImages.push_back(imgl);
                    goodImages.push_back(imgr);
                    j++;
                    cout << "Took a new image pair, remaining images: " << nimages - j << endl;
                    break;
                }
                else cout << "Alert! Calibration pattern must be detected in both frames!" << endl;
    		}
    		// Save the images if required (pressing 's' or 'S')
    		if(keyPressed== 83 || keyPressed==115)
    		{
    			cout << "Saving image pairs..." << endl;

    			if(!outputFolder.empty()){
    				// Create the file names for saving the images
    				char fileName[256];
    				sprintf(fileName, "%sCalib_%d.png", outputFolder.c_str(), frameID);

    				// Write the images
    				try {
    					imwrite(fileName, calibDisplay);
    				}
    				catch (runtime_error& ex) {
    					fprintf(stderr, "Exception converting image to PNG format: %s \n", ex.what());
    				}
    			}
    			frameID++;
    		}
    		else if(keyPressed==27){
    			destroyWindow("Live calibration view");
    			return;
    		}
    	}
    }
    
    destroyWindow("Live calibration view");

    ////////FILL "objectPoints" WITH THE COORDINATES OF THE BOARD CORNERS////////
    for( i = 0; i < nimages; i++ )
    {
        for( j = 0; j < boardSize.height; j++ )
            for( k = 0; k < boardSize.width; k++ )
                objectPoints[i].push_back(Point3f(j*squareSize, k*squareSize, 0));
    }

    /////////////RUN STEREO CALIBRATION//////////////
    cout << "******************************" << endl;
    cout << "Running stereo calibration ..." << endl;

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
    			"R" << R << "T" << T << "Q" << Q;
    	fs.release();
    	cout << "Parameters saved" << endl;
    }
    else cout << "CALIBRATION ERROR: Cannot save calibration results to file" << endl;

   //////////Once parameters are saved, reinitialize rectification from them////////////////
    this->setupRectification(outputFile);
    this->rectificationViewer();
}
