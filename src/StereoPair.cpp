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

Mat StereoPair::glueTwoImagesHorizontal(Mat Img1, Mat Img2){
    Mat LR(Img1.rows, Img1.cols+Img2.cols, Img1.type());
    
    //Place each image horizontally adjacent to each other
    Mat left_roi(LR, Rect(0, 0, Img1.cols, Img1.rows));
    Img1.copyTo(left_roi);
    Mat right_roi(LR, Rect(Img1.cols, 0, Img2.cols, Img2.rows)); // Copy constructor
    Img2.copyTo(right_roi);
    
    return LR;
}

Mat StereoPair::glueTwoImagesVertical(Mat Img1, Mat Img2){
    Mat LR(Img1.rows+Img2.rows, Img1.cols, Img1.type());
    
    //Place each image vertically adjacent to each other
    Mat top_roi(LR, Rect(0, 0, Img1.cols, Img1.rows));
    Img1.copyTo(top_roi);
    Mat bottom_roi(LR, Rect(0, Img1.rows, Img2.cols, Img2.rows)); // Copy constructor
    Img2.copyTo(bottom_roi);
    
    return LR;
}


StereoPair::StereoPair(){
# ifdef DUO3D
    assert(OpenDUOCamera(width, height, fps));
    // Set exposure and LED brightness
    SetExposure(30);
    SetLed(0);
# else
    //Open and configure cameras
    webcam.left = VideoCapture();
    webcam.right = VideoCapture();
    assert(webcam.left.open(1));
    assert(webcam.right.open(2));
# endif
    
	//Setup Semi Global Block Matching object
	this->setupDisparity();
}

void StereoPair::updateImages(bool rectify) {
    Mat newFrameL, newFrameR;
    
# ifdef DUO3D
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
    
# else
    //First grab the undecoded frames, as this is a fast operation and thus the delay between captures will be lower
    assert(webcam.left.grab());
    assert(webcam.right.grab());
    assert(webcam.left.retrieve(newFrameL));
    assert(webcam.right.retrieve(newFrameR));
    if (!useColorImages) {
        cvtColor(newFrameL,newFrameL,CV_RGB2GRAY);
        cvtColor(newFrameR,newFrameR,CV_RGB2GRAY);
    }
# endif
    
    if (flipped) {
        flip(newFrameL, newFrameL, 0);  // Flip vertically
        flip(newFrameR, newFrameR, 0);  // Flip vertically
        flip(newFrameL, newFrameL, 1);  // Flip horizontally
        flip(newFrameR, newFrameR, 1);  // Flip horizontally
    }
    rightImage = swapped? newFrameL : newFrameR;
    leftImage = swapped? newFrameR : newFrameL;
    
    if (rectify && canRectify) {
        remap(leftImage, leftImage, rectification.rmap[0][0], rectification.rmap[0][1], CV_INTER_LINEAR, BORDER_TRANSPARENT);
        remap(rightImage, rightImage, rectification.rmap[1][0], rectification.rmap[1][1], CV_INTER_LINEAR, BORDER_TRANSPARENT);
    }
}

// ////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
// |||||||||||||||||||||||||||||||||||||setupDisparity|||||||||||||||||||||||||||||||||||||||||||
// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//////////////////////////////////////////////////

void StereoPair::setupDisparity(){
    sgbm = StereoSGBM();
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

// ////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
// |||||||||||||||||||||||||||||||||||||setupRectification|||||||||||||||||||||||||||||||||||||||
// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//////////////////////////////////////////////////

void StereoPair::setupRectification(String calibrationFile) {
    if(!calibrationFile.empty()){
        calibration_parametersFile = calibrationFile;
    }
    
    Mat R1, R2, P1, P2;
    Mat rmap[2][2];
    Rect validRoi[2];
    
    // Open calibration file
    FileStorage fs(calibration_parametersFile, FileStorage::READ);
    
    // Read calibration file
    if(!fs.isOpened()){
        cout << "Calibration file was not found" << endl;
        cout << "Do you want to calibrate now? (y/n): " << endl;
        return;
    }
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
        
        
        // Save the rectification mappings
        rectification.K = P1;
        rectification.B = P2.at<double>(0, 3) / P2.at<double>(0, 0);
        for(int i=0; i<2; i++)
            for(int j=0; j<2; j++)
                rectification.rmap[i][j] = rmap[i][j];
        canRectify = true;
    }
}

//// ////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
//// ||||||||||||||||||||||||||||||||||||||||rectifyImage||||||||||||||||||||||||||||||||||||||||||
//// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//////////////////////////////////////////////////
//
//Mat StereoPair::rectifyImage(const Mat& unrectifiedImage, const Rectification& recti, bool left)
//{
//	Mat rectifiedImage;
//
//	if(left)
//		remap(unrectifiedImage, rectifiedImage, recti.rmap[0][0], recti.rmap[0][1], CV_INTER_LINEAR, BORDER_TRANSPARENT);
//	else
//		remap(unrectifiedImage, rectifiedImage, recti.rmap[1][0], recti.rmap[1][1], CV_INTER_LINEAR, BORDER_TRANSPARENT);
//
////	rectifiedImage = unrectifiedImage;
//	return rectifiedImage;
//}



void StereoPair::displayImages(bool rectified, bool drawLines) {
    // Create visualization windows
    if (rectified) namedWindow("Rectified stereo images", CV_WINDOW_NORMAL);
    else namedWindow("Uncalibrated stereo images", CV_WINDOW_NORMAL);
    // Reset frame counter
    int frameCount = 0;
    
    for (;;) {
        updateImages(rectified);
        Mat LR = glueTwoImagesHorizontal(leftImage, rightImage);
        
        if (drawLines) {
            cvtColor(LR, LR, COLOR_GRAY2BGR);   // Convert to BGR (RGB) color space for drawing coloured lines.
            for(int h=0; h<LR.rows; h+=25) {
                Point pt1(0, h);
                Point pt2(LR.cols, h);
                line(LR, pt1, pt2, CV_RGB(255, 123, 47), 1);
            }
            
        }
        imshow("Uncalibrated stereo images", LR);
        
        int keyPressed = waitKey(10);
        
        // Save the images if 's' or 'S' key has been pressed
        if( keyPressed==83 || keyPressed==115) {
            saveImage(leftImage, (rectified? "Rectified_L_" + to_string(frameCount) : "Uncalibrated_L_" + to_string(frameCount)), outputDirectory);
            saveImage(rightImage, (rectified? "Rectified_R_" + to_string(frameCount) : "Uncalibrated_R_" + to_string(frameCount)), outputDirectory);
            if (drawLines) saveImage(LR, "StereoPair" + to_string(frameCount), outputDirectory);
            frameCount++;
        }
        
        // Exit if 'esc' key is pressed
        if( keyPressed==27) {
            cout << "ESC key is pressed by user" << endl;
            break;
        }
        
    }
    
    // Close the windows
    if (rectified) destroyWindow("Rectified stereo images");
    else destroyWindow("Uncalibrated stereo images");
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
     resize(leftImage, leftImage, Size(), scaleFactor, scaleFactor, INTER_CUBIC);
     resize(rightImage, rightImage, Size(), scaleFactor, scaleFactor, INTER_CUBIC);
}

// ////////////////////////////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
// |||||||||||||||||||||||||||||||||||updateDisparityImg||||||||||||||||||||||||||||||||||||||||||
// \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\//////////////////////////////////////////////////

void StereoPair::updateDisparityImg(float scaleFactor){
	Mat imL, imR, rleftImage, rrightImage;
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
	resize(leftImage, rleftImage, Size(), scaleFactor, scaleFactor, INTER_AREA);
	resize(rightImage, rrightImage, Size(), scaleFactor, scaleFactor, INTER_AREA);
    sgbm(rleftImage, rrightImage, dsp);
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

void StereoPair::updateImg3D(){
    img3D = Mat(dsp.size(), CV_32FC3);
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
        this->updateImages(true);
		this->updateDisparityImg(scaleFactor);

		if(showImages){
			Mat d1 = glueTwoImagesHorizontal(leftImage, rightImage);
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
            updateImg3D();
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
	int nimages = 16;				//Number of images to take for calibration
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
            this->updateImages(false /*rectify*/);
    		Mat niml = leftImage, nimr = rightImage;
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
                calibDisplay = glueTwoImagesHorizontal(leftImage, rightImage);
                resize(calibDisplay, calibDisplay, Size(), windowResize, windowResize);
            }
            imshow("Live calibration view", calibDisplay);
            
    		// Wait for key press
    		int keyPressed = waitKey(20);

    		// Take images if 'n' or 'N' keys have been pressed
    		if( keyPressed==78 || keyPressed==110)
            {
                if(k == 2){
                    if (leftImage.channels() > 1) cvtColor(leftImage, leftImage, CV_RGB2GRAY);
                    if (rightImage.channels() > 1) cvtColor(rightImage, rightImage, CV_RGB2GRAY);
                    goodImages.push_back(leftImage);
                    goodImages.push_back(rightImage);
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
    FileStorage fs(calibration_parametersFile.c_str(), CV_STORAGE_WRITE);
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
    this->setupRectification();
    this->displayImages(true /*rectified*/, true /*drawLines*/);
}
