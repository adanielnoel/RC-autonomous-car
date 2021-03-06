/*
 * Odometry.cpp
 *
 *  Created on: Jul 26, 2014
 *     Author: Alejandro Daniel Noel
 *     Page: http://futuretechmaker.com
 */

#include "Odometry.h"
#include "StereoPair.h"

Odometry::Odometry(){
	camera = new StereoPair();
}

Odometry::Odometry(StereoPair& _camera){
	camera = &_camera;
    
    // Some dafault values
    maxEpipolarDifference = 15.0;

	/*
	 * Create the feature detector. Types:
	 *
	 * "FAST" – FastFeatureDetector
     * "STAR" – StarFeatureDetector
     * "SIFT" – SIFT (nonfree module)
     * "SURF" – SURF (nonfree module)
     * "ORB" – ORB
     * "BRISK" – BRISK
     * "MSER" – MSER
     * "GFTT" – GoodFeaturesToTrackDetector
     * "HARRIS" – GoodFeaturesToTrackDetector with Harris detector enabled
     * "Dense" – DenseFeatureDetector
     * "SimpleBlob" – SimpleBlobDetector
	 */
	detector = FeatureDetector::create("ORB");

	/*
	 * Create descriptor extractor. Types:
	 *
	 * "SIFT" – SIFT
     * "SURF" – SURF
     * "BRIEF" – BriefDescriptorExtractor
     * "BRISK" – BRISK
     * "ORB" – ORB
     * "FREAK" – FREAK
	 */
	descriptor = DescriptorExtractor::create("ORB");

	/*
	 * Create feature matcher. Types:
	 *
     * BruteForce (it uses L2 )
     * BruteForce-L1
     * BruteForce-Hamming
     * BruteForce-Hamming(2)
     * FlannBased
	 */
	matcher = DescriptorMatcher::create("BruteForce");
}

Odometry::~Odometry(){

}

bool Odometry::updateOdometry(){
	//TODO: explained at the end of the header file
    camera->updateImages();
	Mat leftImage = camera->leftImage;
	Mat rightImage = camera->rightImage;
	vector<vector<KeyPoint> > newKeyPoints;
	vector<Mat> newDescriptors;

	//////////Get valid key points and descriptors (those which 3D coordinates can be calculated)///////
	bool enoughKeypoints = processNewFrame(leftImage, rightImage, newKeyPoints, newDescriptors);
	//Don't proceed is there aren't enough key points (less than MINIMUM_POINTS_FOR_PNP)
	if(!enoughKeypoints) return false;


	//////////Match with previous frame////////////
//	vector<DMatch> survivingMatches = filteredMatch(newKeyPoints, kp0, newDescriptors, desc0, DO_CROSS_CHECK);
	//Don't proceed is there aren't enough matches
//	if((int)survivingMatches.size() < 3) return false;

	/* code from http://stackoverflow.com/questions/22441782/vtk-camera-pose-from-opencvs-estimated-pose-from-solvepnp
  cv::Mat op(model_points);

  cv::Mat rvec;
  cv::Mat tvec;

  // op = the 3D coordinates of the markers in the scene
  // ip = the 2D coordinates of the markers in the image
  // camera = the intrinsic camera parameters (for calibration)
  // dists = the camera distortion coefficients
  cv::solvePnP(op, *ip, camera, dists, rvec, tvec, false, CV_ITERATIVE);

  cv::Mat rotM;
  cv::Rodrigues(rvec, rotM);

  rotM = rotM.t();

  cv::Mat rtvec = -(rotM*tvec);
	 */

	return true;
}

vector<Point3f> Odometry::localToGlobalCoords(vector<Point3f> localCoordPoints, Mat T, Mat R){
	// TODO: This function does nothing!
    return localCoordPoints;
}

void Odometry::computeFeatures(Mat& img, vector<KeyPoint> & kp, Mat & descriptors){
	detector->detect(img, kp);
	descriptor->compute(img, kp, descriptors);
}

bool Odometry::processNewFrame(Mat& IL, Mat& IR, vector<vector<KeyPoint> > & kpts, vector<Mat> descriptors){

	vector<KeyPoint> kpL1, kpR1;
	Mat	descL1, descR1;

	computeFeatures(IL, kpL1, descL1);
	computeFeatures(IR, kpR1, descR1);

	if(kpR1.size() == 0 || kpR1.size() == 0){
		return false;
	}
	///////////Match left and right image////////////////
	vector<DMatch> matches = filteredMatch(kpL1, kpR1, descL1, descR1, DO_CROSS_CHECK);

	vector<KeyPoint> matchedKPL1, matchedKPR1;
	Mat matchedDescL1(32, (int) matches.size(), descL1.type());
	Mat matchedDescR1(32, (int) matches.size(), descR1.type());
	for(unsigned int i = 0; i < matches.size(); i++){
		matchedKPL1.push_back(kpL1.at(matches.at(i).queryIdx));
		matchedKPR1.push_back(kpR1.at(matches.at(i).trainIdx));
		matchedDescL1.row(i) = (matchedDescL1.row(matches.at(i).queryIdx));
		matchedDescR1.row(i) = (matchedDescR1.row(matches.at(i).trainIdx));
	}

	/*//////////////3D points from stereo matches///////////////
	Mat Q = camera->getDispToDepthMat();
	float d = matches.at(i).distance;
	float X = pt.x * Q.at<double>(0, 0) + Q.at<double>(0, 3);
	float Y = pt.y * Q.at<double>(1, 1) + Q.at<double>(1, 3);
	float Z = Q.at<double>(2, 3);
	float W = d * Q.at<double>(3, 2) + Q.at<double>(3, 3);
	X /= W;
	Y /= W;
	Z /= W;

	Point3f pt3(X, Y, Z);
	///////////////////////////////////////////////////////////*/

	return true;
}

Mat Odometry::findCommonDescriptors(Mat queryDesc, Mat trainDesc, vector<DMatch> & matches){	//commonDesc picks the matching descriptors from queryDesc
	//this->matchDescriptors(queryDesc, trainDesc, matches, false);
	int descriptorsFound = (int) matches.size();
	Mat commonDesc(descriptorsFound, 32, 0);	//Descriptor matrices have 32 columns
	for(int i=0; i<descriptorsFound; i++){
		commonDesc.row(i) = queryDesc.row(matches.at(i).queryIdx);
	}
	return commonDesc;
}



vector<DMatch> Odometry::filteredMatch(vector<KeyPoint> kp1, vector<KeyPoint> kp2, Mat& desc1, Mat& desc2, bool doCrossCheck, bool doEpipolarFilter){
    vector<DMatch> FMatches;
    if(doCrossCheck){
		int knn = 1;
		FMatches.clear();
	    vector<vector<DMatch> > matches12, matches21;
	    matcher->knnMatch( desc1, desc2, matches12, knn );
	    matcher->knnMatch( desc2, desc1, matches21, knn );
	    for( size_t m = 0; m < matches12.size(); m++ )
	    {
	        bool foundCrossCheck = false;
	        for( size_t fk = 0; fk < matches12[m].size(); fk++ )
	        {
	            DMatch forward = matches12[m][fk];

	            for( size_t bk = 0; bk < matches21[forward.trainIdx].size(); bk++ )
	            {
	                DMatch backward = matches21[forward.trainIdx][bk];
	                if( backward.trainIdx == forward.queryIdx )
	                {
	                	FMatches.push_back(forward);
	                    foundCrossCheck = true;
	                    break;
	                }
	            }
	            if( foundCrossCheck ) break;
	        }
	    }
	}
	else{
		matcher->match(desc1, desc2, FMatches);
	}
    
    if (doEpipolarFilter) {
        vector<DMatch> _FMatches;
        for (unsigned int i = 0; i < FMatches.size(); i++) {
            int id1 = FMatches.at(i).queryIdx;
            int id2 = FMatches.at(i).trainIdx;
            float hDiff = abs(kp1.at(id1).pt.y - kp2.at(id2).pt.y);
            if (hDiff <= maxEpipolarDifference) {
                _FMatches.push_back(FMatches.at(i));
            }
        }
        FMatches = _FMatches;
    }

    return FMatches;
}

void Odometry::showLRMatches(){
	Mat drawImg1;
  //  Mat drawImg2; //For own drawing function

    namedWindow("Stereo matches", CV_WINDOW_NORMAL);

	while(1){
        camera->updateImages();
        Mat leftImage = camera->leftImage;
        Mat rightImage = camera->rightImage;
		vector<KeyPoint> kpL;
		vector<KeyPoint> kpR;
		Mat descL;
		Mat descR;
		vector<DMatch> filteredMatches;

		computeFeatures(leftImage, kpL, descL);
		computeFeatures(rightImage, kpR, descR);

		if(kpL.empty() || kpR.empty()) continue;

		filteredMatches = filteredMatch(kpL, kpR, descL, descR, !DO_CROSS_CHECK, true);

		if(filteredMatches.empty()) continue;

        drawMatches(leftImage, kpL, rightImage, kpR, filteredMatches, drawImg1, Scalar(0, 255, 0), Scalar(255, 0, 0));

       /* //OWN DRAWING FUNCTION

	    drawImg2 = imgL.clone();
        cvtColor(drawImg2, drawImg2, CV_GRAY2RGB);
	    cout << "Own match drawing" << endl;
	    cout << "fltrdmatchs: " << filteredMatches.size() << endl;
	    for(unsigned int i=0; i<filteredMatches.size(); i++){
	    	Point2f point1 = kpL[filteredMatches[i].queryIdx].pt;
	    	Point2f point2 = kpR[filteredMatches[i].trainIdx].pt;
	    	if(abs(point1.y - point2.y) <= maxEpipolarDifference){
	    		circle(drawImg2, point1, 2, Scalar(0, 255, 0));
	    		line(drawImg2, point1, point2, Scalar(255, 0, 0));
	        }
	        else circle(drawImg2, point1, 2, Scalar(0, 0, 255));
	    }
*/

		//////////////////////////////////////////////////////////////

		imshow("Stereo matches", drawImg1);

		int keyPressed = int(char(waitKey(1)));

		// Exit when esc key is pressed
        if( keyPressed== 27) break;
	}
    destroyWindow("Stereo matches");
    for(int i = 0; i < 10; i++) waitKey(1);
}
