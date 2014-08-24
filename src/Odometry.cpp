/*
 * Odometry.cpp
 *
 *  Created on: Jul 29, 2014
 *      Author: alejandro
 */

#include "Odometry.h"

const int Odometry::AUTO_REPEAT = 0;
const int Odometry::PROMPT_REPEAT = 1;

Odometry::Odometry(StereoPair _camera){
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
	detector = FeatureDetector::create("FAST");

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
	descriptor = DescriptorExtractor::create("BRIEF");

	/*
	 * Create feature matcher. Types:
	 *
     * BruteForce (it uses L2 )
     * BruteForce-L1
     * BruteForce-Hamming
     * BruteForce-Hamming(2)
     * FlannBased
	 */
	matcher = DescriptorMatcher::create("BruteForce-Hamming(2)");

	camera = _camera;
}

bool Odometry::updateOdometry(){
	//TODO: explained at the end of the header file
	return true;
}

bool Odometry::processNewFrame(Mat& image, vector<KeyPoint> & kp, Mat & descriptors){
	detector->detect(image, kp);
	descriptor->compute(image, kp, descriptors);

	return descriptors.rows; 					//If no descriptors are found, return false
}

Mat Odometry::findCommonDescriptors(Mat queryDesc, Mat trainDesc, vector<DMatch> & matches){	//commonDesc picks the matching descriptors from queryDesc
	//this->matchDescriptors(queryDesc, trainDesc, matches, false);
	int descriptorsFound = matches.size();
	Mat commonDesc(descriptorsFound, 32, 0);	//Descriptor matrices have 32 columns
	for(int i=0; i<descriptorsFound; i++){
		commonDesc.row(i) = queryDesc.row(matches.at(i).queryIdx);
	}
	return commonDesc;
}



vector<DMatch> Odometry::filteredMatch(vector<KeyPoint> kpL, vector<KeyPoint> kpR, Mat& descL, Mat& descR, bool doCrossCheck){

    vector<DMatch> FMatches, filteredMatches;

    if(doCrossCheck){
		int knn = 1;
		FMatches.clear();
	    vector<vector<DMatch> > matches12, matches21;
	    matcher->knnMatch( descL, descR, matches12, knn );
	    matcher->knnMatch( descR, descL, matches21, knn );
	    for( size_t m = 0; m < matches12.size(); m++ )
	    {
	        bool findCrossCheck = false;
	        for( size_t fk = 0; fk < matches12[m].size(); fk++ )
	        {
	            DMatch forward = matches12[m][fk];

	            for( size_t bk = 0; bk < matches21[forward.trainIdx].size(); bk++ )
	            {
	                DMatch backward = matches21[forward.trainIdx][bk];
	                if( backward.trainIdx == forward.queryIdx )
	                {
	                	FMatches.push_back(forward);
	                    findCrossCheck = true;
	                    break;
	                }
	            }
	            if( findCrossCheck ) break;
	        }
	    }
	}
	else{
		matcher->match(descL, descR, FMatches);
	}

    Mat H_LR;

    vector<int> queryIdxs( FMatches.size() ), trainIdxs( FMatches.size() );
    for( size_t i = 0; i < FMatches.size(); i++ )
    {
        queryIdxs[i] = FMatches[i].queryIdx;
        trainIdxs[i] = FMatches[i].trainIdx;
    }

    if( ransacReprojThreshold >= 0 )
    {
        cout << "< Computing homography (RANSAC)..." << endl;
        vector<Point2f> points1; KeyPoint::convert(kpL, points1, queryIdxs);
        vector<Point2f> points2; KeyPoint::convert(kpR, points2, trainIdxs);
        H_LR = findHomography( Mat(points1), Mat(points2), CV_RANSAC, ransacReprojThreshold );
        cout << ">" << endl;
    }

    Mat drawImg1;
    Mat drawImg2;
    if( !H_LR.empty() ) // filter outliers
    {
        vector<char> matchesMask( FMatches.size(), 0 ); //init with filteredMatches.size and all values to 0
        vector<Point2f> points1; KeyPoint::convert(kpL, points1, queryIdxs);
        vector<Point2f> points2; KeyPoint::convert(kpR, points2, trainIdxs);
        Mat points1t; perspectiveTransform(Mat(points1), points1t, H_LR);

        double maxInlierDist = ransacReprojThreshold < 0 ? 3 : ransacReprojThreshold;
        for( size_t i1 = 0; i1 < points1.size(); i1++ )
        {
            if( norm(points2[i1] - points1t.at<Point2f>((int)i1,0)) <= maxInlierDist ) // inlier
                matchesMask[i1] = 1;
        }

        for(unsigned int i=0; i<FMatches.size(); i++){
        	if(matchesMask.at(i) != 0) filteredMatches.push_back(FMatches[i]);
        }
    }
    return filteredMatches;
}


/*---------------------------------------------------------------------------*
 * TESTING AREA
 *---------------------------------------------------------------------------*/

void Odometry::showLRMatches(){
	float epiHThres = 10000;	//set to very high number to take no effect
	float epiWThres = 10000;//set to very high number to take no effect

	Mat drawImg1;
    Mat drawImg2; //For own drawing function

	while(1){
		camera.updateRectifiedPair();
		Mat imgL = camera.getImgl();
		Mat imgR = camera.getImgr();
		vector<KeyPoint> kpL;
		vector<KeyPoint> kpR;
		Mat descL;
		Mat descR;
		vector<DMatch> filteredMatches;

		this->processNewFrame(imgL, kpL, descL);
		this->processNewFrame(imgR, kpR, descR);

		if(descL.rows == 0 || descR.rows == 0) continue;

		bool doCrossCheck = true; //Enabling cross check ensures that each feature has one only match
		filteredMatches = this->filteredMatch(kpL, kpR, descL, descR, doCrossCheck);

		if(filteredMatches.size() <= 4) continue;

        //drawMatches( imgL, kpL, imgR, kpR, filteredMatches, drawImg1, Scalar(0, 255, 0), Scalar(255, 0, 0));

	        //OWN DRAWING FUNCTION

	    drawImg2 = imgL.clone();
        cvtColor(drawImg2, drawImg2, CV_GRAY2RGB);
	    cout << "Own match drawing" << endl;
	    for(unsigned int i=0; i<filteredMatches.size(); i++){
	    	Point2f point1 = kpL[filteredMatches[i].queryIdx].pt;
	    	Point2f point2 = kpR[filteredMatches[i].trainIdx].pt;
	    	if(abs(point1.y - point2.y)<=epiHThres){
	    		circle(drawImg2, point1, 2, Scalar(0, 255, 0));
	    		line(drawImg2, point1, point2, Scalar(255, 0, 0));
	        }
	        else circle(drawImg2, point1, 2, Scalar(0, 0, 255));
	    }


		///////////////////////////////////////////////////////////////

		namedWindow("Stereo matches", CV_WINDOW_AUTOSIZE);

		imshow("Stereo matches", drawImg2);

		int keyPressed = waitKey(0);

		// Exit when esc key is pressed
        if( keyPressed== 27) break;
	}
}
