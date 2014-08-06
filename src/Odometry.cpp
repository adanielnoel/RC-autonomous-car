/*
 * Odometry.cpp
 *
 *  Created on: Jul 29, 2014
 *      Author: alejandro
 */

#include "Odometry.h"

Odometry::Odometry(StereoPair _camera){
	sightingsToVerify = 3;
	unSightingsToStore = 3;
	unSightingsToDelete = 3;
	detector = FastFeatureDetector(30);
	extractor = BriefDescriptorExtractor();
	matcher = FlannBasedMatcher();//NORM_HAMMING);
	camera = _camera;
}

Odometry::~Odometry() {
	// TODO Auto-generated destructor stub
}


bool Odometry::initOdometry(int numFrames){
	vector<Mat> descriptors;

	for(int i=0; i<(numFrames-1); i++){
		vector<KeyPoint> newKeypoints;
		Mat newDescriptors;
		if(!this->processNewFrame(newKeypoints, newDescriptors))return false;
		descriptors.push_back(newDescriptors);
	}
	matcher.add(descriptors);
	matcher.train();

	vector<KeyPoint> newKeypoints;
	Mat newDescriptors;
	if(!this->processNewFrame(newKeypoints, newDescriptors))return false;
	vector<DMatch> matches;
	matcher.match(newDescriptors, matches);

	return true;
}

bool Odometry::updateQueue(bool showResult){

	vector<KeyPoint> newKeypoints;
	Mat newDescriptors;
	if(!this->processNewFrame(newKeypoints, newDescriptors))return false;

	if(newKeypoints.size() > 0){// && descriptorFeed.rows > 0){

		vector<DMatch> matches;
		matcher.match(newDescriptors, matches);
		int msize = matches.size();

		//If the new image has common matches with the others at imgDataQueue, add it to the queue.
		if(!matches.empty()){
			camera.updateDepthMap();
			Mat depthMap = camera.getDepthMap();
			this->incrementSightningsCounters(matches, newKeypoints, depthMap);

			/*if(showResult){
				// drawing the results
				namedWindow("matches", 1);
				Mat img_matches;
				drawMatches(trainImages.at(1),
							trainKeypoints.at(1),
							img,
							newKeypoints,
							matches,
							img_matches);
				imshow("matches", img_matches);
				//cout << "Queue size: " << imgDataQueue.size() << endl;
			}*/
		}

		cout << (newKeypoints.size() == matches.size())<< endl;
	}

	return true;
}

void Odometry::incrementSightningsCounters(vector<DMatch> matches, vector<KeyPoint> keyPoints, Mat depthMap){
	/*
	vector<ImgData> imgDataQueue;	//Storage for images and their corresponding key points and descriptors.
	vector<pointAndFeat> pointCloud;//Storage for verified points once they are flushed from the pointFeed
	vector<Point3d> pointFeed; 		//Every point in pointFeed has its corresponding descriptor at descriptorFeed with matching index/row
	Mat descriptorFeed;
	vector<int> sightings;
	vector<int> unSightings;
	int sightingsToVerify;	//how many a new key point has to bee detected to become validated
	int sightingsToStore;
	*/
	bool descriptorsSeen[descriptorFeed.rows];
	for(int i=0; i<matches.size(); i++){
		cout << "queryIdx: " << matches.at(i).queryIdx << "   trainIdx: " << matches.at(i).trainIdx << endl;
		sightings.at(matches.at(i).trainIdx)++;
		descriptorsSeen[matches[i].queryIdx] = true;
	}
	for(int i=0; i<descriptorFeed.rows; i++){
		if(!descriptorsSeen[i]){
			unSightings.at(i)++;
		}
	}
	/*
	Point2f pixel = keyPoints.at(i).pt;
	Point3d point3D;
	if(camera.pixelToPoint(depthMap, pixel, point3D));
	*/
}


bool Odometry::processNewFrame(vector<KeyPoint>& kp, Mat& descriptors){
	if(!camera.updateRectifiedPair()) return false;
	Mat img = camera.getMainImg();
	if(img.empty()) return false;

	detector.detect(img, kp);
	extractor.compute(img, kp, descriptors);

	return true;
}
