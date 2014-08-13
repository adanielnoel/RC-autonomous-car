/*
 * Odometry.cpp
 *
 *  Created on: Jul 29, 2014
 *      Author: alejandro
 */

#include "Odometry.h"

int Odometry::AUTO_REPEAT = 0;
int Odometry::PROMPT_REPEAT = 1;

Odometry::Odometry(StereoPair _camera){
	sightingsToVerify = 3;
	unSightingsToStore = 3;
	unSightingsToDelete = 3;
	detector = FastFeatureDetector(30);
	extractor = BriefDescriptorExtractor();
	matcher = BFMatcher(NORM_HAMMING, true); //Cross check enabled
	camera = _camera;
}

Odometry::~Odometry() {
	// TODO Auto-generated destructor stub
}


bool Odometry::initOdometry(int numFrames, bool promptRepeat){	//finds common descriptors in some frames (numFrames)
	Mat descriptorQueue;
	Mat finalDescriptors;
	vector<KeyPoint> newKeypoints;
	Mat newDescriptors;
	int minimumPointsToProceed = 5;
	for(int j=1; ; j++){
		for(int i=0; i<numFrames; i++){
			if(this->processNewFrame(newKeypoints, newDescriptors)){ //Method returns true if features are found

				//First iteration: fill descriptor queue with the descriptors found.
				if(i == 0){
					descriptorQueue = newDescriptors;
					cout << "\n*******Starting odometry********" << endl;
					cout << "Iteration number " << i << "." << endl;
					cout << "Initial descriptorQueue.rows: " << descriptorQueue.rows << endl;
					cout << "**********************************" << endl;
				}

				//Last iteration: fill finalDescriptors with the ones that have survived all the iterations
				else if(i == (numFrames-1)){
					cout << "Iteration number " << i << " (last)." << endl;
					vector<DMatch> matches;
					matcher.match(newDescriptors, descriptorQueue, matches);
					finalDescriptors = this->findCommonDescriptors(newDescriptors, descriptorQueue);
				}

				//Intermediate iterations: keep on the queue only the descriptors that survive the iteration
				else{
					cout << "Iteration number " << i << "." << endl;
					descriptorQueue = this->findCommonDescriptors(newDescriptors, descriptorQueue);
					cout << "newDescriptors.rows: " << newDescriptors.rows << endl;
					cout << "descriptorQueue.rows: " << descriptorQueue.rows << endl;
					cout << "**********************************" << endl;
				}
			}

			else cout << "No features found!!!" << endl;
		}

		//Print initialization results
		cout << "\n*******Odometry initialization attempt: " << j << "*******" << endl;
		cout << "Found descriptors: " << finalDescriptors.rows << endl;

		//Check if the number of points found is enough
		if(finalDescriptors.rows < minimumPointsToProceed){
			cout << "ODOMETRY ERROR: Found less than " << minimumPointsToProceed << " points. Tracking may fail!" << endl;
			if(promptRepeat){
				cout << "Do you wish to repeat initialization? (y/n)" << endl;
				string _repeat;
				cin >> _repeat;
				if(_repeat=="n")break;
			}
			cout << "*****Retrying odometry initialization*****" << endl;
		}
		else{
			cout << "Initialization successful" << endl;
			break;
		}
	}
	if(finalDescriptors.rows < minimumPointsToProceed) return false;
	else return true;
}

bool Odometry::updateQueue(bool showResult){

	vector<KeyPoint> newKeypoints;
	Mat newDescriptors;
	if(!this->processNewFrame(newKeypoints, newDescriptors))return false;

	if(newKeypoints.size() > 0){		// && descriptorFeed.rows > 0){

		vector<DMatch> matches;
		matcher.match(newDescriptors, matches);
		int msize = matches.size();

		//If the new image has common matches with the others at imgDataQueue, add it to the queue.
		if(!matches.empty()){
			camera.updateDisparityImg();
			Mat depthMap = camera.getDisparityImg();
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
	/*bool descriptorsSeen[descriptorFeed.rows];
	for(int i=0; i<matches.size(); i++){
		cout << "queryIdx: " << matches.at(i).queryIdx << "   trainIdx: " << matches.at(i).trainIdx << endl;
		sightings.at(matches.at(i).trainIdx)++;
		descriptorsSeen[matches[i].queryIdx] = true;
	}
	for(int i=0; i<descriptorFeed.rows; i++){
		if(!descriptorsSeen[i]){
			unSightings.at(i)++;
		}
	}*/
	/*
	Point2f pixel = keyPoints.at(i).pt;
	Point3d point3D;
	if(camera.pixelToPoint(depthMap, pixel, point3D));
	*/
}


bool Odometry::processNewFrame(vector<KeyPoint> & kp, Mat & descriptors){
	if(!camera.updateRectifiedPair()) return false;
	Mat img = camera.getMainImg();
	if(img.empty()) return false;

	detector.detect(img, kp);
	extractor.compute(img, kp, descriptors);

	return descriptors.rows; 					//If no descriptors are found, return false
}

Mat Odometry::findCommonDescriptors(Mat queryDesc, Mat trainDesc){	//commonDesc picks the matching descriptors from queryDesc
	vector<DMatch> matches;
	matcher.match(queryDesc, trainDesc, matches);
	int descriptorsFound = matches.size();
	Mat commonDesc(descriptorsFound, 32, 0);	//Descriptor matrices have 32 columns
	for(int i=0; i<descriptorsFound; i++){
		commonDesc.row(i) = queryDesc.row(matches.at(i).queryIdx);
	}
	cout << "Matches: " << descriptorsFound << endl;
	return commonDesc;
}


vector<Point3d> Odometry::featuresToPoints(Mat img3D, vector<KeyPoint> keyPoints,vector<DMatch> validKeyPoints){
	vector<Point3d> points;
	for(int i=0; i< validKeyPoints.size(); i++){
		Point pixel = keyPoints.at(validKeyPoints.at(i).queryIdx).pt;
		Vec3d xyzVector = img3D.at<Vec3d>(pixel);
		Point3d newPoint = xyzVector;
		points.push_back(newPoint);
	}
	return points;
}
