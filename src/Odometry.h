/*
 * Odometry.h
 *
 *  Created on: Jul 29, 2014
 *      Author: alejandro
 */

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/opencv.hpp"
#include <stdio.h>
#include "cv.h"
#include "StereoPair.h"

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

using namespace cv;
using namespace std;

struct ImgData{
	Mat image;
	vector<KeyPoint> keyPoints;
	Mat descriptors;
};

struct pointAndFeat{
	Point3d point;
	Mat descriptor;
};

class Odometry {
	//Atributes

		//Data storage
	vector<ImgData> imgDataQueue;	//Storage for images and their corresponding key points and descriptors.
	vector<pointAndFeat> pointCloud;//Storage for verified points once they are flushed from the pointFeed
	vector<Point3d> vPointFeed; 	//Every point in vPointFeed has its corresponding descriptor at vDescriptorFeed with matching index/row
	Mat uDescriptorFeed;
	Mat vDescriptorFeed;
	vector<int> sightings;			//Sighting counter for unverified points.
	vector<int> unSightings;		//Unsighting counter for unverified points.
	vector<int> vunSightings;		//Unsighting counter for verified points.
	Mat position;					//Matrix to store position and rotation of the camera.

		//Parameters and thresholds
	int sightingsToVerify;			//how many a new key point has to bee detected to become validated.
	int unSightingsToStore;			//How many times a point has to not be detected to be stored.
	int unSightingsToDelete;		//How many times an unverified descriptor has to be unseen to be deleted.

		//Objects
	FastFeatureDetector detector;
	BriefDescriptorExtractor extractor;
	FlannBasedMatcher matcher;
	StereoPair camera;

public:
	//Constructors and destructors
	Odometry();
	Odometry(StereoPair _camera);
	virtual ~Odometry();

	//Initialization methods
	bool initOdometry(int numFrames);

	//Functions
	bool updateQueue(bool showResult);
	bool processNewFrame(vector<KeyPoint> & kp, Mat & descriptors);
	void incrementSightningsCounters(vector<DMatch> matches, vector<KeyPoint> keyPoints, Mat depthMap);
	Point3d triangulatePos(Point3d point1, Point3d point2, Point3d point3, float dist1, float dist2, float dist3);
};

/*
 * When a new image is received it is matched with the descriptorFeed and increment their sightings.
 * From the validated matches (sightings >= sightningsToVerify) we apply solvePNP to estimate camera position.
 * Knowing the camera position we translate the new unmatched key points and their depth to the reference coordinate system.
 * Then we add the new unmatched points and descriptors to the feed.
 * When a descriptor is not seen, its unSightings count is incremented.
 * If unSightings>=SightingsToStore the point is stored in the pointCloud if it was verified or deleted if it was not.
 */

#endif /* ODOMETRY_H_ */
