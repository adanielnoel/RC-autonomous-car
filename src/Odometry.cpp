/*
 * Odometry.cpp
 *
 *  Created on: Jul 29, 2014
 *      Author: alejandro
 */

#include "Odometry.h"

Odometry::Odometry() {
	// TODO Auto-generated constructor stub
	queueSize = 0;

}

Odometry::~Odometry() {
	// TODO Auto-generated destructor stub
}

void Odometry::updateQueue(Mat img, bool showResult){
	imgFeed.push_back(img);
	queueSize++;
	printf("\nQueue size : %i", queueSize);
	if(queueSize > 1){
		// detecting keypoints
		FastFeatureDetector detector(30);
		vector<KeyPoint> keypoints1, keypoints2;
		Mat img1 = imgFeed.at(queueSize-1);
		Mat img2 = imgFeed.at(queueSize-2);
		detector.detect(img1, keypoints1);
		detector.detect(img2, keypoints2);

		//Only compute descriptors and match them if there are key points in both images
		if(keypoints1.size() != 0 && keypoints2.size() != 0){
			// computing descriptors
			BriefDescriptorExtractor extractor;
			Mat descriptors1, descriptors2;
			extractor.compute(img1, keypoints1, descriptors1);
			extractor.compute(img2, keypoints2, descriptors2);

			// matching descriptors
			BFMatcher matcher(NORM_HAMMING);
			vector<DMatch> matches;
			matcher.match(descriptors1, descriptors2, matches);

			if(showResult){
				// drawing the results
				namedWindow("matches", 1);
				Mat img_matches;
				drawMatches(img1, keypoints1, img2, keypoints2, matches, img_matches);
				imshow("matches", img_matches);
			}
		}
	}
}
