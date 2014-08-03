/*
 * ImgData.h
 *
 *  Created on: Aug 3, 2014
 *      Author: alejandro
 */
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/opencv.hpp"
#include <stdio.h>
#include "cv.h"

using namespace cv;
using namespace std;

#ifndef IMGDATA_H_
#define IMGDATA_H_

class ImgData {
	Mat image;

public:
	ImgData();
	virtual ~ImgData();
};

#endif /* IMGDATA_H_ */
