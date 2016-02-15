//
//  ObstacleScenario.cpp
//  Autonomous_RC_Car
//
//  Created on: Jul 26, 2014
//     Author: Alejandro Daniel Noel
//     Page: http://futuretechmaker.com
//

#include "ObstacleScenario.h"

using namespace std;

ObstacleScenario::ObstacleScenario(){
    
}

ObstacleScenario::ObstacleScenario(float _width, float _depth, float _squareSize, float _minY, float _maxY) {
    this->width = _width;
    this->depth = _depth;
    this->squareSize = _squareSize;
    this->minY = _minY;
    this->maxY = _maxY;
    int XSquares = ceil(width/squareSize);
    int ZSquares = ceil(depth/squareSize);
    
    this->scaleFactor = 50.0;
    
    //  Create an empty scenario of the given dimensions
    scenario = {};
    for (int i = 0; i < XSquares; i++) {
        vector<int> newCol;
        for (int j = 0; j < ZSquares; j++) {
            newCol.push_back(0);
        }
        scenario.push_back(newCol);
    }
}


void ObstacleScenario::populateScenario(Mat &image3D, bool &obstaclesDetected) {
    clearScenario();
    if (image3D.empty()) return;
    
    points.clear(); // Remove all previous points
    
    # pragma omp parallel for
    for(int i = 0; i < image3D.cols; i++){
        for(int j = 0; j < image3D.rows; j++){
            Point3f point3D;
            point3D.x = float(image3D.at<Vec3f>(j, i).val[0]*scaleFactor);
            point3D.y = float(image3D.at<Vec3f>(j, i).val[1]*scaleFactor);
            point3D.z = float(image3D.at<Vec3f>(j, i).val[2]*scaleFactor);
            if(point3D.y > minY && point3D.y > maxY) {
                if(point3D.x > -width/2 && point3D.x < width/2) {
                    if (point3D.z >= 0.0 && point3D.z < depth) {
                        // Add points converted to scenario coordinates (origin at top-left corner)
                        points.push_back(Point2f(point3D.x + width/2, depth - point3D.z));
                    }
                }
            }
        }
    }
    
    # pragma omp parallel for
    for (int i = 0; i < points.size(); i++) {
        int square_x = int(points.at(i).x/squareSize);
        int square_y = int(points.at(i).y/squareSize);
        scenario.at(square_x).at(square_y) = 1;
        obstaclesDetected = true;
    }
}

void ObstacleScenario::clearScenario(){
    for (int i = 0; i < scenario.size(); i++) {
        for (int j = 0; j < scenario.at(0).size(); j++) {
            scenario.at(i).at(j) = 0;
        }
    }
}