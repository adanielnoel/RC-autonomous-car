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

ObstacleScenario::ObstacleScenario(float _width, float _depth, float _squareSize){
    this->width = _width;
    this->depth = _depth;
    this->squareSize = _squareSize;
    int XSquares = ceil(width/squareSize);
    int ZSquares = ceil(depth/squareSize);
    
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

    float scaleFactor = 70.0;   // TODO: this is a dirty trick, the correct dimensions shoudn't need scaling.
    
    for (int x = regionOfInterest.x; x < regionOfInterest.width+regionOfInterest.x; x++) {
        for (int y = regionOfInterest.y; y < regionOfInterest.height+regionOfInterest.y; y++) {
            Point3f point3D;
            point3D.x = float(image3D.at<Vec3f>(y, x).val[0]*scaleFactor);
            point3D.y = float(image3D.at<Vec3f>(y, x).val[1]*scaleFactor);
            point3D.z = float(image3D.at<Vec3f>(y, x).val[2]*scaleFactor);
            points.push_back(Point2f(point3D.x, point3D.z));
        }
    }
    
    for (int i = 0; i < points.size(); i++) {
        int square_x = scenario.at(0).size()-ceil(points.at(i).x/squareSize + scenario.size()/3);
        int square_y = scenario.at(0).size()-(ceil(points.at(i).y/squareSize));
        if ((square_x >= 0 && square_x < scenario.size()) && (square_y >= 0 && square_y < scenario.at(0).size())) {
            scenario.at(square_x).at(square_y) = 1;
            obstaclesDetected = true;
        }
    }
}

void ObstacleScenario::clearScenario(){
    for (int i = 0; i < scenario.size(); i++) {
        for (int j = 0; j < scenario.at(0).size(); j++) {
            scenario.at(i).at(j) = 0;
        }
    }
}

/*
 void ObstacleScenario::populateScenario(Mat &image3D, bool &obstaclesDetected) {
 this->clearScenario();
 
 vector< vector<int> > tempScenario;
 
 if (image3D.empty()){
 return;
 }
 
 float scaleFactor = 70.0;   // TODO: this is a dirty trick, the correct dimensions shoudn't need scaling.
 
 for (int x = regionOfInterest.x; x < regionOfInterest.width+regionOfInterest.x; x++) {
 float Z_average = 0;
 float X = 0;
 for (int y = regionOfInterest.y; y < regionOfInterest.height+regionOfInterest.y; y++) {
 Point3f point3D;
 point3D.x = float(image3D.at<Vec3f>(y, x).val[0]*scaleFactor);
 point3D.y = float(image3D.at<Vec3f>(y, x).val[1]*scaleFactor);
 point3D.z = float(image3D.at<Vec3f>(y, x).val[2]*scaleFactor);
 
 points.push_back(Point2f(point3D.x, point3D.z));
 //Z_average += point3D.z;
 //X = point3D.x;
 //cout << "x = " << point3D.x << "     y = " << point3D.y << "     z = " << point3D.z << endl;
 }
 Z_average = Z_average/regionOfInterest.height;
 if ( Z_average > 0) {
 int square_x = scenario.at(0).size()-ceil(X/squareSize + scenario.size()/3);
 int square_y = scenario.at(0).size()-(ceil(Z_average/squareSize));
 if ((square_x >= 0 && square_x < scenario.size()) && (square_y >= 0 && square_y < scenario.at(0).size())) {
 scenario.at(square_x).at(square_y) = 1;
 obstaclesDetected = true;
 }
 }
 
 }
 }
*/