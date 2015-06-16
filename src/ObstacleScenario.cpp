//
//  ObstacleScenario.cpp
//  Autonomous_RC_Car
//
//  Created on: Jul 26, 2014
//     Author: Alejandro Daniel Noel
//     Page: http://futuretechmaker.com
//

#include <stdio.h>
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
    
    vector<vector<int> > newScen;
    
    for (int i = 0; i < XSquares; i++) {
        vector<int> newCol;
        for (int j = 0; j < ZSquares; j++) {
            newCol.push_back(0);
        }
        newScen.push_back(newCol);
    }
    
    scenario = newScen;
}

ObstacleScenario::ObstacleScenario(int _XSquares, int _YSquares){
    int XSquares = _XSquares;
    int ZSquares = _YSquares;
    
    vector<vector<int> > newScen;
    
    for (int i = 0; i < XSquares; i++) {
        vector<int> newCol;
        for (int j = 0; j < ZSquares; j++) {
            newCol.push_back(0);
        }
        newScen.push_back(newCol);
    }
    
    scenario = newScen;
}



void ObstacleScenario::populateScenario(Mat &image3D, bool &obstaclesDetected) {
    this->clearScenario();
    
    if (image3D.empty()){
        return;
    }

    float scaleFactor = 70.0;
    
    for (int x = regionOfInterest.x; x < regionOfInterest.width+regionOfInterest.x; x++) {
        float Z_average = 0;
        float X = 0;
        for (int y = regionOfInterest.y; y < regionOfInterest.height+regionOfInterest.y; y++) {
            Point3f point3D;
            point3D.x = float(image3D.at<Vec3f>(y, x).val[0]*scaleFactor);
            point3D.y = float(image3D.at<Vec3f>(y, x).val[1]*scaleFactor);
            point3D.z = float(image3D.at<Vec3f>(y, x).val[2]*scaleFactor);
            
            Z_average += point3D.z;
            X = point3D.x;
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

void ObstacleScenario::clearScenario(){
    vector<vector<int> > newScen;
    for (int i = 0; i < scenario.size(); i++) {
        vector<int> newCol;
        for (int j = 0; j < scenario.at(0).size(); j++) {
            newCol.push_back(0);
        }
        newScen.push_back(newCol);
    }
    
    scenario = newScen;
}