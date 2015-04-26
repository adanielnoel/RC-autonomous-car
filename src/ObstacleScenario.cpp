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
    cout << "Size: " << scenario.size() << "x" << scenario.at(1).size() << endl;
}

void ObstacleScenario::populateScenario(Mat &image3D, bool &obstaclesDetected) {
    this->clearScenario();
    
    if (image3D.empty()){
        return;
    }

    for (int i = regionOfInterest.x; i < regionOfInterest.width+regionOfInterest.x; i++) {
        float Z_average = 0;
        float X = 0;
        for (int j = regionOfInterest.y; j < regionOfInterest.height+regionOfInterest.y; j++) {
            Point3f point3D;
            point3D.x = image3D.at<Vec3f>(j, i).val[0];
            point3D.y = image3D.at<Vec3f>(j, i).val[1];
            point3D.z = image3D.at<Vec3f>(j, i).val[2];
            Z_average += point3D.z;
            X = point3D.x;
            //cout << "point3D.x = " << point3D.x << "     y = " << y << "     z = " << z << endl;
        }
        Z_average = Z_average/regionOfInterest.height;
        if ( Z_average > 0) {
            int square_x = ceil(X/squareSize);
            int square_y = ceil(Z_average/squareSize);
            if ((square_x >= 0 && square_x < scenario.size()) && (square_y >= 0 && square_y < scenario.at(0).size())) {
                scenario.at(square_x).at(square_y) = 1;
                obstaclesDetected = true;
                cout << "Square: " << square_x << ", " << square_y << endl;
            }
        }
        
    }
    cout << "***************************************************************" << endl;
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