//
//  ObstacleScenario.cpp
//  Autonomous_RC_Car
//
//  Created by Alejandro Daniel Noel on 07/01/15.
//
//

#include <stdio.h>
#include "ObstacleScenario.h"
#include "StereoPair.h"

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

void ObstacleScenario::populateScenario(Mat dispImg){
    bool useCustomReprojectionMethod = false;
    this->clearScenario();
    Mat image3D = stereoPair.reprojectTo3D(dispImg);
    regionOfInterest = Rect(0, 0, 80, 80);
    //imshow("im3d",image3D);
    //waitKey();
    for (int i = regionOfInterest.x; i < regionOfInterest.width-regionOfInterest.x; i++) {
        float Z_average = 0;
        float X = 0;
        for (int j = regionOfInterest.y; j < regionOfInterest.height-regionOfInterest.y; j++) {
            
            float x=0.0, y=0.0, z=0.0;
            if (useCustomReprojectionMethod){
                double disp = dispImg.at<double>(Point(i, j));
                if (disp < 0.1) continue;
                stereoPair.getPixel3Dcoords(i, j, disp, x, y, z);
            }
            else {
                Vec3f point = image3D.at<Vec3f>(Point(i, j));
                x = point.val[0];
                y = point.val[1];
                z = point.val[2];
            }
                Z_average += z;
                X = x;
            //cout << "x = " << x << "     y = " << y << "     z = " << z << endl;
        }
        Z_average = Z_average/regionOfInterest.height;
        if ( Z_average > 0) {
            int square_x = ceil(X/squareSize);
            int square_y = ceil(Z_average/squareSize);
            if (square_x < scenario.size() && square_y < scenario.at(0).size()) {
                scenario.at(square_x).at(square_y) = 1;
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