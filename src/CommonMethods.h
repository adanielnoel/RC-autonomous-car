//
//  CommonFunctions.h
//  Autonomous_RC_Car
//
//  Created by Alejandro Daniel Noel on 30/12/15.
//
//

#ifndef CommonFunctions_h
#define CommonFunctions_h

float mapValue(float val, float min1, float max1, float min2, float max2){
    bool invertRet = false;
    if (min2 > max2) {
        float tmpMin = min2;
        max2 = min2;
        min2 = tmpMin;
        invertRet = true;
    }
    return (((val - min1)*(max2-min2))/(max1-min1))+min2;
}

float constrain(float val, float min, float max){
    if (min > max) {
        float tmp = max;
        max = min;
        min = tmp;
    }
    if (val < min) val = min;
    else if (val > max) val = max;
    return val;
}

struct Color {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

struct ColorPoint {
    float x;
    float y;
    float z;
    Color color;
};

#endif /* CommonFunctions_h */
