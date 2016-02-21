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

int numberOfAvailableThreads(){
    int threadCount = 0;
    # pragma omp parallel
    {threadCount++;}
    return 4;
    //return threadCount;
}

template <typename T>
inline T const& Max(T const& a, T const& b){
    return a < b ? b:a;
}

template <typename T>
inline T const& Min(T const& a, T const& b){
    return a > b ? b:a;
}

template <typename T>
inline T const& constrain (T const& val, T const& limit1, T const& limit2){
    T const& min = Min(limit1, limit2);
    T const& max = Max(limit1, limit2);
    if (val < min) return min;
    else if (val > max) return max;
    else return val;
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
