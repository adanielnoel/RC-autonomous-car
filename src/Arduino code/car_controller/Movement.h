#ifndef Movement_h
#define Movement_h

#include "Arduino.h"
#include <Servo.h>

class Movement{
  private:
 
    int ch1;
    static const int ch1_max = 1822;
    static const int ch1_min = 1079;
    static const int steering_servo_min = 70;
    static const int steering_servo_max = 170;
    int ch1_last;
    int ch1_last_diff;

    int ch2;
    static const int ch2_max = 1770;
    static const int ch2_min = 800;
    static const int throttle_max = 150;
    static const int throttle_min = 80;
    static const int ch2_backwards_threshold = 1200;
    static const int backwards_min = 40;
    static const int backwards_max = 60;
    
  public:
    Servo steeringServo;
    Servo throttleServo;
    int steering;          //in meters (0 means straight)
    unsigned long duration;  //in milliseconds
    int throttle;               //in %
    void perform(bool update);
};

#endif
