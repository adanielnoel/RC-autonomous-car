#include <Servo.h>
#include "Movement.h"
 
const int MISSION_MODE        = 0;
const int RECONNAISSANCE_MODE = 1;
const int MANUAL_MODE         = 2;
const int REPEAT_MODE         = 3;

Servo steeringServo;
Servo throttleServo;

// If the PWM difference between refreshes
// is greather than this, the control is 
// given to the RC system.
const int ch1_diff_threshold = 100;

// While in manual mode, if the PWM diff is
// lower that the threshold during more than
// this value, the control is given to the
// on-board computer.

// Variables for MISSION_MODE
const long user_control_timeout = 3000;
long last_user_control_time = 0;

// Variables for RECONNAISSANCE_MODE
const long save_state_interval = 100; //Time interval for saving steering and speed (dead reckoning)
long last_state_save = 0;
const int movementHistoryLength = 90;
Movement movementQueue[movementHistoryLength];
int movementQueueIterator = 0;

boolean userControl = true; // If true, the car obbeys the RC commands
const bool UPDATE_STATE = true;
int driveMode = RECONNAISSANCE_MODE;

void setup(){
  steeringServo.attach(9); //Steering servo
  throttleServo.attach(8); //Motor ESC
  Serial.begin(9600);
  pinMode(7, INPUT); //RC steering chanel
  pinMode(6, INPUT); //RC throttle chanel
  last_user_control_time = millis();
}

void loop(){
/*  int diff = abs(ch1-ch1_last);
  if(diff > ch1_diff_threshold){
    userControl = true;
    last_user_control_time = millis();
  }
  */
  
  if(driveMode == REPEAT_MODE){
    userControl = false;
    if(millis() - last_state_save > save_state_interval){
      movementQueueIterator ++;
      last_state_save = millis();
      Serial.println("REPEAT_MODE");
      Serial.println(movementQueueIterator);
      if(movementQueueIterator >= movementHistoryLength){
        movementQueueIterator = 0;
        userControl = true;
        driveMode = RECONNAISSANCE_MODE;
      }
    }
    Movement(movementQueue[movementQueueIterator]).perform(!UPDATE_STATE);
  }
  
  if(userControl){
        //This will be executed when the user has control
    Movement newMove;
    newMove.steeringServo = steeringServo;
    newMove.throttleServo = throttleServo;
    newMove.perform(UPDATE_STATE);
    
    if(driveMode == MISSION_MODE && millis() - last_user_control_time > user_control_timeout){
      userControl = false;
    }
    else if(driveMode == RECONNAISSANCE_MODE && millis() - last_state_save > save_state_interval){
      movementQueue[movementQueueIterator] = newMove;
      movementQueueIterator++;
      last_state_save = millis();
      if(movementQueueIterator >= movementHistoryLength){
        movementQueueIterator = 0;
        driveMode = REPEAT_MODE;
      }
      Serial.println("RECONAISSANCE_MODE");
      Serial.println(movementQueueIterator);
    }
  }
  delay(100);
}


