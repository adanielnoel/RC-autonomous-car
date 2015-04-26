#include "Arduino.h"
#include "Movement.h"

void Movement::perform(bool update){
  if(update){
    // Read RC PWM
    ch1 = pulseIn(7, HIGH, 25000);
    ch2 = pulseIn(6, HIGH, 25000);
    
    // Convert PWM to steering control range
    steering = map(ch1, ch1_min, ch1_max, steering_servo_min, steering_servo_max);
    // Convert PWM to throttle control range (allow forward and backward motion)
    if(ch2 < ch2_backwards_threshold)
      throttle = map(ch2, ch2_min, ch2_backwards_threshold, backwards_min, backwards_max);
    else
      throttle = map(ch2, ch2_min, ch2_max, throttle_min, throttle_max);
  }
  // Execute movements
  steeringServo.write(steering);
  throttleServo.write(throttle);
}
