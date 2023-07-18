#include <Arduino.h>


#ifndef MagneticEncoder_H
#define MagneticEncoder_H

extern float targetPosition;
extern float previousPosition;

extern String recevedCommand;
// extern float targetPosition  ;    // Target position in degrees
extern float targetVelocity ;     // Target velocity in degrees per second
extern float currentPosition ;   // Current position in degrees
extern float currentVelocity ;
extern const int csPin;
// Function declaration
// float readAngle();

float readAngle1(const int csPin); 

void updateVelocity() ;

void parseSerialInput(String input) ;
uint16_t readAngle2(const int csPin=5) ;

#endif
