#include <Arduino.h>


#ifndef DriverSettings_H
#define DriverSettings_H

// Function declaration
String uint16ToBinaryString(uint16_t number);

String readWriteRegister(String address_);
uint16_t binaryStringToUint16(const String& binaryString) ;

#endif



