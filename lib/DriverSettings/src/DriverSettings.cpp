
#include "DriverSettings.h"
#include <SPI.h>



// const int chipSelectPin = 5;

extern const int chipSelectPin = 5;

String readWriteRegister(String address_){




  SPI.begin();

  Serial.println();

  Serial.println("");


  // uint16_t number_ = strtoul(address_.c_str(), nullptr, 2);


  uint16_t number_ = binaryStringToUint16(address_);

  Serial.println("This is the number: ");
  Serial.println(number_);


  // Set SPI communication
  SPI.beginTransaction(SPISettings(3000000, MSBFIRST, SPI_MODE1));

  digitalWrite(chipSelectPin, LOW);

  // Send the 16-bit word
  SPI.transfer16(number_);
  digitalWrite(chipSelectPin, HIGH);

  delay(10);

  digitalWrite(chipSelectPin, LOW);


  uint16_t receivedWord = SPI.transfer16(0x0000); // Send dummy data to receive actual data

  // Serial.println("Receved number from SPI");
  // Serial.println(receivedWord);


  digitalWrite(chipSelectPin, HIGH);

  SPI.endTransaction();




  String binaryStringRepresentation = uint16ToBinaryString(receivedWord);

  //  binaryStringRepresentation = String(16 - binaryStringRepresentation.length(), '0') + binaryStringRepresentation;

  Serial.println("binaryStringRepresentation: ");
  Serial.println(binaryStringRepresentation);

  Serial.println("This is the number: ");
  // Serial.printf(number_);
  Serial.println("Receved number from SPI");
  Serial.println(binaryStringRepresentation);


  // String reversedBinaryString = "";
  // for (int i = binaryStringRepresentation.length() - 1; i >= 0; --i) {
  //   reversedBinaryString += binaryStringRepresentation.charAt(i);
  // }

  
  
  return binaryStringRepresentation;
    

}

// String uint16ToBinaryString(uint16_t number) {
//   String binaryString = "";
//   for (int i = 15; i >= 0; --i) {
//     binaryString += (number >> i) & 1 ? '1' : '0';
//   }
//   return binaryString;
// }

String uint16ToBinaryString(uint16_t number) {
  String binaryString = "";
  for (int i = 15; i >= 0; --i) {
    binaryString += (number & (1 << i)) ? '1' : '0';
  }
  binaryString = String(16 - binaryString.length(), '0') + binaryString;
  return binaryString;
}


uint16_t binaryStringToUint16(const String& binaryString) {
  uint16_t number = 0;
  for (size_t i = 0; i < binaryString.length(); ++i) {
    number <<= 1;  // Left shift the number by 1 bit
    number |= (binaryString.charAt(i) == '1') ? 1 : 0;  // Set the LSB based on the current bit
  }
  return number;
}


