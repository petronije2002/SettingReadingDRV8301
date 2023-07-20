#include <Arduino.h>
#include "DriverSettings.h"
#include "MagneticEncoder.h"
#include <SPI.h>
// #include <Serial.h>
#include <PID_v1.h>
#include <SimpleFOC.h>







double setpoint = 100.0; // Target position
double input = 0.0;      // Current position
double output = 0.0;     // Control output

// Define the PID object

#define INH_A_PIN 12
#define INL_A_PIN 14

#define INH_B_PIN 27
#define INL_B_PIN 26

#define INH_C_PIN 25
#define INL_C_PIN 33







#define PHASE_DELAY_1 (double)2.094395102 // 120°
#define PHASE_DELAY_2 (double)4.188790205 // 240°

const float MAX_POSITION = 359.9 * PI / 180;
const float MIN_POSITION = 0.0;

float direction = 0;

float positionIncrement = 0.001;

float actingAngle = 0;

float actingAngleRad = 0;

int POLE_PAIRS = 21;
// #define PI 3.14159265359

float PHASE_SHIFT = 1;

float OFFSET = -0.4;

float smallMovement = 0;

unsigned long currentTime;
unsigned long elapsedTime;
double integralSum = 0.0;
const double integralWindupMax = 3.0; // Maximum value for integral sum (adjust as needed)

float dutyCycle = 0;
float mappedControl;

const int EN_GATE = 15;
const int csPin = 5; // pin to select magnetic encoder....

// Motor control variables
float targetPosition = 100.0; // Target position in degrees
float targetVelocity = 2000;  // Target velocity in degrees per second
float currentPosition = 0.0;  // Current position in degrees
float currentVelocity = 0.0;  // Current velocity in degrees per second
float positionErrorSum = 0.0; // Position error integral sum
float velocityErrorSum = 0.0; // Velocity error integral sum

int middlePWM = 512; // in the case of 10 bit PWM

// Velocity integral gain

float inH_A = 0;
float inL_A = 0;

float inH_B = 0;
float inL_B = 0;

float inH_C = 0;
float inL_C = 0;

// int direction = 1; // define starting direction

String recevedCommand = "";

volatile bool shouldReadAngle = true;

float angleDiff;
float angleDiffRad;

const unsigned long sampleTime = 10; // Sample time in milliseconds

double Kp = 3.5;  // Proportional gain
double Ki = 0.25; // Integral gain
double Kd = 0.0;  // Derivative gain

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

float mapFloat(float value, float inputMin, float inputMax, float outputMin, float outputMax)
{
  // Ensure the input value is within the specified range
  value = constrain(value, inputMin, inputMax);

  // Calculate the input and output range spans
  float inputSpan = inputMax - inputMin;
  float outputSpan = outputMax - outputMin;

  // Convert the input value to a relative position within the input range
  float valueScaled = (value - inputMin) / inputSpan;

  // Map the relative input position to the output range
  float mappedValue = outputMin + (valueScaled * outputSpan);

  return mappedValue;
}

void controlMotor();

float calculateShortestAngleDifference(double currentPos, double targetPos)
{
  float clockwiseDiff = fmod((targetPos - currentPos + MAX_POSITION), MAX_POSITION);
  float counterclockwiseDiff = fmod((currentPos - targetPos + MAX_POSITION), MAX_POSITION);
  return (clockwiseDiff <= counterclockwiseDiff) ? clockwiseDiff : -counterclockwiseDiff;
}

// Smooth transition function
float smoothTransition(double &currentPosition, double targetPosition)
{
  angleDiff = calculateShortestAngleDifference(currentPosition, targetPosition);
  angleDiffRad = angleDiff * PI / 180;
  direction = (angleDiff >= 0) ? 1.0 : -1.0;
  return angleDiff;
}

hw_timer_t *timer = NULL;

void IRAM_ATTR onTimer()
{
  shouldReadAngle = true;
}

 

void setup()
{


 
  // BLDCMotor motor = BLDCMotor(21);





  ledcSetup(0, 20000, 10); // Channel 0, 10 kHz frequency, 10-bit resolution
  ledcSetup(1, 20000, 10); // Channel 1, 10 kHz frequency, 10-bit resolution
  ledcSetup(2, 20000, 10); // Channel 2, 10 kHz frequency, 10-bit resolution
  ledcSetup(3, 20000, 10); // Channel 3, 10 kHz frequency, 10-bit resolution
  ledcSetup(4, 20000, 10); // Channel 4, 10 kHz frequency, 10-bit resolution
  ledcSetup(5, 20000, 10); // Channel 5, 10 kHz frequency, 10-bit resolution

  // Attach the PWM channels to the motor phase pins

  ledcAttachPin(INH_A_PIN, 0); // U phase pin, Channel 0
  ledcAttachPin(INL_A_PIN, 1); // V phase pin, Channel 1
  ledcAttachPin(INH_B_PIN, 2); // W phase pin, Channel 2
  ledcAttachPin(INL_B_PIN, 3); // X phase pin, Channel 3
  ledcAttachPin(INH_C_PIN, 4); // Y phase pin, Channel 4
  ledcAttachPin(INL_C_PIN, 5); // Z phase pin, Channel 5

  Serial.begin(115200);
  SPI.begin();

  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, HIGH); // Deselect AS5048A initially

  pinMode(EN_GATE, OUTPUT);
  digitalWrite(EN_GATE, HIGH);

  timer = timerBegin(0, 80, true);             // Timer 0, prescaler 80, count up
  timerAttachInterrupt(timer, &onTimer, true); // Attach the interrupt handler

  // Compute the necessary values for a 100 Hz interrupt
  float interruptFrequency = 80.0;          // Desired interrupt frequency in Hz
  float timerFrequency = 40000000.0 / 80.0; // Timer frequency with prescaler
  float alarmValue = timerFrequency / interruptFrequency;

  // Set the interrupt interval to achieve the desired frequency
  timerAlarmWrite(timer, static_cast<uint32_t>(alarmValue), true);
  timerAlarmEnable(timer);

  // currentPosition = smoothTransition(currentPosition, targetPosition);

  
}

unsigned long previousTime = 0;

float previousPosition = 0;

float offsetStep = 0.4;

float kp_step = 0.1;
float ki_step = 0.1;

float currentPositionRad = 0;

void loop()
{



  

  // Wrap the position within the valid range [0, MAX_POSITION]
 

  if (Serial.available())
  {
    String input_ = Serial.readStringUntil('\n');
    // Serial.println("This is received from GUI: ");
    // Serial.println(input);
    parseSerialInput(input_);

    if (recevedCommand != "")
    {
      Serial.println("Response from DRV8301: ");
      Serial.println(readWriteRegister(recevedCommand));
    }

    // Process the received command
    if (input_ == "K")
    {

      OFFSET -= 0.1;

      // dtostrf(OFFSET, 6, 3, buffer); // 6 total characters, including 3 decimal places
      Serial.println("OFFSET: ");
      Serial.print(OFFSET);
    }
    else if (input_ == "L")
    {

      OFFSET += 0.1;
      Serial.println("OFFFSET: ");
      Serial.print(OFFSET);
    }

    if (input_ == "O")
    {

      Serial.println("KI ");
      Serial.print(Ki);
    }
    else if (input_ == "P")
    {

      Serial.println("KI ");
      Serial.print(Ki);
    }
  }

  // unsigned long currentTime = millis();
  // unsigned long elapsedTime = currentTime - previousTime;

  // // if (elapsedTime >= sampleTime)
  // // {

  input = (double)readAngle1(csPin);

  double currentPositionRad = input * PI / 180;

   if (currentPositionRad >= MAX_POSITION)
  {
    currentPositionRad -= MAX_POSITION;
  }


  Serial.println(currentPositionRad);


  currentPositionRad += 0.1; // JUST INCREMENT IT A BIT]
  Serial.println(currentPositionRad);

  // // setpoint = (double)targetPosition * PI / 180;

  // // float error = smoothTransition(currentPositionRad, setpoint);

  // // integralSum += error * elapsedTime / 1000.0; // /1000 to get per second

  // // integralSum = constrain(integralSum, -integralWindupMax, integralWindupMax);

  // // output = Kp * error + Ki * integralSum;

  // // float constrainedOutput = mapFloat(output, -10, 10, -0.6, 0.6);

  // // Serial.println("Error:");
  // // Serial.println(error);

  float U = middlePWM + middlePWM * 0.35 * sin(currentPositionRad + OFFSET);
  float V = middlePWM + middlePWM * 0.35 * sin(currentPositionRad + 2.094395102 + OFFSET); // Phase shift of 120 degrees
  float W = middlePWM + middlePWM * 0.35 * sin(currentPositionRad + 4.188790205 + OFFSET); // Phase shift of 240 degrees
  // Serial.print(U, 2);
  // Serial.print(",");
  // Serial.print(V, 2);
  // Serial.print(",");
  // Serial.print(W, 2);
  // Serial.println();

 int middlePWM = 512; // Half of the PWM resolution (e.g., for 10-bit PWM: 1024 / 2 = 512)

// Generate complementary PWM signals for Phase A
 if (U < middlePWM)
 {
    inL_A = int(U);
    inH_A = 1023 - inL_A;
 }
 else
 {
    inH_A = int(U);
    inL_A = 1023 - inH_A;
 }

 // Generate complementary PWM signals for Phase B
 if (V < middlePWM)
 {
    inL_B = int(V);
    inH_B = 1023 - inL_B;
 }
 else
 {
    inH_B = int(V);
    inL_B = 1023 - inH_B;
 }

 // Generate complementary PWM signals for Phase C
 if (W < middlePWM)
 {
    inL_C = int(W);
    inH_C = 1023 - inL_C;
 }
 else
 {
    inH_C = int(W);
    inL_C = 1023 - inH_C;
 }

  ledcWrite(0, int(inH_A)); // Write PWM value to Channel 0
  ledcWrite(1, int(inL_A)); // Write PWM value to Channel 1

  ledcWrite(2, int(inH_B)); // Write PWM value to Channel 2
  ledcWrite(3, int(inL_B)); // Write PWM value to Channel 3

  ledcWrite(4, int(inH_C)); // Write PWM value to Channel 4
  ledcWrite(5, int(inL_C)); // Write PWM value to Channel 5

 

  delay(50);
}
