#include <Arduino.h>
#include "DriverSettings.h"
#include "MagneticEncoder.h"
#include <SPI.h>
// #include <Serial.h>
#include <PID_v1.h>

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

const float MAX_POSITION = 359.9;
const float MIN_POSITION = 0.0;

float direction = 0;

float positionIncrement = 0.0001;

float actingAngle = 0;

float actingAngleRad = 0;

int POLE_PAIRS = 21;
// #define PI 3.14159265359

float PHASE_SHIFT = 1;

float OFFSET = 0.00;

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

int middlePWM = 127; // in the case of 8 bit PWM

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

const unsigned long sampleTime = 10;  // Sample time in milliseconds


double Kp = 3.5;   // Proportional gain
double Ki = 0.25; // Integral gain
double Kd = 0.0; // Derivative gain

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
  ledcSetup(0, 20000, 8); // Channel 0, 10 kHz frequency, 8-bit resolution
  ledcSetup(1, 20000, 8); // Channel 1, 10 kHz frequency, 8-bit resolution
  ledcSetup(2, 20000, 8); // Channel 2, 10 kHz frequency, 8-bit resolution
  ledcSetup(3, 20000, 8); // Channel 3, 10 kHz frequency, 8-bit resolution
  ledcSetup(4, 20000, 8); // Channel 4, 10 kHz frequency, 8-bit resolution
  ledcSetup(5, 20000, 8); // Channel 5, 10 kHz frequency, 8-bit resolution

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
  float interruptFrequency = 20.0;          // Desired interrupt frequency in Hz
  float timerFrequency = 40000000.0 / 80.0; // Timer frequency with prescaler
  float alarmValue = timerFrequency / interruptFrequency;

  // Set the interrupt interval to achieve the desired frequency
  timerAlarmWrite(timer, static_cast<uint32_t>(alarmValue), true);
  timerAlarmEnable(timer);

  // currentPosition = smoothTransition(currentPosition, targetPosition);

  pid.SetMode(AUTOMATIC);       // Set the PID mode to AUTOMATIC
  pid.SetOutputLimits(-1023, 1023); // Set the output limits of the PID controller
  pid.SetSampleTime(10);        // Set the sample time (in milliseconds)
}

unsigned long previousTime = 0;

float previousPosition = 0;

float offsetStep = 0.4;

float kp_step = 0.1;
float ki_step = 0.1;

void loop()
{

  // if (Serial.available())
  // {
  //   String input_ = Serial.readStringUntil('\n');
  //   // Serial.println("This is received from GUI: ");
  //   // Serial.println(input);
  //   parseSerialInput(input_);

  //   if (recevedCommand != "")
  //   {
  //     Serial.println("Response from DRV8301: ");
  //     Serial.println(readWriteRegister(recevedCommand));
  //   }

  //   // Process the received command
  //   if (input_ == "K")
  //   {

  //     Kp -= kp_step;

  //     // dtostrf(OFFSET, 6, 3, buffer); // 6 total characters, including 3 decimal places
  //     Serial.println("KP: ");
  //     Serial.print(Kp);
  //   }
  //   else if (input_ == "L")
  //   {

  //     Kp += kp_step;
  //     Serial.println("KP: ");
  //     Serial.print(Kp);
  //   }

  //   if (input_ == "O")
  //   {

  //     Serial.println("KI ");
  //     Serial.print(Ki);
  //   }
  //   else if (input_ == "P")
  //   {

  //     Serial.println("KI ");
  //     Serial.print(Ki);
  //   }
  // }


  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - previousTime;


  if (elapsedTime >= sampleTime) {

  input = (double)readAngle1(csPin);



  double currentPositionRad = input * PI / 180;

  setpoint = (double)targetPosition * PI/180;

  float error = smoothTransition(currentPositionRad, setpoint);

  integralSum += error * elapsedTime/1000.0;


  
  integralSum = constrain(integralSum, -integralWindupMax, integralWindupMax);


  output = Kp * error + Ki * integralSum;

  // input = (double)error;

  // setpoint = 0.0;

  // pid.Compute();



  // float pidOutput = Kp*error ;
  float constrainedOutput = mapFloat(output, -10, 10, -1.0, 1.0);

  Serial.println("Error:");
  Serial.println(error);

  Serial.println("Setpoint:");
  Serial.println(setpoint);

  Serial.println(" Output:");
  Serial.println(output);

  Serial.println("Constrained Output:");
  Serial.println(constrainedOutput);
  delay(20);

  


  //   if (shouldReadAngle)
  //   {

  //     currentTime = micros();

  //     currentPosition = readAngle1(csPin);

  //     // Serial.println("currentPosition");
  //     // Serial.println(currentPosition);

  //     float currentPositionSmooth = smoothTransition(currentPosition, targetPosition);

  //     Serial.println("currentPosition ");
  //     Serial.println(currentPositionSmooth);

  //     positionErrorSum += angleDiffRad ;

  //     if (positionErrorSum >= 100){

  //       positionErrorSum = 100;
  //     }
  //     else if ( positionErrorSum <-100){

  //       positionErrorSum = -100;

  //     }

  //     float positionControl1 = -Kp * angleDiffRad - Ki * positionErrorSum ;

  //      Serial.println("Control");
  //     Serial.println(positionControl1,6);

  //     float constrainedControl = constrain(positionControl1, -100, 100);

  //     mappedControl = mapFloat(constrainedControl, -100, 100, -0.4, 0.4);

  //     // Serial.println("Mapped control");
  //     // Serial.println(mappedControl);

  //     // Serial.println("Target angle");
  //     // Serial.println(targetPosition);

  //     controlMotor();
  //     shouldReadAngle = false;
  //   }
  // }

  // void controlMotor()
  // {

  //   dutyCycle = abs(mappedControl);

  //   float currentPositionRad = currentPosition * PI / 180.0; // Convert to radians

  //   // Serial.println("DutyCycle");

  //   // Serial.println(dutyCycle);

  float U = middlePWM + middlePWM * output * sin(currentPositionRad * POLE_PAIRS);

  float V = middlePWM + middlePWM * output * sin(currentPositionRad * POLE_PAIRS + PHASE_DELAY_1);

  float W = middlePWM + middlePWM * output * sin(currentPositionRad * POLE_PAIRS + PHASE_DELAY_2);

  if (U < middlePWM)
  {
    inL_A = int(U);
    inH_A = 0;
  }

  if (U >= middlePWM)
  {
    inL_A = 0;
    inH_A = int(U);
  }

  if (V < middlePWM)
  {
    inL_B = int(V);
    inH_B = 0;
  }

  if (V >= middlePWM)
  {
    inL_B = 0;
    inH_B = int(V);
  }

  if (W < middlePWM)
  {
    inL_C = int(W);
    inH_C = 0;
  }

  if (W >= middlePWM)
  {
    inL_C = 0;
    inH_C = int(W);
  }

  ledcWrite(0, int(inH_A)); // Write PWM value to Channel 0
  ledcWrite(1, int(inL_A)); // Write PWM value to Channel 1

  ledcWrite(2, int(inH_B)); // Write PWM value to Channel 2
  ledcWrite(3, int(inL_B)); // Write PWM value to Channel 3

  ledcWrite(4, int(inH_C)); // Write PWM value to Channel 4
  ledcWrite(5, int(inL_C)); // Write PWM value to Channel 5

  previousTime = currentTime;

  }
};
