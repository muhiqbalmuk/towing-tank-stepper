/* Sweep
  by BARRAGAN <http://barraganstudio.com>
  This example code is in the public domain.

  modified 8 Nov 2013
  by Scott Fitzgerald
  http://www.arduino.cc/en/Tutorial/Sweep

  Modified 7 Jul 2018
  by MI
  change delay to millis()
  
  TODO:
  1. Refactor the code
  2. Add in input to the servo and stepper method to determine the time needed to accomplish input motion
*/

#include <Servo.h>
#include <AccelStepper.h>

Servo myServo;
AccelStepper myStepper (1, 10, 11);

const int servoMinDegrees = 60;
const int servoMaxDegrees = 120;
const int servoPin = 9;
const int stepperPulsePin = 10;
const int stepperDirPin = 11;

int servoPosition = 90;
int servoInterval = 15;
int servoDegrees = 5;
char data;

unsigned long timeInit, timeFinal, timeElapsed;
unsigned long currentMillis = 0;
unsigned long previousServoMillis = 0;
unsigned long previousStepMillis = 0;
unsigned long stepperInterval = 1;

void setup() {
  //  pinMode(stepperDirPin, OUTPUT);
  //  pinMode(stepperPulsePin, OUTPUT);

  myServo.attach(servoPin);
  myServo.write(servoPosition);
  myStepper.setMaxSpeed(6000);

  Serial.begin(9600);// rate at which the arduino communicates
  Serial.println("TOWING TANK 2.0 MOTION MODULE");
}

void loop() {
  currentMillis = millis();
  rotateStepper(); // TRANSLATION MOTION
  servoSweep(); // FLAPPING MOTION
}

void rotateStepper() {
  timeInit = millis();
  if (myStepper.distanceToGo() == 0) {
    myStepper.moveTo(-8000);
    myStepper.setSpeed(6000);
  }
  myStepper.runSpeedToPosition();

  timeElapsed = millis() - timeInit;
  Serial.println(timeElapsed);
}

void servoSweep() {
  /*
    This is similar to the servo sweep example except that it uses millis() rather than delay()
    nothing happens unless the interval has expired
    the value of currentMillis was set in loop()
  */
  if (currentMillis - previousServoMillis >= servoInterval && currentMillis < 5000) {
    // its time for another move
    previousServoMillis += servoInterval;

    servoPosition = servoPosition + servoDegrees; // servoDegrees might be negative

    if ((servoPosition >= servoMaxDegrees) || (servoPosition <= servoMinDegrees))  {
      // if the servo is at either extreme change the sign of the degrees to make it move the other way
      servoDegrees = - servoDegrees; // reverse direction
      // and update the position to ensure it is within range
      servoPosition = servoPosition + servoDegrees;
    }
    // make the servo move to the next position
    myServo.write(servoPosition);

    // and record the time when the move happened
    Serial.println(previousServoMillis);
  }
  else if (currentMillis == 5000) {
    myServo.write(90);
  }
}

//=====END
