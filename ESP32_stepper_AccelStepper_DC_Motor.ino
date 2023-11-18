
// Shows how to run 1 stepper (Laser Trotec DC motor)
// Runs  stepper forward and backward, accelerating and decelerating at the limits.
//
// Copyright (C) 2014 Mike McCauley
//https://www.airspayce.com/mikem/arduino/AccelStepper/

#include <AccelStepper.h>

// The X Stepper pins
#define STEPPER1_DIR_PIN 12
#define STEPPER1_STEP_PIN 14


// Define some steppers and the pins the will use
AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);


void setup()
{
  Serial.begin(115200);
  stepper1.setMaxSpeed(5000.0);
  //stepper1.setMaxSpeed(10000.0);
  //stepper1.setMaxSpeed(20000.0);
  //stepper1.setMaxSpeed(30000.0);
  //stepper1.setMaxSpeed(40000.0);
  stepper1.setMaxSpeed(50000.0);
  //stepper1.setMaxSpeed(52000.0); //absolute max stable freq before saturation of PID output

  stepper1.setAcceleration(15000.0);
  stepper1.moveTo(200000);


  Serial.println("stepper started");
}

void loop()
{
  // Change direction at the limits
  if (stepper1.distanceToGo() == 0)
  {
    Serial.print("stepper1 change direction to ");
    delay(3000);
    Serial.println(- stepper1.currentPosition());
    stepper1.moveTo(-stepper1.currentPosition());
  }

  stepper1.run();
}
