#include <Arduino.h>
#include <PID_v1.h>

#define TIME_DELAY (5)
#define OUTPUT_DELAY (TIME_DELAY * 10)

// Arduino code to test motors for straight movement

/*----- Define pin(s) -----*/
// Left motor (motorA)
const int motorA1 = 8;
const int motorA2 = 9;
const int motorAPWM = 5;

// Right motor (motorB)
const int motorB1 = 10;
const int motorB2 = 11;
const int motorBPWM = 6;

void setup() {
  // Set motor pins as output
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorAPWM, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(motorBPWM, OUTPUT);

  // Initialize serial communication
  Serial.begin(9600);

  // Set direction for both motors to move forward
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
}

void loop() {
  // Set PWM speed for both motors (0-255)
  int motorSpeed = 150;

  analogWrite(motorAPWM, motorSpeed); // Left motor
  analogWrite(motorBPWM, motorSpeed); // Right motor

  // Print motor status to Serial Monitor
  Serial.println("Motors running forward...");

  // Let the robot run straight for 5 seconds
  delay(5000);

  // Stop both motors
  analogWrite(motorAPWM, 0);
  analogWrite(motorBPWM, 0);

  Serial.println("Motors stopped.");

  // Wait for 2 seconds before running again
  delay(2000);
}

