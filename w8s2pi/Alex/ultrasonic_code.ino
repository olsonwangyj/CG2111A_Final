#include "Arduino.h"
#include <stdarg.h>
#include <math.h>

//Ultrasonic sensor constants----------------------------------------------
// Define ultrasonic sensor pins
const int trigPinLeft = A8; // left ultrasonic sensor trig pin (White)
const int echoPinLeft = A9; // left ultrasonic sensor echo pin (Blue)
const int trigPinRight = A10; // right ultrasonic sensor trig pin (Yellow)
const int echoPinRight = A11; // right ultrasonic sensor echo pin (Green)

// Define variable to store distance readings
int distanceLeft = 0; // distance from left ultrasonic sensor
int distanceRight = 0; // distance from right ultrasonic sensor

int findUltra()                        //find ultrasonic reading from both sensors
{
  // Send ultrasonic sensor pulse for left sensor
  digitalWrite(trigPinLeft, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPinLeft, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinLeft, LOW);

  // Measure time for left ultrasonic sensor echo
  long durationLeft = pulseIn(echoPinLeft, HIGH);

  // Calculate distance from left ultrasonic sensor
  distanceLeft = durationLeft / 58;

  // Send ultrasonic sensor pulse for right sensor
  digitalWrite(trigPinRight, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPinRight, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinRight, LOW);

  // Measure time for right ultrasonic sensor echo
  long durationRight = pulseIn(echoPinRight, HIGH);

  // Calculate distance from right ultrasonic sensor
  distanceRight = durationRight / 58;

  // Calculate sum of distance readings
  int distanceSum = distanceLeft + distanceRight;   //adjust value accordingly based on dist between sensors
  Serial.print("distanceLeft = ");
  Serial.println(distanceLeft);
  Serial.print("distanceRight = ");
  Serial.println(distanceRight);
  Serial.print("distanceSum = ");
  Serial.println(distanceSum);
  return distanceSum;
}

void setup() {
  Serial.begin(9600);
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);
  //--------------------------------------------

}

void loop() {
  findUltra();
}