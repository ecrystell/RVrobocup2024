#ifndef MOVEMENT_H
#define MOVEMENT_H
#include <Arduino.h>

#define AIN1 27
#define AIN2 28
#define PWMA 29
#define BIN1 26
#define BIN2 15
#define PWMB 14

#define GRABBER 10
#define SORTER 11
#define SPINNER 12

void setupMotors(){
  pinMode(PWMA,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(PWMB,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(BIN2,OUTPUT);
  // copied from 2023
  // attachInterrupt(digitalPinToInterrupt(ENA), isrA, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(ENB), isrB, CHANGE);
  // grabber.attach(GRABBER);
  // sorter.attach(SORTER);
  // leftramp.attach(LEFTRAMP);
  // rightramp.attach(RIGHTRAMP);
  // end of copied
}

void move(int LSpeed, int RSpeed)
{
  if (LSpeed>0){
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA,LSpeed); 
  } else if (LSpeed<0){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA,-LSpeed); 
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA,0); 
  }
  
  if (RSpeed>0){
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB,RSpeed); 
  } else if (RSpeed<0){
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB,-RSpeed); 
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB,0); 
  }
}

// void moveDegrees(int left, int right, int deg) {
//   // reset encoder
//   while (encoder < deg) {
//     //read encoder
//     move(leftspeed, rightspeed); // may not need to do motorsync
//   }
// }
#endif