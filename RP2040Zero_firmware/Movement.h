#ifndef MOVEMENT_H
#define MOVEMENT_H
#include <Arduino.h>
#include <Servo.h>
//#include <Adafruit_TiCoServo.h>

#define AIN1 27
#define AIN2 28
#define PWMA 29
#define BIN1 26
#define BIN2 15
#define PWMB 14

#define GRABBER 10
#define SORTER 11
#define SPINNER 12


//encoders, 8 is right 9 is left, idk which one is A which is B
#define ENA 9
#define ENB 8
bool rightBackward, leftBackward;
volatile int countA, countB;
Servo grabber, sorter, spinner;
int prevLeft = 0;
int prevRight = 0;

// copied from 2023
void resetDegrees(){
  countA = 0;
  countB = 0 ;
}

void isrA() {  // i
  if (leftBackward) {
    countA--;
  } else {
    countA++;
  }
  // Serial.print("A: ");
  // Serial.println(countA);
}

// count encoder thnigy for right side
void isrB() {
  if (rightBackward) {
    countB--;
  } else {
    countB++;
  }
  // Serial.print("B: ");
  // Serial.println(countB);
}
// end copied

void setupMotors(){
  pinMode(PWMA,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(PWMB,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(BIN2,OUTPUT);

  pinMode(ENA, INPUT);
  pinMode(ENB, INPUT);
  // copied from 2023
  attachInterrupt(digitalPinToInterrupt(ENA), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENB), isrB, CHANGE);
  grabber.attach(GRABBER);
  sorter.attach(SORTER);
  spinner.attach(SPINNER);
  // end of copied
}

void moveGrabber(int pos) {
  grabber.write(pos);
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

void moveDegrees(int leftSpeed, int rightSpeed, int degrees) {
  resetDegrees();
  if (leftSpeed == 0){
    while(abs(countB) < degrees){
      move(0, rightSpeed);
    }
  } else if (rightSpeed == 0){
    while(abs(countA) < degrees){
      move(leftSpeed, 0);
    } 
  } else if (abs(leftSpeed) > abs (rightSpeed)) {    
    while(abs(countA) < degrees){
      move(leftSpeed, rightSpeed);
    }
  } else {
    while(abs(countB) < degrees){
      move(leftSpeed, rightSpeed);
    }
  }
  move(0, 0);
  resetDegrees();
  Serial.println(countA);
  
}

void pidMotorSync(int leftSpeed, int rightSpeed) {
  // rmb to reset both count when colour sensor detect green or something
  static float mpreverror = 0;
  float B = abs(countB) * (abs(leftSpeed) / abs(rightSpeed));
  #define mkp 0.5
  #define mki 0
  #define mkd 0

  float merror = B - abs(countA);
  float mP = mkp * merror;
  // float I += mki * (error);
  // float D = mkd * (error - preverror);

  // float PID = P + I + D;
  int leftdir = leftSpeed > 0 ? 1 : -1;
  int rightdir = rightSpeed > 0 ? 1 : -1;
  move((abs(leftSpeed) + mP)* leftdir, (abs(rightSpeed) - mP) * rightdir);


  mpreverror = merror;
  if (prevLeft != leftSpeed || prevRight != rightSpeed) {
    resetDegrees();
  }

  prevLeft = leftSpeed;
  prevRight = rightSpeed;

}

void pidMotorSyncDegrees(int leftSpeed, int rightSpeed, int degrees) {
  resetDegrees();
  if (leftSpeed == 0){
    while(abs(countB) < degrees){
      move(0, rightSpeed);
    }
  } else if (rightSpeed == 0){
    while(abs(countA) < degrees){
      move(leftSpeed, 0);
    } 
  } else if (abs(leftSpeed) > abs (rightSpeed)) {    
    while(abs(countA) < degrees){
      move(leftSpeed, rightSpeed);
    }
  } else {
    while(abs(countB) < degrees){
      move(leftSpeed, rightSpeed);
    }
  }
  move(0, 0);
  resetDegrees();
  Serial.println(countA);
  
}
#endif