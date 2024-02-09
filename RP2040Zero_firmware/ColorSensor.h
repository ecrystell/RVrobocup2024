#ifndef COLORSENSOR_H
#define COLORSENSOR_H

#include <Arduino.h>
#include <Wire.h>
// #include "TCA9548A.h"
#include "Adafruit_TCS34725.h"

// TCA9548A I2CMux;                  // Address can be passed into the constructor
Adafruit_TCS34725 tcs1 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);


void TCA9548A(uint8_t bus){
  // Wire.setSDA(4);
  // Wire.setSCL(5);
  Serial.println("Wire begin");
  Wire.beginTransmission(0x70);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Serial.println(Wire.endTransmission());
  Serial.print(bus);
}

void initSensor(){
  
  TCA9548A(4);
  if (tcs1.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
  }
  /* Code to interactive with revealed address on bus */
}

#endif


