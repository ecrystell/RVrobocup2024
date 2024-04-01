
#include <Arduino.h>
#include "Movement.h"
#include "Utilities.h"
#include "LED.h"
#include "ColorSensor.h"
#define buf_len 255
uint8_t idx = 0;
char buf[buf_len];
char c;
char *pch;
// #include "TCA9548A.h"
// TCA9548A I2CMux; 

#define DELAYVAL 500  // Time (in milliseconds) to pause between pixels


void setup() {
  Wire.begin();
  setupMotors(); // init motors
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.println("begin");
  pixels.begin();
  showColor(0, 0, 0, 1000);
  showColor(100, 100, 100, 100);
  showColor(0, 0, 0, 100); // blink when start

  initSensor(); // init colour sensor
  // I2CMux.begin(Wire);             // Wire instance is passed to the library
  // Serial.println("wire began");
  // I2CMux.closeAll(); 
  // Serial.println("closed all");
  
  
  delay(1000);
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.show();
  
}
bool moving = false;
int Lspeed = 0;
int Rspeed = 0;
void loop() {
  // Serial.print("A ");
  // Serial.println(countA);
  
  while (1) {
    Serial.print("A: ");
    Serial.println(countA);
    if (Serial1.available() > 0) { // change to serial1 when using thru rpi
      c = Serial1.read(); // change to serial1 when using thru rpi
      if (c == '\r') {
        continue;
      }
      if (idx < buf_len - 1) {
        buf[idx] = c;
        idx++;
      } else {
        buf[buf_len - 1] = '\0';
      }
      if (c == '\n')  // end of line
      {
        buf[idx - 1] = '\0';
        if (compareWord(buf, "M")) {  // movement M leftspeed rightspeed 
          Serial.println(buf);
          pch = getNextWord(buf, " ,\0"); // separating the buffer by spaces or comma (\0 is end char)
          int leftspeed = atoi(pch);
          pch = getNextWord(NULL, " ,\0");
          int rightspeed = atoi(pch);
          Serial.print("left speed:");
          Serial.print(leftspeed);
          Serial.print("\tright speed:");
          Serial.println(rightspeed);
          pidMotorSync(leftspeed, rightspeed);
          moving = true;
          Lspeed = leftspeed;
          Rspeed = rightspeed;

        } else if (compareWord(buf, "G")) {  // grabber for now, position
          moving = false;
          Serial.println(buf);
          pch = getNextWord(buf, " ,\0"); // separating the buffer by spaces or comma (\0 is end char)
          int pos = atoi(pch);
          
          Serial.print("pos:");
          Serial.print(pos);
          moveGrabber(pos);
          // insert servo function up or down

        } else if (compareWord(buf, "S")) {  // sorter, position
          moving = false;
          Serial.println(buf);
          pch = getNextWord(buf, " ,\0"); // separating the buffer by spaces or comma (\0 is end char)
          int pos = atoi(pch);
          
          Serial.print("pos:");
          Serial.print(pos);
          moveSorter(pos);
          // insert servo function up or down

        } else if (compareWord(buf, "R")) {  // ramp, position
          moving = false;
          Serial.println(buf);
          pch = getNextWord(buf, " ,\0"); // separating the buffer by spaces or comma (\0 is end char)
          int pos = atoi(pch);
          
          Serial.print("pos:");
          Serial.print(pos);
          moveRamp(pos);
          // insert servo function up or down

        } else if (compareWord(buf, "D")) {  // move for degrees
          moving = false;
          Serial.println(buf);
          pch = getNextWord(buf, " ,\0"); // separating the buffer by spaces or comma (\0 is end char)
          int leftspeed = atoi(pch);
          pch = getNextWord(NULL, " ,\0");
          int rightspeed = atoi(pch);
          pch = getNextWord(NULL, " ,\0");
          int degrees = atoi(pch);
          Serial.print("left speed:");
          Serial.print(leftspeed);
          Serial.print("\tright speed:");
          Serial.println(rightspeed);
          Serial.print("\tdegrees:");
          Serial.println(degrees);
          pidMotorSyncDegrees(leftspeed, rightspeed, degrees);

        } else if (compareWord(buf, "P")) { // pixel colour P r g b (0 to 255)
          moving = false;
          Serial.println(buf);
          pch = getNextWord(buf, " ,\0");
          int R = atoi(pch);
          pch = getNextWord(NULL, " ,\0");
          int G = atoi(pch);
          pch = getNextWord(NULL, " ,\0");
          int B = atoi(pch);
          Serial.print("R:");
          Serial.print(R);
          Serial.print("\tG:");
          Serial.println(G);
          Serial.print("\tB:");
          Serial.println(B);
          pixels.setPixelColor(0, pixels.Color(R, G, B));
          pixels.show();
        } else if (compareWord(buf, "CS1")) { //colour sensor, reading the sensor, not sending back to rpi
          moving = false;
          for(int i=0; i< 10; i++){
            uint16_t r, g, b, c;
            // TCA9548A(0);
            tcs1.getRawData(&r, &g, &b, &c);
            Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
            Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
            Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
            Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
            Serial.println(" ");
            delay(1);
            // to send back to rpi, string all the data tgt n send as one line
            // Serial1.println(text)
          }
        } else {
          Serial.println("Invalid Command");
        }
        idx = 0;
        

        
      }
    }
    if (moving) {
      pidMotorSync(Lspeed, Rspeed);
    }
  }
}
