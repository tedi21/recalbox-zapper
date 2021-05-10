/* ============================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2012 Jeff Rowberg

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/
#include <SoftwareSerial.h>
#include <EEPROM.h>

#include "I2Cdev.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include "RTIMUSettings.h"
#include "RTIMUBNO055.h"
#include "CalLib.h"

#if !defined(BNO055_28) && !defined(BNO055_29)
#error "One of BNO055_28 or BNO055_29 must be selected in RTIMULibdefs.h"
#endif

RTIMUBNO055 *imu;                                     // the IMU object
RTIMUSettings settings;                               // the settings object
CALLIB_DATA calData;                                  // the calibration data

const int pinButton = 3;
const int TRIGGER_DELAY = 200;
unsigned long triggerTime = 0;
volatile int triggerInterrupt = 0;
void triggerReady() {
  // wait 200 ms between two shots
  if (triggerInterrupt == 0) {
    triggerInterrupt = 1;
  }
}

#define BLE_JDY_RX 4
#define BLE_JDY_TX 5
SoftwareSerial BLE_JDY_16(BLE_JDY_TX, BLE_JDY_RX);

const int INIT_SEQUENCE = 1;
const int STAB_SEQUENCE = 2;
const int CALIBRATION_SEQUENCE = 3;
const int GAME_SEQUENCE = 4;
int phase = 0;
String buffer = "";
const int ALIVE_DELAY = 50;
unsigned long aliveTime = 0;

void calib()
{   
  RTVector3 mag;
  if (imu->IMURead()) {                                 
    // get the latest data
    mag = imu->getCompass();
    for (int i = 0; i < 3; i++) {
      if (mag.data(i) < calData.magMin[i]) {
        calData.magMin[i] = mag.data(i);
      }
      if (mag.data(i) > calData.magMax[i]) {
        calData.magMax[i] = mag.data(i);
      }
    }
  }
}

void setup() {
  int errcode;

  Serial.begin(9600);
  BLE_JDY_16.begin(9600);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
      delay(300);
  #endif

  // on met le bouton en entrée
  pinMode(pinButton, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinButton), triggerReady, FALLING);

  calLibRead(0, &calData);                           // pick up existing mag data if there   
  calData.magValid = false;
  for (int i = 0; i < 3; i++) {
    calData.magMin[i] = 10000000;                    // init mag cal data
    calData.magMax[i] = -10000000;
  }
  imu = (RTIMUBNO055 *)RTIMU::createIMU(&settings);                        // create the imu object
  Serial.print("ArduinoIMU starting using device "); Serial.println(imu->IMUName());
  if ((errcode = imu->IMUInit()) < 0) {
      Serial.print("Failed to init IMU: "); Serial.println(errcode);
  }    
  aliveTime = millis();
  triggerTime = aliveTime;
  phase = 0;
  
  ///
  // phase = CALIBRATION_SEQUENCE;
}

unsigned long aliveDelay() {
    return millis() + 2 * ALIVE_DELAY;
}

void loop() {  
  double yaw = 0.0;
  double pitch = 0.0;
  double roll = 0.0;
  String receive = "";
      
  while (BLE_JDY_16.available()) {
    receive += (char)BLE_JDY_16.read();
  }
  
  if (receive != "") {
    Serial.println("Debug " + receive + ".");
    if (phase == 0) {
      buffer += receive;
      int n = buffer.length() - 12;
      if (n > 0) buffer.remove(0, n);
      if (buffer == "+CONNECTED\r\n") {
        delay(2000);
        Serial.println("Start sequence.");
        BLE_JDY_16.print("A;");
        BLE_JDY_16.flush();
        aliveTime = aliveDelay();
        phase = INIT_SEQUENCE;
      }
    }
    else {
      if (receive == "Z") {
        triggerInterrupt = 0;
        imu->setCalibrationMode(true);
        Serial.print("ArduinoIMU calibrating device "); 
        Serial.println(imu->IMUName());
        phase = STAB_SEQUENCE;
      }
      else if (receive == "Y") {
        phase = CALIBRATION_SEQUENCE;
      }
      else if (receive == "X") {
        phase = GAME_SEQUENCE;
      }
    }
  }

  if (phase > INIT_SEQUENCE) {
    if (phase == STAB_SEQUENCE) {
      calib();
      if (triggerInterrupt == 1) {
        calData.magValid = true;
        calLibWrite(0, &calData);
        BLE_JDY_16.print("B;");
        BLE_JDY_16.flush();
        Serial.println("B;");
        triggerInterrupt = TRIGGER_DELAY;
        triggerTime = millis();
        aliveTime = aliveDelay();
      }
    }
    else {
  
      // get the latest data if ready yet
      while (imu->IMURead());                               
          
      const RTVector3& vec = imu->getFusionPose();
      yaw = vec.z() * RTMATH_RAD_TO_DEGREE;
      roll = vec.y() * RTMATH_RAD_TO_DEGREE;
      pitch = vec.x() * RTMATH_RAD_TO_DEGREE;
  
      if (triggerInterrupt == 1) {
        if (phase == CALIBRATION_SEQUENCE) {
          BLE_JDY_16.print("C " + String(yaw, 2) + " " + String(pitch, 2) + " " + String(roll, 2) + ";");
          BLE_JDY_16.flush();
          Serial.println("C " + String(yaw, 2) + " " + String(pitch, 2) + " " + String(roll, 2) + ";");
        }
        else {
          BLE_JDY_16.print("D " + String(yaw, 2) + " " + String(pitch, 2) + " " + String(roll, 2) + ";");
          BLE_JDY_16.flush();
          Serial.println("D " + String(yaw, 2) + " " + String(pitch, 2) + " " + String(roll, 2) + ";");
        }
        triggerInterrupt = TRIGGER_DELAY;
        triggerTime = millis();
        aliveTime = aliveDelay();
      }
    }
    unsigned long now = millis();
    if ((aliveTime < now) && ((now - aliveTime) >= ALIVE_DELAY)) {
      BLE_JDY_16.print("E " + String(yaw, 2) + " " + String(pitch, 2) + " " + String(roll, 2) + ";");
      BLE_JDY_16.flush();
      Serial.println("E " + String(yaw, 2) + " " + String(pitch, 2) + " " + String(roll, 2) + ";");
      aliveTime = now;
    }
    if ((triggerInterrupt == TRIGGER_DELAY) && ((now - triggerTime) >= TRIGGER_DELAY)) {
      triggerInterrupt = 0;
    }
  }
}
