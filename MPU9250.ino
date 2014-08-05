// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

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

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
uint8_t ADR = 0x68;

uint8_t MAG_ADR = 0x0c;

uint8_t buffer[14];  
int16_t ax,ay,az;
int16_t p,q,r;
int16_t temp;
int16_t mx,my,mz;

// For Arduino
#define LED_PIN 13
bool blinkState = false;

uint64_t timer;
int32_t  led_count;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 2; // 24: 400kHz I2C clock (200kHz if CPU is 8MHz) //2014.01.10変えてみた．
        //TWBR = 12; // 12;400kHz I2C clock (400kHz if CPU is 16MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);

    // initialize device
    Serial.println("Initializing I2C devices...");

    uint8_t who_am_i;
    I2Cdev::readByte(ADR, 0x75, &who_am_i);
    if(who_am_i == 0x71){
      Serial.println("Successfully connected to MPU9250");
    }
    else{
      Serial.println("Failed to Connect to MPU9250");
    }

    // PWR_MGMT_1(0x6B) -> 0x00
    // Full Scale Gyro Range  = 250 deg/s
//    Wire.beginTransmission(ADR);
//    Wire.write(0x1b);
//    Wire.write(0x00);
//    Wire.endTransmission();
    I2Cdev::writeByte(ADR, 0x1b, 0x00);
    delay(1);
    
    // GYRO_CONFIG(0x1B) -> 0x00
    // Full Scale Accelerometer Range  = 250 deg/s
//    Wire.beginTransmission(ADR);
//    Wire.write(0x1c);
//    Wire.write(0x00);
//    Wire.endTransmission();
    I2Cdev::writeByte(ADR, 0x1c, 0x00);
    delay(1);

    // ACCEL_CONFIG(0x1C) -> 0x00
    // CLKSEL = internal 20MHz oscillator
    // NON Sleep mode
//    Wire.beginTransmission(ADR);
//    Wire.write(0x6b);
//    Wire.write(0x00);
//    Wire.endTransmission();
    I2Cdev::writeByte(ADR, 0x6b, 0x00);
    delay(1);
    
    // INT_PIN_CFG(0x37) -> 0x02
    // Enable Bypass-mode (Enable to access a magnet compass:AK8963)
//    Wire.beginTransmission(ADR);
//    Wire.write(0x37);
//    Wire.write(0x02);
//    Wire.endTransmission();
    I2Cdev::writeByte(ADR, 0x37, 0x02);
    delay(5);
    
    I2Cdev::readByte(MAG_ADR,0x00,&who_am_i);
    if(who_am_i == 0x48){
      Serial.println("Successfully connected to COMPASS(AK8963)");
    }
    else{
      Serial.println("Failed to Connect to COMPASS(AK8963)");
    }
    
      // CONTROL(0x0A) -> 0x00
      // Reset Magnet Compass
//    Wire.beginTransmission(MAG_ADR);
//    Wire.write(0x0A);
//    Wire.write(0x00);
//    Wire.endTransmission();
    I2Cdev::writeByte(MAG_ADR, 0x0a, 0x00);
    delay(10);

      // CONTROL(0x0A) -> 0x16
      // BIT       = 1  -> 16 bit output
      // MODE[3:0] = 0110b -> continuous measurement mode 2(100Hz);
//    Wire.beginTransmission(MAG_ADR);
//    Wire.write(0x0A);
//    Wire.write(0x16);
//    Wire.endTransmission();
    I2Cdev::writeByte(MAG_ADR, 0x0a, 0x16);
    delay(10);
    
    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);

    timer = micros();
    
}

void loop() {

  if(micros()-timer >= 5000){
  
    // read ACCEL, TEMP and Gyro data
    I2Cdev::readBytes(ADR,0x3B,14,buffer);
    ax = (((int16_t)buffer[0]) << 8) | buffer[1];
    ay = (((int16_t)buffer[2]) << 8) | buffer[3];
    az = (((int16_t)buffer[4]) << 8) | buffer[5];
    temp = (((int16_t)buffer[6]) << 8) | buffer[7];
    p = (((int16_t)buffer[8]) << 8) | buffer[9];
    q = (((int16_t)buffer[10]) << 8) | buffer[11];
    r = (((int16_t)buffer[12]) << 8) | buffer[13];
  
    //read MAGNET_COMPASS

    //I2Cdev::writeByte(MAG_ADR, 0x0A, 0x11); // single measurement mode
    //delay(10);
  
    uint8_t mag_status2;
    I2Cdev::readBytes(MAG_ADR, 0x03, 6, buffer);
    I2Cdev::readByte(MAG_ADR, 0x09, &mag_status2);
    mx = (((int16_t)buffer[1]) << 8) | buffer[0];
    my = (((int16_t)buffer[3]) << 8) | buffer[2];
    mz = (((int16_t)buffer[5]) << 8) | buffer[4];
  
    Serial.print(ax);    Serial.print(",");
    Serial.print(ay);    Serial.print(",");
    Serial.print(az);    Serial.print(",");
    Serial.print(p);    Serial.print(",");
    Serial.print(q);    Serial.print(",");
    Serial.print(r);    Serial.print(",");
    Serial.print(mx);    Serial.print(",");
    Serial.print(my);    Serial.print(",");
    Serial.print(mz);    Serial.print(",");
    Serial.println(temp);
//  Serial.println(mag_status2,BIN);

    // blink LED to indicate activity
    if(led_count == 100){
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
      led_count = 0;
    }
    else{
      led_count++;
    }
    
  }
  
}
