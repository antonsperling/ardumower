/*
  Ardumower (www.ardumower.de)
  Copyright (c) 2013-2014 by Alexander Grau
  Copyright (c) 2013-2014 by Sven Gennat
  
  Private-use only! (you need to ask for a commercial-use)
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  
  Private-use only! (you need to ask for a commercial-use)
*/

#include "imu.h"
#include <Arduino.h>
#include <Wire.h>
#include "drivers.h"
#include "i2c.h"
#include "config.h"
#include "flashmem.h"
#include "buzzer.h"
#include <quaternionFilters.h>

// -------------I2C addresses ------------------------
#define ADXL345B (0x53)          // ADXL345B acceleration sensor (GY-80 PCB)
#define HMC5883L (0x1E)          // HMC5883L compass sensor (GY-80 PCB)
#define L3G4200D (0xD2 >> 1)     // L3G4200D gyro sensor (GY-80 PCB)


#define ADDR 600
#define MAGIC 6


struct {
  uint8_t xl;
  uint8_t xh;
  uint8_t yl;
  uint8_t yh;
  uint8_t zl;
  uint8_t zh;
} gyroFifo[32];



IMU::IMU(){
  hardwareInitialized = false;
  calibrationAvail = false;
  state = IMU_RUN;
  callCounter = 0;  
  errorCounter = 0;
  temperature = 0;
  tempCount = 0;
  
  gyroOfs.x=gyroOfs.y=gyroOfs.z=0;  
  gyroNoise = 0;      
  gyroCounter = 0; 
  useGyroCalibration = false;
  lastGyroTime = millis();
  lastUpdateMicros = 0;
  
  accelCounter = 0;
  calibAccAxisCounter = 0;
  useAccCalibration = true; 
  accPitch = 0;
  accRoll = 0;
  ypr.yaw=ypr.pitch=ypr.roll = 0;
  
  accMin.x=accMin.y=accMin.z = 0;
  accMax.x=accMax.y=accMax.z = 0;  
  accOfs.x=accOfs.y=accOfs.z = 0;
  accScale.x=accScale.y=accScale.z = 2;  
  com.x=com.y=com.z=0;  
  
  comScale.x=comScale.y=comScale.z=2;  
  comOfs.x=comOfs.y=comOfs.z=0;    
  useComCalibration = true;
  imuUseMPU                  = 1;          // use MPU9250 instead of GY-80?

}

// rescale to -PI..+PI
float IMU::scalePI(float v)
{
  float d = v;
  while (d < 0) d+=2*PI;
  while (d >= 2*PI) d-=2*PI;
  if (d >= PI) return (-2*PI+d); 
  else if (d < -PI) return (2*PI+d);
  else return d;  
}

// rescale to -180..+180
float IMU::scale180(float v)
{
  float d = v;
  while (d < 0) d+=2*180;
  while (d >= 2*180) d-=2*180;
  if (d >= 180) return (-2*180+d); 
  else if (d < -180) return (2*180+d);
  else return d;  
}


// computes minimum distance between x radiant (current-value) and w radiant (set-value)
float IMU::distancePI(float x, float w)
{
  // cases:   
  // w=330 degree, x=350 degree => -20 degree
  // w=350 degree, x=10  degree => -20 degree
  // w=10  degree, x=350 degree =>  20 degree
  // w=0   degree, x=190 degree => 170 degree
  // w=190 degree, x=0   degree => -170 degree 
  float d = scalePI(w - x);
  if (d < -PI) d = d + 2*PI;
  else if (d > PI) d = d - 2*PI;  
  return d;
}

float IMU::distance180(float x, float w)
{
  float d = scale180(w - x);
  if (d < -180) d = d + 2*180;
  else if (d > 180) d = d - 2*180;  
  return d;
}


// weight fusion (w=0..1) of two radiant values (a,b)
float IMU::fusionPI(float w, float a, float b)
{ 
  float c;
  if ((b >= PI/2) && (a <= -PI/2)){
    c = w * a + (1.0-w) * (b-2*PI);
  } else if ((b <= -PI/2) && (a >= PI/2)){
    c = w * (a-2*PI) + (1.0-w) * b;
  } else c = w * a + (1.0-w) * b;
  return scalePI(c);
}

void IMU::loadSaveCalib(boolean readflag){
  int addr = ADDR;
  short magic = MAGIC;
  eereadwrite(readflag, addr, magic); // magic
  eereadwrite(readflag, addr, accOfs);
  eereadwrite(readflag, addr, accScale);    
  eereadwrite(readflag, addr, comOfs);
  eereadwrite(readflag, addr, comScale);      
}

void IMU::loadCalib(){
  short magic = 0;
  int addr = ADDR;
  eeread(addr, magic);
  if (magic != MAGIC) {
    Console.println(F("IMU error: no calib data"));
    return;  
  }
  calibrationAvail = true;
  Console.println(F("IMU: found calib data"));
  loadSaveCalib(true);
}

void IMU::saveCalib(){
  loadSaveCalib(false);
}

void IMU::deleteCalib(){
  int addr = ADDR;
  eewrite(addr, (short)0); // magic  
  accOfs.x=accOfs.y=accOfs.z=0;
  accScale.x=accScale.y=accScale.z=2;  
  comOfs.x=comOfs.y=comOfs.z=0;
  comScale.x=comScale.y=comScale.z=2;  
  Console.println("IMU calibration deleted");  
}

void IMU::printPt(point_float_t p){
  Console.print(p.x);
  Console.print(",");    
  Console.print(p.y);  
  Console.print(",");
  Console.println(p.z);  
}

void IMU::printCalib(){
  Console.println(F("--------"));
  Console.print(F("accOfs="));
  printPt(accOfs);
  Console.print(F("accScale="));
  printPt(accScale);
  Console.print(F("comOfs="));
  printPt(comOfs);
  Console.print(F("comScale="));
  printPt(comScale);  
  Console.println(F("--------"));
}



// calculate gyro offsets
void IMU::calibGyro(){
  Console.println(F("---calibGyro---"));    
  useGyroCalibration = false;
  gyroOfs.x = gyroOfs.y = gyroOfs.z = 0;
  point_float_t ofs;
  int x=0;
  while(true){    
    float zmin =  99999;
    float zmax = -99999;  
    gyroNoise = 0;
    ofs.x = ofs.y = ofs.z = 0;      
    for (int i=0; i < 50; i++){
      delay(10);
      if (imuUseMPU) {
        readMPUGyro();
      } else {
        readL3G4200D(true);      
      }

      zmin = min(zmin, gyro.z);
      zmax = max(zmax, gyro.z);
      ofs.x += ((float)gyro.x)/ 50.0;
      ofs.y += ((float)gyro.y)/ 50.0;
      ofs.z += ((float)gyro.z)/ 50.0;          
      gyroNoise += sq(gyro.z-gyroOfs.z) /50.0;   // noise is computed with last offset calculation
    }
    Console.print(F("gyro calib min="));
    Console.print(zmin);
    Console.print(F("\tmax="));    
    Console.print(zmax);
    Console.print(F("\tofs="));        
    Console.print(ofs.z);    
    Console.print(F("\tnoise="));                
    Console.println(gyroNoise);  
    if (gyroNoise < 20) break; // optimum found    
    x++;
    if (x > 50) break; // no optimum, but want to go ahead
    gyroOfs = ofs; // new offset found
  }  
  useGyroCalibration = true;
  Console.print(F("counter="));
  Console.println(gyroCounter);  
  Console.print(F("ofs="));
  printPt(gyroOfs);  
  Console.println(F("------------"));  
}      

// ADXL345B acceleration sensor driver
void  IMU::initADXL345B(){
  I2CwriteTo(ADXL345B, 0x2D, 0);
  I2CwriteTo(ADXL345B, 0x2D, 16);
  I2CwriteTo(ADXL345B, 0x2D, 8);         
}

void IMU::readADXL345B(){  
  uint8_t buf[6];
  if (I2CreadFrom(ADXL345B, 0x32, 6, (uint8_t*)buf) != 6){
    errorCounter++;
    return;
  }
  // Convert the accelerometer value to G's. 
  // With 10 bits measuring over a +/-4g range we can find how to convert by using the equation:
  // Gs = Measurement Value * (G-range/(2^10)) or Gs = Measurement Value * (8/1024)
  // ( *0.0078 )
  float x=(int16_t) (((uint16_t)buf[1]) << 8 | buf[0]); 
  float y=(int16_t) (((uint16_t)buf[3]) << 8 | buf[2]); 
  float z=(int16_t) (((uint16_t)buf[5]) << 8 | buf[4]); 
  //Console.println(z);
  if (useAccCalibration){
    x -= accOfs.x;
    y -= accOfs.y;
    z -= accOfs.z;
    x /= accScale.x*0.5;    
    y /= accScale.y*0.5;    
    z /= accScale.z*0.5;
    acc.x = x;
    //Console.println(z);
    acc.y = y;
    acc.z = z;
  } else {
    acc.x = x;
    acc.y = y;
    acc.z = z;
  }
  /*float accXreal = x + sin(gyroYpr.pitch);
  accXmax = max(accXmax, accXreal );    
  accXmin = min(accXmin, accXreal );        */
  //float amag = sqrt(Xa*Xa + Ya*Ya + Za*Za);
  //Xa /= amag;
  //Ya /= amag;
  //Za /= amag;  
  accelCounter++;
}

// L3G4200D gyro sensor driver
boolean IMU::initL3G4200D(){
  Console.println(F("initL3G4200D"));
  uint8_t buf[6];    
  int retry = 0;
  while (true){
    I2CreadFrom(L3G4200D, 0x0F, 1, (uint8_t*)buf);
    if (buf[0] != 0xD3) {        
      Console.println(F("gyro read error"));
      retry++;
      if (retry > 2){
        errorCounter++;
        return false;
      }
      delay(1000);            
    } else break;
  }
  // Normal power mode, all axes enabled, 100 Hz
  I2CwriteTo(L3G4200D, 0x20, 0b00001100);    
  // 2000 dps (degree per second)
  I2CwriteTo(L3G4200D, 0x23, 0b00100000);      
  I2CreadFrom(L3G4200D, 0x23, 1, (uint8_t*)buf);
  if (buf[0] != 0b00100000){
      Console.println(F("gyro write error")); 
      while(true);
  }  
  // fifo mode 
 // I2CwriteTo(L3G4200D, 0x24, 0b01000000);        
 // I2CwriteTo(L3G4200D, 0x2e, 0b01000000);          
  delay(250);
  calibGyro();    
  return true;
}

// MPU9250 9-axis sensor driver
boolean IMU::initMPU(){
  Console.println(F("initMPU9250"));
  
  // Read the WHO_AM_I register, this is a good test of communication
  uint8_t buf[6];    
  int retry = 0;
  I2CreadFrom(MPU9250_ADDRESS, WHO_AM_I_MPU9250, 1, (uint8_t*)buf);
  byte c = buf[0];

  
  Console.print(F("MPU9250 I AM 0x"));
  Console.print(c, HEX);
  Console.print(F(" I should be 0x"));
  Console.println(0x71, HEX);

  if (c != 0x71) // WHO_AM_I should always be 0x71
  {
    Console.print("Could not connect to MPU9250: 0x");
    Console.println(c, HEX);

    // Communication failed, stop here
    Console.println(F("Communication failed, abort!"));
    return false; // don't loop forever if communication failure
  }

  Console.println(F("MPU9250 is online..."));

  // Start by performing self test and reporting values
  MPU9250SelfTest(SelfTest);
  Console.print(F("x-axis self test: acceleration trim within : "));
  Console.print(SelfTest[0],1); Console.println("% of factory value");
  Console.print(F("y-axis self test: acceleration trim within : "));
  Console.print(SelfTest[1],1); Console.println("% of factory value");
  Console.print(F("z-axis self test: acceleration trim within : "));
  Console.print(SelfTest[2],1); Console.println("% of factory value");
  Console.print(F("x-axis self test: gyration trim within : "));
  Console.print(SelfTest[3],1); Console.println("% of factory value");
  Console.print(F("y-axis self test: gyration trim within : "));
  Console.print(SelfTest[4],1); Console.println("% of factory value");
  Console.print(F("z-axis self test: gyration trim within : "));
  Console.print(SelfTest[5],1); Console.println("% of factory value");

  // Calibrate gyro and accelerometers, load biases in bias registers
  calibrateMPU9250(gyroBias, accelBias);
  delay(1000); 

  initMPU9250();
  // Initialize device for active mode read of acclerometer, gyroscope, and
  // temperature
  Console.println("MPU9250 initialized for active data mode....");

  // Read the WHO_AM_I register of the magnetometer, this is a good test of
  // communication
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  I2CreadFrom(AK8963_ADDRESS, WHO_AM_I_AK8963, 1, (uint8_t*)rawData);        // Read the six raw data registers into data array
  byte d = rawData[0];
  Console.print("AK8963 ");
  Console.print("I AM 0x");
  Console.print(d, HEX);
  Console.print(" I should be 0x");
  Console.println(0x48, HEX);

  if (d != 0x48)
  {
    // Communication failed, stop here
    Console.println(F("Communication failed on WHO_AM_I for magnetometer, abort!"));
    return false;
  }

  // Get magnetometer calibration from AK8963 ROM
  initAK8963(magCalibration);
  // Initialize device for active mode read of magnetometer
  Console.println("AK8963 initialized for active data mode....");

  //  Serial.println("Calibration values: ");
  Console.print("X-Axis factory sensitivity adjustment value ");
  Console.println(magCalibration[0], 2);
  Console.print("Y-Axis factory sensitivity adjustment value ");
  Console.println(magCalibration[1], 2);
  Console.print("Z-Axis factory sensitivity adjustment value ");
  Console.println(magCalibration[2], 2);

  getAres();
  getMres();
  getGres();
  
  delay(250);
  calibGyro();    
  return true;
}


void IMU::readL3G4200D(boolean useTa){  
  now = micros();
  float Ta = 1;
  //if (useTa) {
    Ta = ((now - lastGyroTime) / 1000000.0);    
    //float Ta = ((float)(millis() - lastGyroTime)) / 1000.0; 			    
    lastGyroTime = now;
    if (Ta > 0.5) Ta = 0;   // should only happen for the very first call
    //lastGyroTime = millis();    
  //}  
  uint8_t fifoSrcReg = 0;  
  I2CreadFrom(L3G4200D, 0x2F, sizeof(fifoSrcReg), &fifoSrcReg);         // read the FIFO_SRC_REG
   // FIFO_SRC_REG
   // 7: Watermark status. (0: FIFO filling is lower than WTM level; 1: FIFO filling is equal or higher than WTM level)
   // 6: Overrun bit status. (0: FIFO is not completely filled; 1:FIFO is completely filled)
   // 5: FIFO empty bit. (0: FIFO not empty; 1: FIFO empty)
   // 4..0: FIFO stored data level
   //Console.print("FIFO_SRC_REG: "); Console.println(fifoSrcReg, HEX);
  uint8_t countOfData = (fifoSrcReg & 0x1F) + 1;   
  //  if (bitRead(fifoSrcReg, 6)==1) Console.println(F("IMU error: FIFO overrun"));

  memset(gyroFifo, 0, sizeof(gyroFifo[0])*32);
  I2CreadFrom(L3G4200D, 0xA8, sizeof(gyroFifo[0])*countOfData, (uint8_t *)gyroFifo);         // the first bit of the register address specifies we want automatic address increment
  //I2CreadFrom(L3G4200D, 0x28, sizeof(gyroFifo[0])*countOfData, (uint8_t *)gyroFifo);         // the first bit of the register address specifies we want automatic address increment

  gyro.x = gyro.y = gyro.z = 0;
  //Console.print("fifo:");
  //Console.println(countOfData);
  if (!useGyroCalibration) countOfData = 1;
  for (uint8_t i=0; i<countOfData; i++){
      gyro.x += (int16_t) (((uint16_t)gyroFifo[i].xh) << 8 | gyroFifo[i].xl);
      gyro.y += (int16_t) (((uint16_t)gyroFifo[i].yh) << 8 | gyroFifo[i].yl);
      gyro.z += (int16_t) (((uint16_t)gyroFifo[i].zh) << 8 | gyroFifo[i].zl);
      if (useGyroCalibration){
        gyro.x -= gyroOfs.x;
        gyro.y -= gyroOfs.y;
        gyro.z -= gyroOfs.z;               
      }
  }
  if (useGyroCalibration){
    gyro.x *= 0.07 * PI/180.0;  // convert to radiant per second
    gyro.y *= 0.07 * PI/180.0; 
    gyro.z *= 0.07 * PI/180.0;      
  }
  gyroCounter++;
}

// HMC5883L compass sensor driver
void  IMU::initHMC5883L(){
  I2CwriteTo(HMC5883L, 0x00, 0x70);  // 8 samples averaged, 75Hz frequency, no artificial bias.       
  //I2CwriteTo(HMC5883L, 0x01, 0xA0);      // gain
  I2CwriteTo(HMC5883L, 0x01, 0x20);   // gain
  I2CwriteTo(HMC5883L, 0x02, 00);    // mode         
}

void IMU::readHMC5883L(){    
  uint8_t buf[6];  
  if (I2CreadFrom(HMC5883L, 0x03, 6, (uint8_t*)buf) != 6){
    errorCounter++;
    return;
  }
  // scale +1.3Gauss..-1.3Gauss  (*0.00092)  
  float x = (int16_t) (((uint16_t)buf[0]) << 8 | buf[1]);
  float y = (int16_t) (((uint16_t)buf[4]) << 8 | buf[5]);
  float z = (int16_t) (((uint16_t)buf[2]) << 8 | buf[3]);  
  if (useComCalibration){
    x -= comOfs.x;
    y -= comOfs.y;
    z -= comOfs.z;
    x /= comScale.x*0.5;    
    y /= comScale.y*0.5;    
    z /= comScale.z*0.5;
    com.x = x;
    //Console.println(z);
    com.y = y;
    com.z = z;
  } else {
    com.x = x;
    com.y = y;
    com.z = z;
  }  
}

float IMU::sermin(float oldvalue, float newvalue){
  if (newvalue < oldvalue) {
    Console.print(".");
    digitalWrite(pinLED, true);
  }
  return min(oldvalue, newvalue);
}

float IMU::sermax(float oldvalue, float newvalue){
  if (newvalue > oldvalue) {
    Console.print(".");
    digitalWrite(pinLED, true);
  }
  return max(oldvalue, newvalue);
}

void IMU::calibComStartStop(){  
  while (Console.available()) Console.read();  
  if (state == IMU_CAL_COM){
    // stop 
    Console.println(F("com calib completed"));    
    calibrationAvail = true;
    float xrange = comMax.x - comMin.x;
    float yrange = comMax.y - comMin.y;
    float zrange = comMax.z - comMin.z;
    comOfs.x = xrange/2 + comMin.x;
    comOfs.y = yrange/2 + comMin.y;
    comOfs.z = zrange/2 + comMin.z;
    comScale.x = xrange;
    comScale.y = yrange;  
    comScale.z = zrange;
    saveCalib();  
    printCalib();
    useComCalibration = true; 
    state = IMU_RUN;    
    // completed sound
    Buzzer.tone(600);
    delay(200); 
    Buzzer.tone(880);
    delay(200); 
    Buzzer.tone(1320);              
    delay(200); 
    Buzzer.noTone();    
    delay(500);
  } else {
    // start
    Console.println(F("com calib..."));
    Console.println(F("rotate sensor 360 degree around all three axis"));
    foundNewMinMax = false;  
    useComCalibration = false;
    state = IMU_CAL_COM;  
    comMin.x = comMin.y = comMin.z = 9999;
    comMax.x = comMax.y = comMax.z = -9999;
  }
}

boolean IMU::newMinMaxFound(){
  boolean res = foundNewMinMax;
  foundNewMinMax = false;
  return res;
}  
  
void IMU::calibComUpdate(){
  comLast = com;
  delay(20);
  if (imuUseMPU) {
    readMPUMag();
  } else {
    readHMC5883L();  
  }
  boolean newfound = false;
  if ( (abs(com.x-comLast.x)<10) &&  (abs(com.y-comLast.y)<10) &&  (abs(com.z-comLast.z)<10) ){
    if (com.x < comMin.x) { 
      comMin.x = com.x;
      newfound = true;      
    }
    if (com.y < comMin.y) { 
      comMin.y = com.y;
      newfound = true;      
    }
    if (com.z < comMin.z) { 
      comMin.z = com.z;
      newfound = true;      
    }
    if (com.x > comMax.x) { 
      comMax.x = com.x;
      newfound = true;      
    }
    if (com.y > comMax.y) { 
      comMax.y = com.y;
      newfound = true;      
    }
    if (com.z > comMax.z) { 
      comMax.z = com.z;
      newfound = true;      
    }    
    if (newfound) {      
      foundNewMinMax = true;
      Buzzer.tone(440);
      Console.print("x:");
      Console.print(comMin.x);
      Console.print(",");
      Console.print(comMax.x);
      Console.print("\t  y:");
      Console.print(comMin.y);
      Console.print(",");
      Console.print(comMax.y);
      Console.print("\t  z:");
      Console.print(comMin.z);
      Console.print(",");
      Console.print(comMax.z);    
      Console.println("\t");
    } else Buzzer.noTone();   
  }    
}

// calculate acceleration sensor offsets
boolean IMU::calibAccNextAxis(){  
  boolean complete = false;
  Buzzer.tone(440);
  while (Console.available()) Console.read();  
  useAccCalibration = false;  
  if (calibAccAxisCounter >= 6) calibAccAxisCounter = 0;
  if (calibAccAxisCounter == 0){
    // restart
    Console.println(F("acc calib restart..."));
    accMin.x = accMin.y = accMin.z = 99999;
    accMax.x = accMax.y = accMax.z = -99999;    
  }
  point_float_t pt = {0,0,0};
  for (int i=0; i < 100; i++){        
    if (imuUseMPU) {
      readMPUAcc();
    } else {
      readADXL345B();            
    }
    pt.x += acc.x / 100.0;
    pt.y += acc.y / 100.0;
    pt.z += acc.z / 100.0;                  
    Console.print(acc.x);
    Console.print(",");
    Console.print(acc.y);
    Console.print(",");
    Console.println(acc.z);
    delay(1);
  }
  accMin.x = min(accMin.x, pt.x);
  accMax.x = max(accMax.x, pt.x);         
  accMin.y = min(accMin.y, pt.y);
  accMax.y = max(accMax.y, pt.y);         
  accMin.z = min(accMin.z, pt.z);
  accMax.z = max(accMax.z, pt.z);           
  calibAccAxisCounter++;        
  useAccCalibration = true;  
  Console.print("side ");
  Console.print(calibAccAxisCounter);
  Console.println(" of 6 completed");    
  if (calibAccAxisCounter == 6){    
    // all axis complete 
    float xrange = accMax.x - accMin.x;
    float yrange = accMax.y - accMin.y;
    float zrange = accMax.z - accMin.z;
    accOfs.x = xrange/2 + accMin.x;
    accOfs.y = yrange/2 + accMin.y;
    accOfs.z = zrange/2 + accMin.z;
    accScale.x = xrange;
    accScale.y = yrange;  
    accScale.z = zrange;    
    printCalib();
    saveCalib();    
    Console.println("acc calibration completed");    
    complete = true;
    // completed sound
    Buzzer.tone(600);
    delay(200); 
    Buzzer.tone(880);
    delay(200); 
    Buzzer.tone(1320);              
    delay(200); 
  };
  Buzzer.noTone();
  delay(500);
  return complete;
}      

// first-order complementary filter
// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()
float Complementary2(float newAngle, float newRate,int looptime, float angle) {
  float k=10;
  float dtc2=float(looptime)/1000.0;
  float x1 = (newAngle -   angle)*k*k;
  float y1 = dtc2*x1 + y1;
  float x2 = y1 + (newAngle -   angle)*2*k + newRate;
  angle = dtc2*x2 + angle;
  return angle;
}

// second-order complementary filter
// a=tau / (tau + loop time)
// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()
float Complementary(float newAngle, float newRate,int looptime, float angle) {
  float tau=0.075;
  float a=0.0;
  float dtC = float(looptime)/1000.0;
  a=tau/(tau+dtC);
  angle= a* (angle + newRate * dtC) + (1-a) * (newAngle);
  return angle;
}

// Kalman filter                                      
// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()
float Kalman(float newAngle, float newRate,int looptime, float x_angle)
{
  float Q_angle  =  0.01; //0.001
  float Q_gyro   =  0.0003;  //0.003
  float R_angle  =  0.01;  //0.03

  float x_bias = 0;
  float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
  float  y, S;
  float K_0, K_1;

  float dt = float(looptime)/1000;
  x_angle += dt * (newRate - x_bias);
  P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt;
  P_01 +=  - dt * P_11;
  P_10 +=  - dt * P_11;
  P_11 +=  + Q_gyro * dt;

  y = newAngle - x_angle;
  S = P_00 + R_angle;
  K_0 = P_00 / S;
  K_1 = P_10 / S;

  x_angle +=  K_0 * y;
  x_bias  +=  K_1 * y;
  P_00 -= K_0 * P_00;
  P_01 -= K_0 * P_01;
  P_10 -= K_1 * P_00;
  P_11 -= K_1 * P_01;

  return x_angle;
}

// scale setangle, so that both PI angles have the same sign    
float scalePIangles(float setAngle, float currAngle){
  if ((setAngle >= PI/2) && (currAngle <= -PI/2)) return (setAngle-2*PI);
    else if ((setAngle <= -PI/2) && (currAngle >= PI/2)) return (setAngle+2*PI);
    else return setAngle;
}

void IMU::update(){
  read();  
  now = millis();
  int looptime = (now - lastAHRSTime);
  lastAHRSTime = now;
  
  if (state == IMU_RUN){


    if (imuUseMPU) {
      // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
      // In this coordinate system, the positive z-axis is down toward Earth. 
      // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
      // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
      // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
      // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
      // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
      // applied in the correct order which for this configuration is yaw, pitch, and then roll.
      // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
      ypr.yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
      ypr.pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
      ypr.roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
      //ypr.pitch *= 180.0f / PI;
      //ypr.yaw   *= 180.0f / PI; 
      //ypr.yaw   += 2.8; // Declination at Cologne, Germany is 2.48° W  ± 0.36°  changing by  0.20° E per year on 2019-04-26
      //ypr.roll  *= 180.0f / PI;
    } else {

      // ------ roll, pitch --------------  
      float forceMagnitudeApprox = abs(acc.x) + abs(acc.y) + abs(acc.z);    
      //if (forceMagnitudeApprox < 1.2) {
      //Console.println(forceMagnitudeApprox);      
      accPitch   = atan2(-acc.x , sqrt(sq(acc.y) + sq(acc.z)));         
      accRoll    = atan2(acc.y , acc.z);       
      accPitch = scalePIangles(accPitch, ypr.pitch);
      accRoll  = scalePIangles(accRoll, ypr.roll);
      // complementary filter            
      ypr.pitch = Kalman(accPitch, gyro.x, looptime, ypr.pitch);  
      ypr.roll  = Kalman(accRoll,  gyro.y, looptime, ypr.roll);            
      /*} else {
        //Console.print("too much acceleration ");
        //Console.println(forceMagnitudeApprox);
        ypr.pitch = ypr.pitch + gyro.y * ((float)(looptime))/1000.0;
        ypr.roll  = ypr.roll  + gyro.x * ((float)(looptime))/1000.0;
      }*/
      ypr.pitch=scalePI(ypr.pitch);
      ypr.roll=scalePI(ypr.roll);
      // ------ yaw --------------
      // tilt-compensated yaw
      comTilt.x =  com.x  * cos(ypr.pitch) + com.z * sin(ypr.pitch);
      comTilt.y =  com.x  * sin(ypr.roll)         * sin(ypr.pitch) + com.y * cos(ypr.roll) - com.z * sin(ypr.roll) * cos(ypr.pitch);
      comTilt.z = -com.x  * cos(ypr.roll)         * sin(ypr.pitch) + com.y * sin(ypr.roll) + com.z * cos(ypr.roll) * cos(ypr.pitch);     
      comYaw = scalePI( atan2(comTilt.y, comTilt.x)  );  
      comYaw = scalePIangles(comYaw, ypr.yaw);
      //comYaw = atan2(com.y, com.x);  // assume pitch, roll are 0
      // complementary filter
      ypr.yaw = Complementary2(comYaw, -gyro.z, looptime, ypr.yaw);
      ypr.yaw = scalePI(ypr.yaw);
    }
  } 
  else if (state == IMU_CAL_COM) {
    calibComUpdate();
  }
}  

boolean IMU::init(){    
  loadCalib();
  printCalib();    
  if (imuUseMPU){
    if (!initMPU()) return false;
  } else {
    if (!initL3G4200D()) return false;
    initADXL345B();
    initHMC5883L();    
  }
  now = 0;  
  hardwareInitialized = true;
  return true;
}

int IMU::getCallCounter(){
  int res = callCounter;
  callCounter = 0;
  return res;
}

int IMU::getErrorCounter(){
  int res = errorCounter;
  errorCounter = 0;
  return res;
}

void IMU::read(){  
  if (!hardwareInitialized) {
    errorCounter++;
    return;
  }
  callCounter++;    
  if (imuUseMPU) {
    readMPUGyro();
    readMPUAcc();
    readMPUMag();
    readMPUTemp();

    NowMicros = micros();
    deltat = ((NowMicros - lastUpdateMicros)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdateMicros = NowMicros;

    MahonyQuaternionUpdate(acc.x, acc.y, acc.z, gyro.x*PI/180.0f, gyro.y*PI/180.0f, gyro.z*PI/180.0f, com.y, com.x, com.z, deltat);
    q = getQ();
  } else {
    readL3G4200D(true);
    readADXL345B();
    readHMC5883L();  
  }
  //calcComCal();
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void IMU::MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
   uint8_t selfTest[6];
   int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
   float factoryTrim[6];
   uint8_t FS = 0;
   uint8_t buf[6];    


  I2CwriteTo(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  I2CwriteTo(MPU9250_ADDRESS, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  I2CwriteTo(MPU9250_ADDRESS, GYRO_CONFIG, FS<<3);  // Set full scale range for the gyro to 250 dps
  I2CwriteTo(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  I2CwriteTo(MPU9250_ADDRESS, ACCEL_CONFIG, FS<<3); // Set full scale range for the accelerometer to 2 g

  for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer
  
    I2CreadFrom(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, (uint8_t*)rawData);        // Read the six raw data registers into data array
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
    
    I2CreadFrom(MPU9250_ADDRESS, GYRO_XOUT_H, 6, (uint8_t*)rawData);       // Read the six raw data registers sequentially into data array
    gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  
  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
    aAvg[ii] /= 200;
    gAvg[ii] /= 200;
  }
  
// Configure the accelerometer for self-test
   I2CwriteTo(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
   I2CwriteTo(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   delay(25);  // Delay a while to let the device stabilize

  for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer
  
    I2CreadFrom(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, (uint8_t*)rawData);  // Read the six raw data registers into data array
    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
    
    I2CreadFrom(MPU9250_ADDRESS, GYRO_XOUT_H, 6, (uint8_t*)rawData);  // Read the six raw data registers sequentially into data array
    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  
  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
    aSTAvg[ii] /= 200;
    gSTAvg[ii] /= 200;
  }   
  
 // Configure the gyro and accelerometer for normal operation
   I2CwriteTo(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);  
   I2CwriteTo(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);  
   delay(25);  // Delay a while to let the device stabilize
   
   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
   I2CreadFrom(MPU9250_ADDRESS, SELF_TEST_X_ACCEL, 1, (uint8_t*)rawData);  // X-axis accel self-test results
   selfTest[0] = rawData[0];
   I2CreadFrom(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL, 1, (uint8_t*)rawData);  // Y-axis accel self-test results
   selfTest[1] = rawData[0];
   I2CreadFrom(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL, 1, (uint8_t*)rawData);  // Z-axis accel self-test results
   selfTest[2] = rawData[0];
   I2CreadFrom(MPU9250_ADDRESS, SELF_TEST_X_GYRO, 1, (uint8_t*)rawData);  // X-axis gyro self-test results
   selfTest[3] = rawData[0];
   I2CreadFrom(MPU9250_ADDRESS, SELF_TEST_Y_GYRO, 1, (uint8_t*)rawData);  // Y-axis gyro self-test results
   selfTest[4] = rawData[0];
   I2CreadFrom(MPU9250_ADDRESS, SELF_TEST_Z_GYRO, 1, (uint8_t*)rawData);  // Z-axis gyro self-test results
   selfTest[5] = rawData[0];

  // Retrieve factory self-test value from self-test code reads
   factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
   factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
   factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
   factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
   factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
   factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation
 
 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get percent, must multiply by 100
   for (int i = 0; i < 3; i++) {
     destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.;   // Report percent differences
     destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.; // Report percent differences
   }
   
}
void IMU::initAK8963(float * destination)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  I2CwriteTo(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  delay(10);
  I2CwriteTo(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  I2CreadFrom(AK8963_ADDRESS, AK8963_ASAX, 3, (uint8_t*)rawData);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128)/256. + 1.;  
  destination[2] =  (float)(rawData[2] - 128)/256. + 1.; 
  I2CwriteTo(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
  delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  I2CwriteTo(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  delay(10);
}


void IMU::initMPU9250()
{  
 // wake up device
  I2CwriteTo(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
  delay(100); // Wait for all registers to reset 

 // get stable time source
  I2CwriteTo(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200); 
  
 // Configure Gyro and Thermometer
 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
 // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  I2CwriteTo(MPU9250_ADDRESS, CONFIG, 0x03);  

 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  I2CwriteTo(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
                                    // determined inset in CONFIG above
  uint8_t rawData[1];

 // Set gyroscope full scale range
 // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  I2CreadFrom(MPU9250_ADDRESS, GYRO_CONFIG, 1, (uint8_t*)rawData); // get current GYRO_CONFIG register value
  uint8_t c = rawData[0];
 // c = c & ~0xE0; // Clear self-test bits [7:5] 
  c = c & ~0x03; // Clear Fchoice bits [1:0] 
  c = c & ~0x18; // Clear GFS bits [4:3]
  c = c | Gscale << 3; // Set full scale range for the gyro
 // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  I2CwriteTo(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register
  
 // Set accelerometer full-scale range configuration
  I2CreadFrom(MPU9250_ADDRESS, ACCEL_CONFIG, 1, (uint8_t*)rawData); // get current ACCEL_CONFIG register value
  c = rawData[0];
 // c = c & ~0xE0; // Clear self-test bits [7:5] 
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | Ascale << 3; // Set full scale range for the accelerometer 
  I2CwriteTo(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  I2CreadFrom(MPU9250_ADDRESS, ACCEL_CONFIG2, 1, (uint8_t*)rawData); // get current ACCEL_CONFIG2 register value
  c = rawData[0];
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  I2CwriteTo(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master
   //I2CwriteTo(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);    
   I2CwriteTo(MPU9250_ADDRESS, INT_PIN_CFG, 0x12);    
   I2CwriteTo(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
   delay(100);
}

void IMU::getMres() {
  switch (Mscale)
  {
   // Possible magnetometer scales (and their register bit settings) are:
  // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
          mRes = 10.*4912./8190.; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          mRes = 10.*4912./32760.0; // Proper scale to return milliGauss
          break;
  }
}

void IMU::getGres() {
  switch (Gscale)
  {
  // Possible gyro scales (and their register bit settings) are:
  // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}

void IMU::getAres() {
  switch (Ascale)
  {
  // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void IMU::calibrateMPU9250(float * dest1, float * dest2)
{  
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
 // reset device
  I2CwriteTo(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);
   
 // get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
 // else use the internal oscillator, bits 2:0 = 001
  I2CwriteTo(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  
  I2CwriteTo(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
  delay(200);                                    

// Configure device for bias calculation
  I2CwriteTo(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
  I2CwriteTo(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
  I2CwriteTo(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  I2CwriteTo(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
  I2CwriteTo(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  I2CwriteTo(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);
  
// Configure MPU6050 gyro and accelerometer for bias calculation
  I2CwriteTo(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  I2CwriteTo(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  I2CwriteTo(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  I2CwriteTo(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
 
  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
  I2CwriteTo(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO  
  I2CwriteTo(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
  I2CwriteTo(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  I2CreadFrom(MPU9250_ADDRESS, FIFO_COUNTH, 2, (uint8_t*)data); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
  
  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    I2CreadFrom(MPU9250_ADDRESS, FIFO_R_W, 12, (uint8_t*)data);// read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    
    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
            
}
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}
   
// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;
  
// Push gyro biases to hardware registers
  I2CwriteTo(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
  I2CwriteTo(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  I2CwriteTo(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
  I2CwriteTo(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  I2CwriteTo(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
  I2CwriteTo(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
  
// Output scaled gyro biases for display in the main program
  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  I2CreadFrom(MPU9250_ADDRESS, XA_OFFSET_H, 2, (uint8_t*)data);
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  I2CreadFrom(MPU9250_ADDRESS, YA_OFFSET_H, 2, (uint8_t*)data);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  I2CreadFrom(MPU9250_ADDRESS, ZA_OFFSET_H, 2, (uint8_t*)data);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  
  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  
  for(ii = 0; ii < 3; ii++) {
    if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }
  
  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
  
  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
 
// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
/*  I2CwriteTo(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
  I2CwriteTo(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
  I2CwriteTo(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
  I2CwriteTo(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
  I2CwriteTo(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
  I2CwriteTo(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]); */

// Output scaled accelerometer biases for display in the main program
   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

void IMU::readMPUGyro(){  
  now = micros();
  lastGyroTime = now;
  uint8_t rawData[6];  // x/y/z gyro register data stored here

  I2CreadFrom(MPU9250_ADDRESS, GYRO_XOUT_H, 6, (uint8_t *)rawData);         // the first bit of the register address specifies we want automatic address increment

  gyro.x = gyro.y = gyro.z = 0;
  float x = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  float y = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  float z = ((int16_t)rawData[4] << 8) | rawData[5] ; 

  gyro.x = x*gRes;
  gyro.y = y*gRes;
  gyro.z = z*gRes;
  if (gyro.x > 250) gyro.x = 500-gyro.x;
  if (gyro.y > 250) gyro.y = 500-gyro.y;
  if (gyro.z > 250) gyro.z = 500-gyro.z;
  gyroCounter++;

}


void IMU::readMPUMag(){    
  uint8_t buf[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition  
  uint8_t rawData[1];
  I2CreadFrom(AK8963_ADDRESS, AK8963_ST1, 1, (uint8_t*)rawData);
  byte c = rawData[0];
  if(c & 0x01) { // wait for magnetometer data ready bit to be set
    I2CreadFrom(AK8963_ADDRESS, AK8963_XOUT_L, 7, (uint8_t*)buf);
    uint8_t d = rawData[6]; // End data read by reading ST2 register
    if(!(d & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
      // scale +1.3Gauss..-1.3Gauss  (*0.00092)  
      float x = (int16_t) (((uint16_t)buf[1]) << 8 | buf[0]);
      float y = (int16_t) (((uint16_t)buf[3]) << 8 | buf[2]);
      float z = (int16_t) (((uint16_t)buf[5]) << 8 | buf[4]);  

      if (useComCalibration){
        x -= comOfs.x;
        y -= comOfs.y;
        z -= comOfs.z;
        x /= comScale.x*0.5;    
        y /= comScale.y*0.5;    
        z /= comScale.z*0.5;
        com.x = x;
        //Console.println(z);
        com.y = y;
        com.z = z;
      } else {
      magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
      magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
      magbias[2] = +125.;  // User environmental x-axis correction in milliGauss
      
      // Calculate the magnetometer values in milliGauss
      // Include factory calibration per data sheet and user environmental corrections
      com.x = (float)x*mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
      com.y = (float)y*mRes*magCalibration[1] - magbias[1];  
      com.z = (float)z*mRes*magCalibration[2] - magbias[2];   
        //com.x = x;
        //com.y = y;
        //com.z = z;
      }  

    }
  }
}

void IMU::readMPUAcc(){  
  uint8_t buf[6];
  if (I2CreadFrom(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, (uint8_t*)buf) != 6){
    errorCounter++;
    return;
  }
  // Convert the accelerometer value to G's. 
  // With 10 bits measuring over a +/-4g range we can find how to convert by using the equation:
  // Gs = Measurement Value * (G-range/(2^10)) or Gs = Measurement Value * (8/1024)
  // ( *0.0078 )
  float x=(int16_t) (((uint16_t)buf[0]) << 8 | buf[1]); 
  float y=(int16_t) (((uint16_t)buf[2]) << 8 | buf[3]); 
  float z=(int16_t) (((uint16_t)buf[4]) << 8 | buf[5]); 
  //Console.println(z);
  acc.x = x*aRes;
  acc.y = y*aRes;
  acc.z = z*aRes;
  accelCounter++;
}

void IMU::readMPUTemp(){  
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  I2CreadFrom(MPU9250_ADDRESS, TEMP_OUT_H, 2, (uint8_t*)rawData);
  int16_t temp = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
  // Temperature in degrees Centigrade
  temperature = ((float) temp) / 333.87 + 21.0;
  //Console.print("Current temperature is ");
  //Console.print(temperature);
  //Console.println(" degrees Celsius.");
}      




