#include <Arduino.h>

//BLE Module Libraries
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BLEMIDI.h"
#include "BluefruitConfig.h"

//Wire Libraries
#include<Wire.h>

//BLE Variables
#define FACTORYRESET_ENABLE 1
#define MINIMUM_FIRMWARE_VERSION "0.7.0"
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
Adafruit_BLEMIDI midi(ble);
bool isConnected = false;

//MPU Variables
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw, grav;
float AccErrorX=0.0, AccErrorY=0.0, AccErrorZ=0.0, GyroErrorX=0.0, GyroErrorY=0.0, GyroErrorZ=0.0;
float elapsedTime, currentTime, previousTime;

//Configuration variables
bool calibrateSensor = true;
bool sendMidi = true;
int threshold=2;
int current_note=-10000;
int current_vel=-10000;
int current_mod=-10000;

// Error helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void connected(void)
{
  isConnected = true;
  Serial.println(F(" CONNECTED!"));
  delay(1000);
}

void disconnected(void)
{
  Serial.println("disconnected");
  isConnected = false;
}

void setup(void)
{
  
  Serial.begin(115200);
  Serial.print("MIDI send enabled: ");
  if(sendMidi){Serial.println("True");} else{Serial.println("False");}
  
  /* MPU Initialization */
  Serial.println(F("Initialising MPU module... "));
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission

  if(calibrateSensor){
    Serial.println("Please don't move the sensor while calbrating...");
    calibrate(200);
    Serial.println("Done calibrating");
    printErrorValues();
  }
  
  /* BLE Initialization */
  Serial.print(F("Initialising the Bluefruit LE module... "));
  if (!ble.begin()){error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));} else{Serial.println( F("OK!") );}

  ble.echo(false);
  ble.setConnectCallback(connected);
  ble.setDisconnectCallback(disconnected);

  Serial.print(F("Enable MIDI: "));
  if (!midi.begin(true)){error(F(" ERROR!"));} else {Serial.println("OK!");}
  ble.verbose(false);
  Serial.print(F("Waiting for a connection...")); 
}

void sendControlChange(int control, int val){
   midi.send(0xB0, control, val);
}

void calibrate(int samples){
  // Read accelerometer x=samples times and accumulate the sum of each reading to calculate avg
  int c = 0;
  while (c < samples) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  AccErrorX = AccErrorX / samples;
  AccErrorY = AccErrorY / samples;
  AccErrorZ = AccErrorZ / samples;
  c = 0;
  // Read gyro x=samples times and accumulate the sum of each reading to calculate avg
  while (c < samples) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  GyroErrorX = GyroErrorX / samples;
  GyroErrorY = GyroErrorY / samples;
  GyroErrorZ = GyroErrorZ / samples;
}

void printErrorValues(){
  Serial.print("AccErrorX: ");
  Serial.print(AccErrorX);
  Serial.print(" AccErrorY: ");
  Serial.print(AccErrorY);
  Serial.print(" AccErrorZ: ");
  Serial.print(AccErrorZ);
  Serial.print(" GyroErrorX: ");
  Serial.print(GyroErrorX);
  Serial.print(" GyroErrorY: ");
  Serial.print(GyroErrorY);
  Serial.print(" GyroErrorZ: ");
  Serial.println(GyroErrorZ);
  }

void printAngles(float y, float p, float r){
  
  Serial.print("yaw: ");
  Serial.print(y);
  Serial.print(" pitch: ");
  Serial.print(p);
  Serial.print(" roll: ");
  Serial.println(r);
}

void printParameters(int n, int v, int m){
  Serial.print("Note: ");
  Serial.print(n);
  Serial.print(" Vel: ");
  Serial.print(v);
  Serial.print(" Mod: ");
  Serial.println(m);
}

void loop(void)
{
//  ble.update(500); // interval for each scanning ~ 500ms (non blocking)
//  if (! isConnected) 
//    return;
    
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers

  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value, divide the raw values by 16384, according to the datasheet
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value, divide the raw values by 16384, according to the datasheet
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value, divide the raw values by 16384, according to the datasheet
  
  // Calculating Roll and Pitch from the accelerometer data and removing error values
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX;
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY;
  
  //Calulating time variables
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  
  // === Read gyroscope data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
    
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  
  // Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX;
  GyroY = GyroY - GyroErrorY;
  GyroZ = GyroZ - GyroErrorZ;
  
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  
  //Calculates YPR
  yaw =  yaw + GyroZ * elapsedTime;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY; // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX; // Complementary filter - combine acceleromter and gyro angle values

  //printAngles(yaw,pitch,roll);
  
  //Transform ypr to midi parameters
  int note = map(pitch,-90,90,21,108); // Map from A0 to C8 all piano range
  int velocity = map(roll,-90,90,127,0);
  int modulation = map(yaw,-90,90,0,127);

  //printParameters(note, velocity, modulation);

  // === Send MIDI informaition === //
  if(sendMidi){
   if (note != current_note){
        Serial.print("Sending Note: ");
        Serial.print(note);
        Serial.print(", Velocitiy: ");
        Serial.print(velocity);
        Serial.print(", Mod: ");
        Serial.println(modulation);
        // send note off
        midi.send(0x80, current_note, velocity);
        delay(50);
        // send note on
        midi.send(0x90, note, velocity);
        delay(50);
    }
    if(modulation != current_mod){
      midi.send(0xB0, 1, modulation);
    }
    current_note = note;
    current_vel = velocity;
    current_mod = modulation;     
   }
}
