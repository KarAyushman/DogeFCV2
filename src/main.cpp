#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Adafruit_BMP280.h"
#include "SimpleKalmanFilter.h"
#include <math.h>
#include <SD.h>
#define SOUND

const int R_LED = 5;
const int G_LED = 6;
const int B_LED = 9;
const int BUZZ = 10;
const int PY1 = 21;

// =============================================
// ===               STATE                   ===
// =============================================

int16_t launchState = 0, pyroState = 0, landState = 0, abortState = 0;
int16_t launchAtt = 0, abortAtt = 0, landAtt = 0, launchTime = 0, launchAlt = 0;
int16_t landprev = 0, buzzDel = 0, buzzState = 0, buzzer = 0, armed = 0;
double prevBuzzTime = 0.00;

// =============================================
// ===              MPU6050                  ===
// =============================================

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
double launchAx = 0.00, launchAy = 0.00, launchAz = 0.00;

double yaw = 0.00;
double pitch = 0.00;

bool zero_detect;
bool TurnOnZI = false;

// =============================================
// ===               BMP280                  ===
// =============================================

Adafruit_BMP280 bmp280;
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);
const float sealvl = 1013.25;
static float alt;
float est_alt;
float lastAlt = 0;
float temperature;
float pascal;

// =============================================
// ===              SD CARD                  ===
// =============================================
String filename;
File myFile;
int sd_count = 0;
bool FL = false;
bool fileclosed = false;

void Write() {
  myFile = SD.open(filename, FILE_WRITE);

  if (myFile) {

    //Writing in SD Card!
    myFile.print(millis());
    myFile.print(",");
    myFile.print(pascal);
    myFile.print(",");
    myFile.print(alt);
    myFile.print(",");
    myFile.print(est_alt);
    myFile.print(",");
    myFile.print(ax/16384.);
    myFile.print(",");
    myFile.print(ay/16384.);
    myFile.print(",");
    myFile.print(az/16384.);
    myFile.print(",");
    myFile.print(gx/131.072);
    myFile.print(",");
    myFile.print(gy/131.072);
    myFile.print(",");
    myFile.print(gz/131.072);
    myFile.print(",");
    myFile.print(yaw);
    myFile.print(",");
    myFile.print(pitch);
    myFile.print(",");
    myFile.print(launchState);
    myFile.print(",");
    myFile.print(pyroState);
    myFile.print(",");
    myFile.println(landState);
    myFile.close();
    //delay(10);
  }
}

boolean loadSDFile() {
  int i = 0;
  boolean file = false;

  while (!file && i < 1024) {
    filename = (String)i + "FL.csv";

    if (!SD.exists(filename)) {
      myFile = SD.open(filename, FILE_WRITE);
      delay(10);
      myFile.close();
      file = true;
    }
    i++;
  }

  return file;
}

void initializeSD() {
  SD.begin(15);
  //Create a file with new name
  if (!loadSDFile()) {
    Serial.println("Error");
  }
  Serial.println(filename);
  myFile = SD.open(filename, FILE_WRITE);
  Serial.println(myFile);
  if (myFile) {
    //Print Header Files  - - alt, pascal, est_alt, mpuPitch, mpuRoll, mpuYaw, OutputX, OutputY
    myFile.print("t");
    myFile.print(",");
    myFile.print("p"); 
    myFile.print(",");
    myFile.print("alt");
    myFile.print(",");
    myFile.print("KMF");
    myFile.print(",");
    myFile.print("ax");
    myFile.print(",");
    myFile.print("ay");
    myFile.print(",");
    myFile.print("az");
    myFile.print(",");
    myFile.print("gx");
    myFile.print(",");
    myFile.print("gy");
    myFile.print(",");
    myFile.print("gz");
    myFile.print(",");
    myFile.print("yaw");
    myFile.print(",");
    myFile.print("pitch");
    myFile.print(",");
    myFile.print("Launch");
    myFile.print(",");
    myFile.print("Pyro");
    myFile.print(",");
    myFile.println("Land");

    myFile.close();
  }
}

void initializeBMP() {

  // Serial.print("Intializing BMP280");

  if (!bmp280.begin(0x76,0x58)) {
    // Serial.println("BMP280 not initialized."); //Stop at Infinite Loop and Blink LED
    while (1);
  }

  /* Default settings from datasheet. */
  bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                     Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                     Adafruit_BMP280::SAMPLING_X1,    /* Pressure oversampling */
                     Adafruit_BMP280::FILTER_OFF,      /* Filtering. */
                     Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

  // Serial.println("BMP280 Initialization Successful!");
  
}

void initializeMPU() {
  // Serial.println("Initializing MPU6050");
  Wire.begin();
  accelgyro.initialize();
  accelgyro.setAccelerometerPowerOnDelay(3);
  accelgyro.setIntZeroMotionEnabled(TurnOnZI);
  accelgyro.setDHPFMode(1);
  accelgyro.setMotionDetectionThreshold(2);
  accelgyro.setZeroMotionDetectionThreshold(2);
  accelgyro.setZeroMotionDetectionDuration(1);
  // Serial.println("MPU6050 Initialization Successful!");
}

void motion() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  pitch = atan2((ay/16384.),sqrt(ax/16384.*ax/16384. + az/16384.*az/16384.))*57.32;
  yaw = -atan2((ax/16384.),sqrt(ay/16384.*ay/16384. + az/16384.*az/16384.))*57.32;
}

void getAlt() {
  alt = bmp280.readAltitude(sealvl);
  pascal = bmp280.readPressure();
  est_alt = pressureKalmanFilter.updateEstimate(alt);
}

void launchDet(){
  if(launchAtt >= 3 && launchState != 1){
    // Serial.println("Launch!");
    launchState = 1;
    launchTime = millis();
    buzzer = 0;
    analogWrite(R_LED, 0);
    analogWrite(G_LED, 255);
    analogWrite(B_LED, 0);
    getAlt();
    launchAlt = alt;
  }
  if(az/16384. >= 1.5 && launchState != 1){
    launchAtt++;
    delay(100);
    launchDet();
    Write();
  }
  else if(az/16384. <1.5 && launchState != 1){
    launchAtt = 0;
  }
}

void apDet(){
  if (lastAlt - alt >= 1 && launchState == 1 && pyroState == 0 && millis() - launchTime >= 4000) {
    delay(200);
    getAlt();
    Write();
    // Serial.println(F("P1"));

    if (lastAlt - alt >= 1) {

      delay(200);
      getAlt();
      Write();
      // Serial.println(F("P2"));

      if(lastAlt - alt >= 1) {
        analogWrite(B_LED, 0);
        digitalWrite(R_LED, 255);
        digitalWrite(G_LED, 0);
        // Serial.println(F("P3"));
        pyroState = 1;
        digitalWrite(PY1, HIGH);
        buzzer = 1;
        buzzDel = 500;
        Write();

      } else {
        lastAlt = alt;
      }
    } else {
      lastAlt = alt;
      }
    }
  else {
    lastAlt = alt;
  }
}

void landDet(){
  if(abs(alt - launchAlt) <= 2 && millis() - landprev > 1000 && millis() - launchTime >= 10000 && launchState == 1 && landState == 0) {
    landAtt++;
    //PASS 2
    if (abs(alt - launchAlt) <= 2 && landprev != 0 && landAtt == 5) {
      landState = 1;
      digitalWrite(PY1, LOW);
      analogWrite(R_LED, 255);
      analogWrite(G_LED, 0);
      analogWrite(B_LED, 255);
    }
    landprev = millis();
  } 
}

void beepy(){
  if(buzzer == 1){
    if(millis()-prevBuzzTime > buzzDel && buzzState == 0) {
      analogWrite(BUZZ, 255);
      buzzState = 1;
      prevBuzzTime = millis();
    }
    else if(millis()-prevBuzzTime > buzzDel && buzzState ==1){
      analogWrite(BUZZ, 0);
      buzzState = 0;
      prevBuzzTime = millis();
    }
  }
  else if (buzzer == 0){
    analogWrite(BUZZ, 0);
  }
}

void idle(){
  if(launchState==0 && pyroState == 0 && landState == 0){
    buzzer = 1;
    buzzDel = 1000;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(4, INPUT_PULLUP);
  pinMode(R_LED, OUTPUT);
  pinMode(G_LED, OUTPUT);
  pinMode(B_LED, OUTPUT);
  pinMode(BUZZ, OUTPUT);
  pinMode(PY1, OUTPUT);
  analogWrite(BUZZ, 255);
  analogWrite(R_LED, 0);
  analogWrite(G_LED, 73);
  analogWrite(B_LED, 62);
  // Serial.println(""
  // "░░░░░░░░░▄░░░░░░░░░░░░░░▄░░░░\n"
  // "░░░░░░░░▌▒█░░░░░░░░░░░▄▀▒▌░░░\n"
  // "░░░░░░░░▌▒▒█░░░░░░░░▄▀▒▒▒▐░░░\n"
  // "░░░░░░░▐▄▀▒▒▀▀▀▀▄▄▄▀▒▒▒▒▒▐░░░\n"
  // "░░░░░▄▄▀▒░▒▒▒▒▒▒▒▒▒█▒▒▄█▒▐░░░\n"
  // "░░░▄▀▒▒▒░░░▒▒▒░░░▒▒▒▀██▀▒▌░░░\n"
  // "░░▐▒▒▒▄▄▒▒▒▒░░░▒▒▒▒▒▒▒▀▄▒▒▌░░\n"
  // "░░▌░░▌█▀▒▒▒▒▒▄▀█▄▒▒▒▒▒▒▒█▒▐░░\n"
  // "░▐░░░▒▒▒▒▒▒▒▒▌██▀▒▒░░░▒▒▒▀▄▌░\n"
  // "░▌░▒▄██▄▒▒▒▒▒▒▒▒▒░░░░░░▒▒▒▒▌░\n"
  // "▀▒▀▐▄█▄█▌▄░▀▒▒░░░░░░░░░░▒▒▒▐░\n"
  // "▐▒▒▐▀▐▀▒░▄▄▒▄▒▒▒▒▒▒░▒░▒░▒▒▒▒▌\n"
  // "▐▒▒▒▀▀▄▄▒▒▒▄▒▒▒▒▒▒▒▒░▒░▒░▒▒▐░\n"
  // "░▌▒▒▒▒▒▒▀▀▀▒▒▒▒▒▒░▒░▒░▒░▒▒▒▌░\n"
  // "░▐▒▒▒▒▒▒▒▒▒▒▒▒▒▒░▒░▒░▒▒▄▒▒▐░░\n"
  // "░░▀▄▒▒▒▒▒▒▒▒▒▒▒░▒░▒░▒▄▒▒▒▒▌░░\n"
  // "░░░░▀▄▒▒▒▒▒▒▒▒▒▒▄▄▄▀▒▒▒▒▄▀░░░\n"
  // "░░░░░░▀▄▄▄▄▄▄▀▀▀▒▒▒▒▒▄▄▀░░░░░\n"
  // "░░░░░░░░░▒▒▒▒▒▒▒▒▒▒▀▀░░░░░░░░\n");
  // Serial.println("Monarch Aerospace©");
  // Serial.println("DogeFC v2 - by Ayushyman LP. Kar");
  // Serial.println("Initiaiting DogeFC...");
  delay(2000);
  // Serial.println("Init-ing MPU6050");
  initializeMPU();
  delay(500);
  // Serial.println("Init-ing BMP280");
  initializeBMP();
  delay(500);
  initializeSD();
  delay(500);
  digitalWrite(PY1, LOW);
  analogWrite(R_LED, 0);
  analogWrite(G_LED, 90);
  analogWrite(B_LED, 255);
  analogWrite(BUZZ, 0);
}

void loop() {
  
  if(armed==0){
    while(digitalRead(4)==HIGH){}
    tone(BUZZ, 2500, 1000);
    analogWrite(R_LED, 100);
    analogWrite(G_LED, 100);
    analogWrite(B_LED, 100);
    armed = 1;
    delay(10000);
    analogWrite(R_LED, 255);
    analogWrite(G_LED, 0);
    analogWrite(B_LED, 255);
  }
  else{
    idle();
    beepy();
    getAlt();
    motion();
    launchDet();
    apDet();
    landDet();
    Write();
  }  
}