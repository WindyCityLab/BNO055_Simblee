#include "SimbleeBLE.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

volatile bool BLEconnected = false;
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define MAX_READINGS 100
#define ACCELERATION_THRESHOLD 1.5
#define START_BYTE 0x7F
Adafruit_BNO055 bno = Adafruit_BNO055(55);

typedef struct dataStore {
  float orientationX;
  float orientationY;
  float orientationZ;
  float accelerationX;
  float accelerationY;
  float accelerationZ;
  float gyroX;
  float gyroY;
  float gyroZ;
} dataStore_t;

dataStore_t memory[MAX_READINGS];
int readingCount = 0;

char data[15];
long currenttime;
float maxAcceleration;
float currentMax;
bool radioActive;
int step = 0;
#define NUMBER_OF_PLAYERS 11
int jerseyNumbers[NUMBER_OF_PLAYERS] = {11, 16, 22, 31, 69, 1, 82, 88, 29, 42, 19};
long currentTime = 0;

void setup() {
  randomSeed(analogRead(6));
  radioActive = false;
  maxAcceleration = 0.0;
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);

  Serial.begin(9600);
  SimbleeBLE.deviceName = "TakeItNeo!";
  SimbleeBLE.customUUID = "6AF197E0-7CC8-45BB-85AE-DAE907E2382D";
  //  SimbleeBLE.advertisementData = "11";

  Serial.println("NWTN Initialized");
  //  digitalWrite(3, HIGH);

  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);
  displayCalStatus();
}
//int jerseyNumbers[NUMBER_OF_PLAYERS] = {11, 16, 22, 31, 69, 1, 82, 88, 29, 42, 19};

void setPlayerID()
{
  int index = random(0, NUMBER_OF_PLAYERS - 1);
  switch (index)
  {
    case 0 : SimbleeBLE.advertisementData = "11"; break;
    case 1 : SimbleeBLE.advertisementData = "16"; break;
    case 2 : SimbleeBLE.advertisementData = "22"; break;
    case 3 : SimbleeBLE.advertisementData = "31"; break;
    case 4 : SimbleeBLE.advertisementData = "69"; break;
    case 5 : SimbleeBLE.advertisementData = "1"; break;
    case 6 : SimbleeBLE.advertisementData = "82"; break;
    case 7 : SimbleeBLE.advertisementData = "88"; break;
    case 8 : SimbleeBLE.advertisementData = "29"; break;
    case 9 : SimbleeBLE.advertisementData = "42"; break;
    case 10 : SimbleeBLE.advertisementData = "19"; break;
  }
//  char value[2];
//  sprintf(value, "%2d", jerseyNumbers[index]);
//  Serial.print("Player ID index is "); Serial.println(index);
//  Serial.print("converted to string "); Serial.println(value);
//  String test = "31";
//  SimbleeBLE.advertisementData = test.c_str();
}

void loop() {
  sensors_event_t event;
  bno.getEvent(&event);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  if (BLEconnected && radioActive)
  {
    switch (step) {
      case 0 :
        {
          digitalWrite(3, HIGH);
          data[0] = START_BYTE;
          data[1] = 0x01;
          data[2] = 100;
          data[3] = 100;
          Serial.println("Sending count packet");
          Simblee_ULPDelay(SECONDS(1));
          SimbleeBLE.send(data, 4);
          step++;
        }
        break;
      case 1 :
        {
          for (int i = 0; i < MAX_READINGS; i++)
          {
            int16_t x = (uint16_t)memory[i].orientationX;
            int16_t y = (uint16_t)memory[i].orientationY;
            int16_t z = (uint16_t)memory[i].orientationZ;

            data[1] = (x >> 8) & 0xFF;
            data[0] = (byte)x;
            data[2] = (uint8_t)((memory[i].orientationX - x) * 100);
            data[4] = (y >> 8) & 0xFF;
            data[3] = (byte)y;
            data[5] = (uint8_t)((memory[i].orientationY - y) * 100);
            data[7] = (z >> 8) & 0xFF;
            data[6] = (byte)z;
            data[8] = (uint8_t)((memory[i].orientationZ - z) * 100);

            x = (uint16_t)memory[i].accelerationX;
            y = (uint16_t)memory[i].accelerationY;
            z = (uint16_t)memory[i].accelerationZ;

            data[10] = x >> 8 & 0xFF;
            data[9] = (byte)x;
            data[12] = y >> 8 & 0xFF;
            data[11] = (byte)y;
            data[14] = z >> 8 & 0xFF;
            data[13] = (byte)z;

            Serial.print(i); Serial.print("  ");
            Serial.print(memory[i].orientationX); Serial.print("  ");
            Serial.print(memory[i].orientationY); Serial.print("  ");
            Serial.print(memory[i].orientationZ); Serial.print(" -- ");
            Serial.print(x); Serial.print("  ");
            Serial.print(y); Serial.print("  ");
            Serial.println(z);

            SimbleeBLE.send(data, sizeof(data));
          }
          step++;
        }
        break;
      case 2: {
          Simblee_ULPDelay(SECONDS(1));
          digitalWrite(4, LOW);
          SimbleeBLE.end();
          BLEconnected = false;
          step = 0;
          maxAcceleration = 0;
          currentMax = 0;
          radioActive = false;
          readingCount = 0;
        }
        break;
    }
  }
  else
  {
    if (readingCount == 0) currentTime = millis();
    digitalWrite(2, LOW); digitalWrite(3, HIGH);
    float x = accel.x();
    float y = accel.y();
    float z = accel.z();
    currentMax = maxAcceleration;
    maxAcceleration = max(abs(z), max(abs(y), max(abs(x), maxAcceleration)));
    if (maxAcceleration > currentMax)
    {
      Serial.println(maxAcceleration / 9.8);
    }
    if (maxAcceleration / 9.8 > ACCELERATION_THRESHOLD)
    {
      if (readingCount < MAX_READINGS)
      {
        digitalWrite(2, HIGH); digitalWrite(3, LOW);
        memory[readingCount].orientationX = event.orientation.x;
        memory[readingCount].orientationY = event.orientation.y;
        memory[readingCount].orientationZ = event.orientation.z;
        memory[readingCount].accelerationX = event.acceleration.x;
        memory[readingCount].accelerationY = event.acceleration.y;
        memory[readingCount].accelerationZ = event.acceleration.z;
        memory[readingCount].gyroX = event.gyro.x;
        memory[readingCount].gyroY = event.gyro.y;
        memory[readingCount].gyroZ = event.gyro.z;
        readingCount++;
      }
      else
      {
        digitalWrite(2, LOW);
        Serial.print("Time: "); Serial.println(millis()-currentTime);
        if (!radioActive)
        {
          setPlayerID();
          SimbleeBLE.begin();
          radioActive = true;
        }
      }
    }
  }
  Simblee_ULPDelay(10);
}
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.println(mag, DEC);
}
void SimbleeBLE_onAdvertisement(bool start)
{
  if (start)
  {
    Serial.println("Advertising...");
  }
  else
  {
    Serial.println("Advertising off");
  }
}

void SimbleeBLE_onConnect()
{
  Serial.println("connected...");
  BLEconnected = true;
  digitalWrite(4, HIGH);
}

void SimbleeBLE_onDisconnect()
{
  BLEconnected = false;
  Serial.println("disconnected...");
  digitalWrite(4, LOW);
}

