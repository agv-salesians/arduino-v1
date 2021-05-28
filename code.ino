#include <AFMotor.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// bluetooth

#define KEY 53 // key
#define POWER 52 // vout

#define RX_PIN 19
#define TX_PIN 18

#define BLUETOOTH_LIVE_BAUD_RATE 9600
#define BLUETOOTH_AP_BAUD_RATE 38400

// button

#define buttonFront 30
#define buttonBack 31
#define readDelay 100


// change history

#define arraySize 100

// encoder channels

#define channel1A 22
#define channel1B 23

#define channel2A 24
#define channel2B 25

#define channel3A 26
#define channel3B 27

#define channel4A 28
#define channel4B 29

const int mpuAddress = 0x68;  // Puede ser 0x68 o 0x69
MPU6050 mpu(mpuAddress);
int gx, gy, gz;

// bluetooth

enum mode {
  AT1,
  AT2,
  LIVE
};

// motor definitions

AF_DCMotor motor1(1, MOTOR12_64KHZ);
AF_DCMotor motor2(2, MOTOR12_64KHZ);
AF_DCMotor motor3(3, MOTOR12_64KHZ);
AF_DCMotor motor4(4, MOTOR12_64KHZ);

// buttons

float maxSpeed = 3.75;

bool buttonFrontLast = false;
bool buttonBackLast = false;
long lastButtonFrontChange = 0;
long lastButtonBackChange = 0;

// encoder control

bool lastValue1A = false;
long lastChange1A = 0;
bool lastValue1B = false;
long lastChange1B = 0;

bool lastValue2A = false;
long lastChange2A = 0;
bool lastValue2B = false;
long lastChange2B = 0;

bool lastValue3A = false;
long lastChange3A = 0;
bool lastValue3B = false;
long lastChange3B = 0;

bool lastValue4A = false;
long lastChange4A = 0;
bool lastValue4B = false;
long lastChange4B = 0;

// change history

int channelIndex1A = 0;
long channelHistory1A[arraySize] = {0};

int channelIndex2A = 0;
long channelHistory2A[arraySize] = {0};

int channelIndex3A = 0;
long channelHistory3A[arraySize] = {0};

int channelIndex4A = 0;
long channelHistory4A[arraySize] = {0};

// actual program

bool active = true;
long lastSpeedRead = 0;

// speed control
float speedHistory[4] = {0, 0, 0, 0};
float speeds[4] = {0, 0, 0, 0};
float targetSpeeds[4] = {0, 0, 0, 0};

float swervePower = 0;

int giroCalibrationMax = -32767;
int giroCalibrationMin = 32767;
int giroCalibrationSamples = 0;
int giroOffset = 0;
int giroTrigger = 0;
int giroRepeat = 0;
int lastGiroTrigger = 0;
bool straightMovement = false;
int swervePowerPI = 0;
bool giroCalibrationDone = false;

void setup() {

  Serial.begin(9600);

  while (!Serial) {
    // wait for serial
  }

  Wire.begin();
  mpu.initialize();
  mpu.testConnection();

  // encoder button

  pinMode(buttonFront, INPUT_PULLUP);
  pinMode(buttonBack, INPUT_PULLUP);

  // encoder setup

  pinMode(channel1A, INPUT);
  pinMode(channel1B, INPUT);

  pinMode(channel1A, INPUT);
  pinMode(channel1B, INPUT);

  pinMode(channel1A, INPUT);
  pinMode(channel1B, INPUT);

  pinMode(channel1A, INPUT);
  pinMode(channel1B, INPUT);

  pinMode(KEY, OUTPUT);
  pinMode(POWER, OUTPUT);

  Serial.println("Calibrating giro");
  while (!giroCalibrationDone) {
    calibrateGiro();
  }
  Serial.println("Giro calibrated. ");
  Serial.println(giroCalibrationMax);
  Serial.println(giroCalibrationMin);
  Serial.println("Giro calibrated. ");

  // Serial.println(setupBluetooth("1230"));
  changeMode(LIVE);

}

void calibrateGiro() {
  mpu.getRotation(&gx, &gy, &gz);
  if (giroCalibrationSamples >= 1000) {
    giroOffset = -((giroCalibrationMin + giroCalibrationMax) / 2);
    giroTrigger = abs(giroCalibrationMax - giroCalibrationMin) * 1.1; // +-10% range
    giroCalibrationDone = true;
  } else {
    if (gx > giroCalibrationMax) {
      giroCalibrationMax = gx;
    }
    if (gx < giroCalibrationMin) {
      giroCalibrationMin = gx;
    }
    giroCalibrationSamples++;
  }
}

void changeMode(mode mode) {
  Serial1.end();
  digitalWrite(KEY, LOW);
  digitalWrite(POWER, LOW);
  delay(20);
  if (mode == AT2) {
    Serial1.begin(BLUETOOTH_AP_BAUD_RATE);
    digitalWrite(KEY, HIGH);
    delay(800);
    digitalWrite(POWER, HIGH);
    delay(800);
    digitalWrite(KEY, LOW);
    Serial1.println("AT");
    while (!Serial1.available()) {

    }
  } else if (mode == AT1) {
    Serial1.begin(BLUETOOTH_LIVE_BAUD_RATE);
    digitalWrite(KEY, HIGH);
    digitalWrite(POWER, HIGH);
    delay(800);
    digitalWrite(KEY, LOW);
  } else {
    Serial1.begin(BLUETOOTH_LIVE_BAUD_RATE);
    digitalWrite(POWER, HIGH);
  }
}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

String setupBluetooth(String password) {
  changeMode(AT2);
  Serial1.println("AT+NAME=AGV");
  int completed = 0;
  while (completed < 1) {
    if (Serial1.available()) {
      Serial1.readString();
      completed++;
    }
  }
  Serial1.print("AT+UART=");
  Serial1.print(BLUETOOTH_LIVE_BAUD_RATE);
  Serial1.println(",0,0");
  while (completed < 2) {
    if (Serial1.available()) {
      Serial1.readString();
      completed++;
    }
  }
  Serial1.println("AT+ROLE=0");
  while (completed < 3) {
    if (Serial1.available()) {
      Serial1.readString();
      completed++;
    }
  }
  Serial1.print("AT+PSWD=");
  Serial1.println(password);
  while (completed < 4) {
    if (Serial1.available()) {
      Serial1.readString();
      completed++;
    }
  }
  Serial1.println("AT+ADDR");
  String address = "";
  while (completed < 6) {
    if (Serial1.available()) {
      if (completed == 5) {
        address = Serial1.readString();
        address = address.substring(6, address.length() - 6);
      }
      completed++;
    }
  }
  while (Serial1.available()) {
    Serial1.readString(); // read missing
  }
  changeMode(LIVE);
  return address;
}

bool shouldReadSpeed() {
  if (millis() - lastSpeedRead > 100) {
    lastSpeedRead = millis();
    return true;
  } else {
    return false;
  }
}

bool shouldButtonTrigger(int pin, bool &last, long &lastChange) {
  if (last != digitalRead(pin)) {
    last = !last;
    if (millis() - lastChange > 100 ) {
      lastChange = millis();
      return !last;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

bool isFrontButtonTriggered() {
  return shouldButtonTrigger(buttonFront, buttonFrontLast, lastButtonFrontChange);
}

bool isBackButtonTriggered() {
  return shouldButtonTrigger(buttonBack, buttonBackLast, lastButtonBackChange);
}

bool readEncoder(int channelPin, bool &lastValue, long &lastChange) {
  if (digitalRead(channelPin) != lastValue) {
    lastValue = !lastValue;
    if (lastValue) {
      lastChange = micros();
    }
    return true;
  } else {
    return false;
  }
}

void addToCyclicArray(long *targetArray, int &index, long value) {
  targetArray[index] = value;
  index++;
  if (index >= arraySize) {
    index = 0;
  }
}


// |  |  |  |  |  |  |
// 0  1  2  3  4
//    ^  .  .

int countFromCyclicArray(long *targetArray, int index, long until) {
  index += -1;
  if (index < 0) {
    index = arraySize - 1;
  }
  int untilIndex = index - arraySize;
  long val = 0;
  int count = 0;
  for (index + arraySize; index > untilIndex; index += -1) {
    if (index < 0) {
      val = targetArray[arraySize + index];
    } else {
      val = targetArray[index];
    }
    if (val <= 0 || val < until) {
      return count;
    } else {
      count++;
    }
  }
  return count;
}

float pulsesToRadians(int pulses, int totalPulses, long micros) {
  if (pulses > 0) {
    float radiansPerDefinedTime = ((float) pulses * (float) 2 * (float) 3.14) / (float) totalPulses;
    return radiansPerDefinedTime * (float) 1000000 / (float) micros;
  } else {
    return 0;
  }
}

float radiansToRPM(float radians) {
  return (radians / (float) 2 * (float) 3.14) * 60;
}

float getUpdatedSpeed(int count, float *previousSpeed, float *targetSpeed, float *history, int index) {
  if (targetSpeed[index] != 0) {
   
    float currentSpeed = pulsesToRadians(count, 1497, (long) readDelay * (long) 1000);
    history[index] = currentSpeed;
    float error = (targetSpeed[index] - currentSpeed) / maxSpeed * (float) 255;
    float deriv = 2 * error;

    //integr = integr + (error * 10* 0.05);

    if(straightMovement){
     
    }
   
    if (previousSpeed[index] > (float) 255) {
      return 255;
    } else if (previousSpeed[index] < -255) {
      return -255;
    } else {
      return deriv;
    }
  } else {
    return (float) 0;
  }
}

int directionFromSpeed(float speed) {
  if (speed > 0) {
    return BACKWARD;
  } else {
    return FORWARD;
  }
}

void applyNewSpeed(String val) {
  float m1 = (float) getValue(val, ':', 0).toFloat();
  float m2 = (float) getValue(val, ':', 1).toFloat();
  float m3 = (float) getValue(val, ':', 2).toFloat();
  float m4 = (float) getValue(val, ':', 3).toFloat();

  targetSpeeds[0] = m1;
  targetSpeeds[1] = m2;
  targetSpeeds[2] = m3;
  targetSpeeds[3] = m4;
}

void updateSwervePower() {
  if ((gz > giroCalibrationMax || gz < giroCalibrationMin) && abs(lastGiroTrigger - (gz + giroOffset)) > giroTrigger && (gz > giroCalibrationMax || gz < giroCalibrationMin)) {
    lastGiroTrigger = (gz + giroOffset);
  } else {
    lastGiroTrigger = 0;
  }
  swervePower = (lastGiroTrigger / (float) 32767) * (float) 2.5;
  if (swervePower > 1) {
    swervePower = 1;
  } else if (swervePower < -1) {
    swervePower = -1;
  }
}

void loop() {

  mpu.getRotation(&gx, &gy, &gz);

  // put your main code here, to run repeatedly:

  if (Serial1.available()) {
    String val = Serial1.readStringUntil('\n');
    applyNewSpeed(val);
  }
  if (Serial.available()) {
    String val = Serial.readStringUntil('\n');
    applyNewSpeed(val);
  }
  /*
    if (Serial1.available()) {
    targetSpeeds[0] = Serial1.readStringUntil('\n').toFloat();
    targetSpeeds[1] = targetSpeeds[0];
    targetSpeeds[2] = targetSpeeds[0];
    targetSpeeds[3] = targetSpeeds[0];
    }*/
   
  motor1.run(directionFromSpeed(speeds[0]));
  motor2.run(directionFromSpeed(speeds[1]));
  motor3.run(directionFromSpeed(speeds[2]));
  motor4.run(directionFromSpeed(speeds[3]));

  if ((targetSpeeds[0] == targetSpeeds[1] && targetSpeeds[2] == targetSpeeds[3] && -targetSpeeds[0] == targetSpeeds[2]) || targetSpeeds[0] == targetSpeeds[1] && targetSpeeds[1] == targetSpeeds[2] && targetSpeeds[2] == targetSpeeds[3]) {
    straightMovement = true;
  } else {
    straightMovement = false;
    motor1.setSpeed((int) speeds[0]);
    motor2.setSpeed((int) speeds[1]);
    motor3.setSpeed((int) speeds[2]);
    motor4.setSpeed((int) speeds[3]);
  }


  if (readEncoder(channel1A, lastValue1A, lastChange1A)) {
    addToCyclicArray(channelHistory1A, channelIndex1A, micros());
  }

  if (readEncoder(channel2A, lastValue2A, lastChange2A)) {
    addToCyclicArray(channelHistory2A, channelIndex2A, micros());
  }

  if (readEncoder(channel3A, lastValue3A, lastChange3A)) {
    addToCyclicArray(channelHistory3A, channelIndex3A, micros());
  }

  if (readEncoder(channel4A, lastValue4A, lastChange4A)) {
    addToCyclicArray(channelHistory4A, channelIndex4A, micros());
  }

  if (shouldReadSpeed()) {

    /*
      Serial1.print(speedHistory[0]);
      Serial1.print(":");
      Serial1.print(speedHistory[1]);
      Serial1.print(":");
      Serial1.print(speedHistory[2]);
      Serial1.print(":");
      Serial1.print(speedHistory[3]);
      Serial1.print("-");
      Serial1.print(speeds[0]);
      Serial1.print(":");
      Serial1.print(speeds[1]);
      Serial1.print(":");
      Serial1.print(speeds[2]);
      Serial1.print(":");
      Serial1.print(speeds[3]);
      Serial1.print("-");
      Serial1.print(targetSpeeds[0]);
      Serial1.print(":");
      Serial1.print(targetSpeeds[1]);
      Serial1.print(":");
      Serial1.print(targetSpeeds[2]);
      Serial1.print(":");
      Serial1.println(targetSpeeds[3]);*/

    //


    Serial.print("s1:");
    Serial.print(speedHistory[0]);
    Serial.print(",s2:");
    Serial.print(speedHistory[1]);
    Serial.print(",s3:");
    Serial.print(speedHistory[2]);
    Serial.print(",s4:");
    Serial.print(speedHistory[3]);
    Serial.print(",i1:");
    Serial.print(speeds[0]);
    Serial.print(",i2:");
    Serial.print(speeds[1]);
    Serial.print(",i3:");
    Serial.print(speeds[2]);
    Serial.print(",i4:");
    Serial.print(speeds[3]);
    Serial.print(",t1:");
    Serial.print(targetSpeeds[0]);
    Serial.print(",t2:");
    Serial.print(targetSpeeds[1]);
    Serial.print(",t3:");
    Serial.print(targetSpeeds[2]);
    Serial.print(",t4:");
    Serial.print(targetSpeeds[3]);
    Serial.print(",sp:");
    Serial.println(swervePower);
    updateSwervePower();

    speeds[0] = getUpdatedSpeed(countFromCyclicArray(channelHistory1A, channelIndex1A, micros() - ((long) readDelay * (long) 1000)), speeds, targetSpeeds, speedHistory, 0);
    speeds[1] = getUpdatedSpeed(countFromCyclicArray(channelHistory2A, channelIndex2A, micros() - ((long) readDelay * (long) 1000)), speeds, targetSpeeds, speedHistory, 1);
    speeds[2] = getUpdatedSpeed(countFromCyclicArray(channelHistory3A, channelIndex3A, micros() - ((long) readDelay * (long) 1000)), speeds, targetSpeeds, speedHistory, 2);
    speeds[3] = getUpdatedSpeed(countFromCyclicArray(channelHistory4A, channelIndex4A, micros() - ((long) readDelay * (long) 1000)), speeds, targetSpeeds, speedHistory, 3);
  }

}
