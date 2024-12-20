#include <Arduino.h>
#include <ESP32Servo.h>
#include <VL53L1X.h>
#include <Wire.h>
VL53L1X sensor;
Servo servo;  // create four servo objects
const int servoPin = D3;

// Published values for SG90 servos; adjust if needed
int minUs = 620;
int maxUs = 2400;

long anaVal0, anaVal1, anaVal2;

int pos;  // position in degrees

float kp;
float kd;
float ki;

int dt;
int preTime;

int val;
int targetVal = 160;

int x;
float intX;
int preX;
int v;

void setup() {
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);

  servo.setPeriodHertz(50);  // Standard 50hz servo
  servo.attach(servoPin, minUs, maxUs);
  servo.write(90);

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);  // use 400 kHz I2C

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);
  sensor.startContinuous(50);

  delay(5000);
}

void loop() {
  for (int i = 0; i < 100; i++) {
    anaVal0 += analogRead(A2);
    anaVal1 += analogRead(A1);
    anaVal2 += analogRead(A0);
  }
  anaVal0 = anaVal0 / 100;
  anaVal1 = anaVal1 / 100;
  anaVal2 = anaVal2 / 100 - 2048;

  kp = anaVal0 * 0.001;
  kd = anaVal1 * 0.001;
  ki = anaVal2 * 0.0005;

  if (sensor.timeoutOccurred()) {
    Serial.print(" TIMEOUT");
  }
  Serial.println();

  val = sensor.read();
  if (val > 240 || sensor.timeoutOccurred()) {
    val = 0;
  }

  // dt = millis() - preTime;
  // preTime = millis();

  x = val - targetVal;
  v = x - preX;
  intX += x * 0.0001;
  preX = x;

  pos = 90 + kp * x + kd * v + ki * intX;

  if (pos < 0 || pos > 180) {
    pos = 90;
  }

  servo.write(pos);

  Serial.printf(">val:%d\n", val);
  Serial.printf(">x:%d\n", x);
  Serial.printf(">v:%d\n", v);
  Serial.printf(">kp:%.2f\n", kp);
  Serial.printf(">kd:%.2f\n", kd);
  Serial.printf(">ki:%.2f\n", ki);
  Serial.printf(">pos:%d\n", pos);
  Serial.printf(">intX:%.2f\n", intX);
}