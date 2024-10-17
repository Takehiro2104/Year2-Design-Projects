// if error "1-10" occurs when uploading, disconnect reset pin and pin 12 connections

#include <NewPing.h>
#include <Servo.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "I2Cdev.h"

#define ECHO_PIN     8  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_PIN  9  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define MAX_DISTANCE 120
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
int dist;

#include <FastLED.h>
#define LED_PIN_10  A1
#define LED_PIN_11  A2
#define NUM_LEDS    8
CRGB leds_10[NUM_LEDS];
CRGB leds_11[NUM_LEDS];

const int on_off_pin = 5;
int buttonState = 0;
const int toggle_pin = 6;
int buttonState_toggle = 0;
const int reset_pin = 3; //check pin
int reset_value = 0;
const int signalout_pin = 12;


bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion quat;
VectorFloat gravity;
float ypr[3];

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

#define INTERRUPT_PIN 2

MPU6050 mpu;

Servo servo_pitch;
Servo servo_roll;

unsigned long previousMillis = 0;  // Stores the last time the function was run
const long interval = 500;        // Interval in milliseconds (5 seconds in this case)

void setup() {
  digitalWrite(signalout_pin, HIGH);
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setZAccelOffset(1450);

  if (devStatus == 0) {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  }

  servo_roll.attach(11);
  servo_pitch.attach(10);
  
  FastLED.addLeds<WS2812, LED_PIN_10, GRB>(leds_10, NUM_LEDS);
  FastLED.addLeds<WS2812, LED_PIN_11, GRB>(leds_11, NUM_LEDS);
  for (int i = 4; i < NUM_LEDS; i++) {
  leds_10[i] = CRGB(255, 0, 0);
  leds_11[i] = CRGB(0, 255, 0);
  FastLED.show();
  }

  pinMode(on_off_pin, INPUT);
  pinMode(toggle_pin, INPUT);
  pinMode(reset_pin, INPUT);
  pinMode(signalout_pin, OUTPUT);
  Serial.println("SetUp rebooted");
}

float correct;
int j = 0;


void led(){
  dist = sonar.ping_cm();
  for (int i = 0; i < (NUM_LEDS/2); i++) {
    leds_10[i] = CRGB(0, 0, 0);
    leds_11[i] = CRGB(0, 0, 0);
    }
  
  if (dist < 20 && dist != 0){
    for (int i = 0; i < (NUM_LEDS/2); i++) {
      leds_10[i] = CRGB(255, 255, 0);
      leds_11[i] = CRGB(255, 255, 0);
      }
  }
  FastLED.show();
}

void readMPU6050() {
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
    }
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();
  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    Serial.println(F("FIFO overflow!"));
  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&quat, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &quat);
    mpu.dmpGetYawPitchRoll(ypr, &quat, &gravity);

    ypr[1] = -ypr[1] * 180 / M_PI + 45; // calibration of the servo motors
    ypr[2] = ypr[2] * 180 / M_PI * -1;
    ypr[0] = ypr[0] * 180 / M_PI * -1;
    
    if (j <= 300) {
      j++;
      correct = ypr[0];
      readMPU6050();
      return;
    }
    
    ypr[0] = ypr[0] - correct;
  }

}



void loop() {
  if (!dmpReady) return;
  unsigned long currentMillis = millis();  // Get the current time

  if (currentMillis - previousMillis >= interval) {  // Check if it's time to run the function
    previousMillis = currentMillis;  
    led();
    buttonState = pulseIn(on_off_pin, HIGH, 30000);
    reset_value = pulseIn(reset_pin, HIGH, 30000);
    buttonState_toggle = pulseIn(toggle_pin, HIGH, 30000);
  }
  if (buttonState < 1500 ){
    readMPU6050();
    int pitchValue = map(ypr[1], -90, 90, 0, 180);
    int rollValue = map(ypr[2], -90, 90, 0, 180);
    int yawValue = map(ypr[0], -90, 90, 0, 180);

    if (buttonState_toggle > 1500){
      servo_pitch.write(min(180, max(0, pitchValue - 95)));      
      servo_roll.write(min(180, max(0, yawValue)));
    }
    else{
      servo_pitch.write(min(180, max(0, pitchValue)));
      servo_roll.write(min(180, max(0, rollValue)));  
    }
  }  
  if (reset_value > 1500){
    Serial.println("Rebooting System");
    delay(5);6
    digitalWrite(signalout_pin, LOW);
  }
}
