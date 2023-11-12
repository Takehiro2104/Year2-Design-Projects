#include <Servo.h>

const int ldrPin = A0;   //pin for LDR
const int servoPin = 9;  //pin for servo motor

Servo jawServo;  

void setup() {
  Serial.begin(9600);
  jawServo.attach(servoPin);  // Attach the servo to the pin
}

void loop() {
  int lightIntensity = analogRead(ldrPin);  // to read light intensity from LDR
  Serial.println("Light Intensity: " + String(lightIntensity));

  // the servo angle based on light intensity
  if (lightIntensity < 500) {
    oscillateJaw();
  } else {
    openJaw();
  }
}

void openJaw() {
  jawServo.write(90);  // servo angle to open the jaw
  Serial.println("Jaw is open");
}

void oscillateJaw() {
  for (int angle = 0; angle <= 90; angle += 5) {
    jawServo.write(angle);
    delay(50);
  }

  delay(1000);  

  for (int angle = 90; angle >= 0; angle -= 5) {
    jawServo.write(angle);
    delay(50);
  }

  Serial.println("Jaw is oscillating");
}

