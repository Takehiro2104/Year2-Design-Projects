#include <Servo.h>

Servo servo;

int LDRPin = A2;
int servoPin = 3;
int threshold = 100;

void setup(){
  servo.attach(servoPin); //attach the servo
  pinMode(LDRPin, INPUT); //sets LDR as input
  Serial.begin(9600);
}

void loop(){
  int LDRValue = analogRead(LDRPin); //Reading value from LDR
  if (LDRValue < threshold){
      servo.write(180); //Rotate servo 180 degrees if LDR value is lower than threshold
  }else{
       servo.write(0); //return back to 0 degrees if LDR is higher than threshold
  }
  Serial.println(LDRValue);
  delay(100);
}
  
// will test tmr when I have my testing kit
// thanks Adrian for writing in the code with this empty file i opened but I think we're going with Isham's code :)
// Worked*
