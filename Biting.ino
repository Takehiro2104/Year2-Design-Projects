
#include <Servo.h>

Servo servo;

int LDRPin = A0;
int servoPin = 2;
int threshold = 500;

void setup(){
  servo.attach(servoPin); //attach the servo
  pinMode(LDRPin, INPUT); //sets LDR as input
}

void loop(){
  int LDRValue = analogRead(LDRPin); //Reading value from LDR
  if (LDRValue < threshold){
      servo.write(180) //Rotate servo 180 degrees if LDR value is lower than threshold
  }else{
       servo.write(0); //return back to 0 degrees if LDR is higher than threshold
  }
}
  
// will test tmr when I have my testing kit
// thanks Adrian for writing in the code with this empty file i opened but I think we're going with Isham's code :)
