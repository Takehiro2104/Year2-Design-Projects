// Define Trig and Echo pin:
#define trigPin 3
#define echoPin 2
#define led1pin 4 //Red
#define led2pin 5 //Yellow
#define led3pin 6 //Green
#define led4pin A4 //Blue 
#include <Servo.h>
#include <Stepper.h>

Servo servo;
const int stepsPerRevolution = 2048;
Stepper myStepper = Stepper(stepsPerRevolution, 13,11,12,10); // 1 3 2 4

int LDRPin = A2; //LDR FOR MOUTH
int servoPin = 7;
long duration;
int distance;
int LDRPin1 = A0; //LDR FOR HEAD
int LDRPin2 = A1; //LDR FOR TAIL

void setup(){
  servo.attach(servoPin); //attach the servo
  pinMode(LDRPin, INPUT); //sets LDR as input
  pinMode(LDRPin1, INPUT);
  pinMode(LDRPin2, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(led1pin, OUTPUT);
  pinMode(led2pin, OUTPUT);
  pinMode(led3pin, OUTPUT);
  pinMode(A4, OUTPUT);
  Serial.begin(9600);
}

void loop(){
  functionA();
  functionB();
  functionC();
}

void functionA (){
int LDRPinMouth = analogRead(LDRPin); //Reading value from LDR
  if (LDRPinMouth < 160){
      servo.write(40); //Rotate servo 180 degrees if LDR value is lower than threshold
  }else{
       servo.write(0); //return back to 0 degrees if LDR is higher than threshold
  }
  Serial.print("LDRMouth: ");
  Serial.println(LDRPinMouth);
  delay(150);
}

void functionB (){
  // Clear the trigPin by setting it LOW:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);

  // Trigger the sensor by setting the trigPin high for 10 microseconds:
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echoPin, pulseIn() returns the duration (length of the pulse) in microseconds:
  duration = pulseIn(echoPin, HIGH);
  // Calculate the distance:
  distance = duration * 0.034 / 2;


  // Print the distance on the Serial Monitor (Ctrl+Shift+M):
  Serial.print("Distance = ");
  Serial.print(distance);
  Serial.println(" cm");
  myStepper.setSpeed(10);
  // LED light up  due to distance 
  // Run time same as function needs to called every loop.
  if (distance < 15){
    myStepper.step(256);
    myStepper.setSpeed(200);
    digitalWrite(led1pin, HIGH); 
    digitalWrite(led2pin, LOW);
    digitalWrite(led3pin, LOW);
    
  }
  else if (distance < 50){
    myStepper.step(-256);
    myStepper.setSpeed(200);
    digitalWrite(led2pin, HIGH);
    digitalWrite(led3pin, LOW);
    digitalWrite(led1pin, LOW);
  }

  else {
    myStepper.step(-512);
    myStepper.setSpeed(200); 
    digitalWrite(led3pin, HIGH);
    digitalWrite(led1pin, LOW);
    digitalWrite(led2pin, LOW);
  }
}
void functionC(){
  LDRPin1 = analogRead(A0);
  LDRPin2 = analogRead(A1);
  if(LDRPin1 < 50 && LDRPin2 < 50){
    analogWrite(A4, 255);
  }else{
    analogWrite(A4,0);
  }
  Serial.print("LDR1: ");
  Serial.println(LDRPin1);
  Serial.print("LDR2: ");
  Serial.println(LDRPin2);
  delay(150);
}
