/* Example code for HC-SR04 ultrasonic distance sensor with Arduino. No library required. More info: https://www.makerguides.com */

// Define Trig and Echo pin:
#define trigPin 2
#define echoPin 3
#define led1pin 4 //Red
#define led2pin 5 //Yellow
#define led3pin 6 //Green


// Define variables:
long duration;
int distance;


void setup() {
  // Define inputs and outputs:
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(led1pin, OUTPUT);
  pinMode(led2pin, OUTPUT);
  pinMode(led3pin, OUTPUT);
  //Begin Serial communication at a baudrate of 9600:
  Serial.begin(9600);
}

void loop() {
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

  // LED light up  due to distance 
  // Run time same as function needs to called every loop.
  if (distance < 15){
    analogWrite(led1pin, 255); 
    analogWrite(led2pin, 0);
    analogWrite(led3pin, 0);
    
  }
  else if (distance < 30){
    analogWrite(led2pin, 255);
    analogWrite(led3pin, 0);
    analogWrite(led1pin, 0);
  }
  else{
    analogWrite(led3pin, 255);
    analogWrite(led1pin, 0);
    analogWrite(led2pin, 0);
  }

}
