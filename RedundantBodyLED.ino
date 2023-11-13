int LDRPin1;
int LDRPin2;

void setup() {
  // put your setup code here, to run once:
  pinMode(A4, OUTPUT);
  Serial.begin(9600);
}

void loop() {

  LDRPin1 = analogRead(A0);
  LDRPin2 = analogRead(A1);

  if(LDRPin1 < 80 && LDRPin2 < 80){
    analogWrite(A4, 255);
  }else{
    analogWrite(A4,0);
  }
  Serial.println(LDRPin1);
  delay(100);
}

