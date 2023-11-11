int LDRPin1;
int LDRPin2;

void setup() {
  // put your setup code here, to run once:
  pinMode(8, OUTPUT);
  Serial.begin(9600);
}

void loop() {

  LDRPin1 = analogRead(A0);
  LDRPin2 = analogRead(A1);

  if(LDRPin1 <60 && LDRPin2 <60){
    digitalWrite(8, HIGH);
  }else{
    digitalWrite(8,LOW);
  }
  Serial.println(LDRPin1);
  delay(100);

}
