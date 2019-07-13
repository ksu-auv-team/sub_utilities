void setup() {
  Serial.begin(9600);
  pinMode(4, INPUT);
}

void loop() {
  if(digitalRead(4)){
    Serial.write("1");
  }
  else{
    Serial.write("0");
  }
}
