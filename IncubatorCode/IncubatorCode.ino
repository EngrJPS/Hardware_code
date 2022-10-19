void setup() {
  Serial.begin(9600);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

}

void loop() {

  //Turn for coil A
  for(int i = 0; i < 7; i++){
    coilA();
  }

  delay(3600000);

  //Turn for coil B
  for(int i = 0; i < 7; i++){
    coilB();
  }

  delay(3600000);

void coilA(){
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(7, HIGH);
  Serial.println("LED-On-A");
  delay(2000);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("LED-Off-A");
  digitalWrite(7, LOW);
  delay(2000);
}

void coilB(){
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(8, HIGH);
  Serial.println("LED-On-B");
  delay(2000);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(8, LOW);
  Serial.println("LED-Off-B");
  delay(2000);
}