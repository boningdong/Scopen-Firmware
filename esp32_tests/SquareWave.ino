


void setup() {
  pinMode(14, OUTPUT);
  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);
}

void loop() {
   digitalWrite(14, HIGH);
   delay(20);
   digitalWrite(14, LOW);
   delay(20);
}
