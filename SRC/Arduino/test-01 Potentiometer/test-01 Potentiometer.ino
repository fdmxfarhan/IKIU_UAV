int volt = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  volt = map(analogRead(A0), 0, 1023, 128, 255);
  Serial.println(volt);
  analogWrite(5, volt);
  analogWrite(6, volt);
  analogWrite(9, volt);
  analogWrite(10, volt);
}
