void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(11, 0);
  delay(2000);


  analogWrite(11, 100);
  delay(2000);

  analogWrite(11, 150);
  delay(2000);

  analogWrite(11, 200);
  delay(2000);

  analogWrite(11, 250);
  delay(2000);

  analogWrite(11, 255);
  delay(2000);
}
