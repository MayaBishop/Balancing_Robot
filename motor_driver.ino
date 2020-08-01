// motor one
int spd1 = 10;
int fwd1 = 9;
int rev1 = 8;
// motor two
int spd2 = 5;
int fwd2 = 7;
int rev2 = 6;
void setup() {
  pinMode(spd1, OUTPUT);
  pinMode(spd2, OUTPUT);
  pinMode(fwd1, OUTPUT);
  pinMode(rev1, OUTPUT);
  pinMode(fwd2, OUTPUT);
  pinMode(rev2, OUTPUT);

}

void moveForward() {
  digitalWrite(fwd1, HIGH);
  digitalWrite(rev1, LOW);
  digitalWrite(fwd2, HIGH);
  digitalWrite(rev2, LOW);
  for (int i = 0; i < 256; i++) {
    analogWrite(spd1, i);
    analogWrite(spd2, i);
    delay(200);
  }
  for (int i = 255; i > 0; i-) {
    analogWrite(spd1, i);
    analogWrite(spd2, i);
    delay(200);
  }
  
  digitalWrite(fwd1, LOW);
  digitalWrite(rev1, LOW);
  digitalWrite(fwd2, LOW);
  digitalWrite(rev2, LOW);
}

void moveBackward() {
  digitalWrite(fwd1, LOW);
  digitalWrite(rev1, HIGH);
  digitalWrite(fwd2, LOW);
  digitalWrite(rev2, HIGH);
  for (int i = 0; i < 256; i++) {
    analogWrite(spd1, i);
    analogWrite(spd2, i);
    delay(200);
  }
  for (int i = 255; i > 0; i-) {
    analogWrite(spd1, i);
    analogWrite(spd2, i);
    delay(200);
  }
  
  digitalWrite(fwd1, LOW);
  digitalWrite(rev1, LOW);
  digitalWrite(fwd2, LOW);
  digitalWrite(rev2, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  moveForward();
}
