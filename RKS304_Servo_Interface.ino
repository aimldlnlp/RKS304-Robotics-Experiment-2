#include <Servo.h>

// const int buzzer = 8;
const int trig_pin = 3;
const int echo_pin = 2;
float timing = 0.0;
float distance = 0.0;

Servo myServo;

void setup() {
  pinMode(echo_pin, INPUT);
  pinMode(trig_pin, OUTPUT);
  // pinMode(buzzer, OUTPUT);

  digitalWrite(trig_pin, LOW);
  // digitalWrite(buzzer, LOW);
  myServo.attach(5);
  // moveStartTime = millis();  // start moving
  Serial.begin(9600);
}

void loop() {
  digitalWrite(trig_pin, LOW);
  delay(10);
  digitalWrite(trig_pin, HIGH);
  delay(10);
  digitalWrite(trig_pin, LOW);

  timing = pulseIn(echo_pin, HIGH);
  distance = (timing * 0.034) / 2;

  if (distance <400) {
    myServo.write(0);
  } else myServo.write(90);

  // Serial.print("Distance: ");
  Serial.println(distance);
  // Serial.print("cm");
  // Serial.print(distance / 2.54);
  // Serial.println("in");


  // if (distance <= 50) {
  // 	tone(buzzer, 500);
  // } else {
  // 	noTone(buzzer);
  // }

  delay(100);
}
