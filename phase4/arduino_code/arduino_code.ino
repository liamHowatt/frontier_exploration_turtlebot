#include <Servo.h>

Servo servo;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  servo.attach(9);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()){
    uint8_t angle = Serial.read();
    if (angle <= 180) {
      servo.write(angle);
    }
  }
}
