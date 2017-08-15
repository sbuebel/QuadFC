#include <Servo.h>
Servo motor;

void setup() {
  motor.attach(4);
  // put your setup code here, to run once:
  for (int i = 0; i < 1000; i ++) {
    motor.writeMicroseconds(1000);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  motor.writeMicroseconds(1100);
}
