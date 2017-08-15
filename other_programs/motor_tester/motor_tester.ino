#include <Servo.h>
  Servo m_4;

void setup() {
  // put your setup code here, to run once:
  m_4.attach(7);
  for (int i = 0; i < 2000; i ++) {
    m_4.writeMicroseconds(1000);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  m_4.writeMicroseconds(1200);
}
