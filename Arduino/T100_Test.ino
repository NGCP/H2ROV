#include <Arduino.h>
#include <Servo.h>

Servo thruster;
int data = 0;

void setup() {
  Serial1.begin(9600);
  Serial.begin(9600);
  thruster.attach(9);

  delay(500);
  thruster.writeMicroseconds(1500);
  delay(1000);
}

void loop() {
  //thruster.writeMicroseconds(1600);
  if (Serial1.available() > 0) {
    data = Serial1.read();
    Serial.println(data);
    
    if (data == 1) {
      thruster.writeMicroseconds(1600);
      Serial.println("ONE");
      _delay_ms(100);
    }
    else if (data == 2) {
      thruster.writeMicroseconds(1400);
      Serial.println("TWO");
      _delay_ms(100);
    }
    else {
      thruster.writeMicroseconds(1500);
    }
  }
  else {
    thruster.writeMicroseconds(1500);
  }
}
