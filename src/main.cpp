#include <Arduino.h>

#define HEADLIGHT PA6
#define INDICATE_LEFT PA5
#define INDICATE_RIGHT PA7

#define MOTOR_INA PB9
#define MOTOR_INB PB8
#define MOTOR_PWM PB10

void fnvdIndicateLeft();
void fnvdIndicateRight();

void setup() {
  pinMode(HEADLIGHT, OUTPUT);
  pinMode(INDICATE_LEFT, OUTPUT);
  pinMode(INDICATE_RIGHT, OUTPUT);
  pinMode(MOTOR_INA, OUTPUT);
  pinMode(MOTOR_INB, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);

  digitalWrite(MOTOR_INA, HIGH);
  digitalWrite(MOTOR_INB, LOW);
}

void loop() {
  digitalToggle(MOTOR_PWM);
  delay(1000);
}

void fnvdIndicateLeft(){
  for(uint8_t i = 0; i < 6; i++){
    digitalToggle(INDICATE_LEFT);
    delay(500);
  }
}

void fnvdIndicateRight(){
  for(uint8_t i = 0; i < 6; i++){
    digitalToggle(INDICATE_RIGHT);
    delay(500);
  }
}

