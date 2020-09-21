#include <Arduino.h>
#include <Servo.h>

#define HEADLIGHT PA6
#define INDICATE_LEFT PA5
#define INDICATE_RIGHT PA7

#define MOTOR_INA PB9
#define MOTOR_INB PB8

#define MOTOR_PWM PB10
#define MOTOR_PWM_RAW PB_10

#define SERVO PB11
#define SERVO_RAW PB_11

#define ANGLE_CONVERSION_COEFFECIENT 800/120
#define ANGLE_CONVERSION_CONSTANT 2050

Servo wheels;

void fnvdIndicateLeft();
void fnvdIndicateRight();

void turnServo(int32_t angle);


void setup() {
  Serial1.begin(115200);
  //for(;;)
    Serial1.println("henlo");
  pinMode(HEADLIGHT, OUTPUT);
  pinMode(INDICATE_LEFT, OUTPUT);
  pinMode(INDICATE_RIGHT, OUTPUT);
  pinMode(MOTOR_INA, OUTPUT);
  pinMode(MOTOR_INB, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);;

  digitalWrite(MOTOR_INA, HIGH);
  digitalWrite(MOTOR_INB, LOW);
  digitalWrite(HEADLIGHT, HIGH);
  /*
  wheels.attach(SERVO);
  wheels.write(90);
  wheels.detach();
  */
  //pwm_start(PB_)
  //pwm_start(MOTOR_PWM_RAW, 50, 50, PERCENT_COMPARE_FORMAT);
  //pwm_start(SERVO_RAW, 50, 0, MICROSEC_COMPARE_FORMAT);
  /*
  for(int i = 1650; i <= 2450; i += 1)
  {
    pwm_start(SERVO_RAW, 50, i, MICROSEC_COMPARE_FORMAT);
    delay(10);   
  }
  */
  }
void loop() {
  String input = Serial.readString();
  int angle;
  sscanf(input.c_str(), "%d", &angle); 
  turnServo(angle);
  Serial.println(angle);
  //turnServo((int32_t)Serial.parseInt());
  /*
  for(uint8_t i = 0; i < 100; i++){
    // pwm_start(MOTOR_PWM_RAW, 50, i, PERCENT_COMPARE_FORMAT);
    delay(120 - i);
    pwm_start(SERVO_RAW, 50, 1600, MICROSEC_COMPARE_FORMAT);
    
  }
  */
 //pwm_start(SERVO_RAW, 50, 1600, MICROSEC_COMPARE_FORMAT);
 /*
 for(int i = 0; i < 10000; i += 100)
 {
   Serial.println(i);
   pwm_start(SERVO_RAW, 50, i + 1, MICROSEC_COMPARE_FORMAT);
   delay(1000);
 }
 *//*
  pwm_start(SERVO_RAW, 50, 1600, MICROSEC_COMPARE_FORMAT);
  delay(1000);
  pwm_start(SERVO_RAW, 50, 1500, MICROSEC_COMPARE_FORMAT);
delay(1000);
  pwm_start(SERVO_RAW, 50, 1400, MICROSEC_COMPARE_FORMAT);
  delay(1000);
  pwm_start(SERVO_RAW, 50, 1200, MICROSEC_COMPARE_FORMAT);
  delay(1000);
  pwm_start(SERVO_RAW, 50, 1000, MICROSEC_COMPARE_FORMAT);
  delay(1000);
  pwm_start(SERVO_RAW, 50, 100, MICROSEC_COMPARE_FORMAT);*/
  // pwm_stop(SERVO_RAW);
  
  //Serial.println("henlo");
  /*
  wheels.attach(SERVO);
  wheels.write(90);
  wheels.detach();
  delay(10000);
  */

  /*
  digitalToggle(MOTOR_PWM);
  digitalToggle(HEADLIGHT);
  fnvdIndicateRight();
  digitalToggle(HEADLIGHT);
  fnvdIndicateLeft();
  */

 /*
  for(int i = 0; i <= 100; i += 0.5)
  {
    analogWrite(MOTOR_PWM, i);
  }
  delay(1000);
  for(int i = 100; i >= 0; i -= 0.5)
  {
    analogWrite(MOTOR_PWM, i);
  }
  */
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

void turnServo(int32_t angle)
{
  if(angle < -60 || angle > 60)
  {
    angle = 0;
  }
  //1650 - 2450
  //-60 - 60
  int value = (angle * ANGLE_CONVERSION_COEFFECIENT) + ANGLE_CONVERSION_CONSTANT;
  pwm_start(SERVO_RAW, 50, value, MICROSEC_COMPARE_FORMAT);
}