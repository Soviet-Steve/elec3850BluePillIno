#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>

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

//#define HIGH_ACCURACY
//#define LONG_RANGE

Servo wheels;
VL53L0X vcselSensor;

void fnvdIndicateLeft();
void fnvdIndicateRight();
void turnServo(int32_t angle);
void deadState();

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

  Wire.begin();
  vcselSensor.setTimeout(500);
  if(!vcselSensor.init())
  {
    Serial1.print("Sesnor initialisation failed.");
    deadState();
  }
  #if defined LONG_RANGE
    vcselSensor.setSignalRateLimit(0.1);
    vcselSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    vcselSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  #endif

  #if defined HIGH_SPEED
    vcselSensor.setMeasurementTimingBudget(20000);
  #elif defined HIGH_ACCURACY
    vcselSensor.setMeasurementTimingBudget(200000);
  #endif
  }
void loop() {
  int distance = 0;

  distance = vcselSensor.readRangeSingleMillimeters();
  if(distance < 4000)
    Serial1.println(distance);
  if (vcselSensor.timeoutOccurred()) 
  { 
    Serial1.println("TIMEOUT");
  }
  // delay(1000);
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

void deadState()
{
  Serial1.println("The dead state has been entered.");
  bool error = true;
  while(error);
}
