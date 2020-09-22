#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>

#define HEADLIGHT PA6
#define INDICATE_LEFT PA5
#define INDICATE_RIGHT PA7

#define HEADLIGHT_RAW PA_6

#define MOTOR_INA PB9
#define MOTOR_INB PB8

#define MOTOR_PWM PB10
#define MOTOR_PWM_RAW PB_10

#define SERVO PB11
#define SERVO_RAW PB_11

#define BATTERY_PIN PA4
#define BATTERY_COVERSION 4096/3.3

#define ANGLE_CONVERSION_COEFFECIENT 800/120
#define ANGLE_CONVERSION_CONSTANT 2050

#define HARD_LEFT -60



//States
#define INITIALISDATION 0 //Not using, will be implemented later when making the code more robust.
#define FORWARD 1
#define OBSTACLE_AVOIDANCE 2
#define INDICATING 3
#define TURNING 4
#define STOP 5
#define REVERSE 6

//#define HIGH_ACCURACY
//#define LONG_RANGE

Servo wheels;
VL53L0X vcselSensor;
int state;
int distance;
int vBat;
char txBuf[512];
int headLightbrightness;

void forward();
void avoid(void);
void indicate();
void turn();
void reverse();
void checkSensors();

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
  digitalWrite(MOTOR_INB, HIGH);
  digitalWrite(HEADLIGHT, HIGH);
  pwm_start(MOTOR_PWM_RAW, 50, 20, PERCENT_COMPARE_FORMAT);
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

  state = FORWARD;
  analogReadResolution(12);
}


void loop() {
  // digitalToggle(HEADLIGHT);
  if(distance > 200)
    state = FORWARD;
  else
    state = avoid;
  
  switch(state)
  {
    case FORWARD:
      forward();
    break;
    case OBSTACLE_AVOIDANCE:
      avoid();
    break;
    case INDICATING:
      indicate();
    break;
    case TURNING:
      turn();
    break;
    case REVERSE:
      reverse();
    break;
  }

  checkSensors();
  sprintf(txBuf, "DisF: %d Vbat: %d \n", distance, (vBat));
  Serial.printf(txBuf);
  
}

void forward()
{
  digitalWrite(INDICATE_LEFT, LOW);
  digitalWrite(MOTOR_INA, LOW);
  digitalWrite(MOTOR_INB, HIGH);
}

void avoid(void)
{
  digitalWrite(MOTOR_INA, HIGH);
  digitalWrite(MOTOR_INB, HIGH);
}

void indicate()
{
  digitalWrite(INDICATE_LEFT, HIGH);
}

void turn()
{
  turnServo(HARD_LEFT);
}

void reverse()
{
  digitalWrite(MOTOR_INA, LOW);
  digitalWrite(MOTOR_INB, HIGH);
}

void checkSensors()
{
  distance = vcselSensor.readRangeSingleMillimeters();
  vBat = analogRead(BATTERY_PIN);
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
