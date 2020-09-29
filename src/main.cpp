#define INITIALISDATION 0 //Not using, will be implemented later when making the code more robust.
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_LSM9DS1.h>

#define HEADLIGHT PA6
#define INDICATOR_LEFT PA5
#define INDICATOR_RIGHT PA7

#define HEADLIGHT_RAW PA_6

#define MOTOR_INA PB9
#define MOTOR_INB PB8

#define MOTOR_PWM PB10
#define MOTOR_PWM_RAW PB_10

#define SERVO PB11
#define SERVO_RAW PB_11

#define LSM9DS1_M_CS	PA15
#define LSM9DS1_AG_CS	PB13

#define LSM9DS1_SCK PB3
#define LSM9DS1_MISO PB4
#define LSM9DS1_MOSI PB5

#define BATTERY_PIN PA4
#define BATTERY_RAW PA_4
#define BATTERY_COVERSION 4096/3.3

#define ANGLE_CONVERSION_COEFFECIENT 800/120
#define ANGLE_CONVERSION_CONSTANT 2050

#define HARD_LEFT -60
#define HARD_RIGHT 60



//States
#define INITIALISDATION 0 //Not using, will be implemented later when making the code more robust.
#define FORWARD 1
#define OBSTACLE_AVOIDANCE 2
#define INDICATING_LEFT 3
#define INDICATING_RIGHT 4
#define TURNING_RIGHT 5
#define TURNING_LEFT 6
#define TURNING_STRAIGHT 7
#define STOP 8
#define REVERSE 9

//#define HIGH_ACCURACY
//#define LONG_RANGE
Servo wheels;
VL53L0X vcselSensor;
Adafruit_LSM9DS1 imu = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_AG_CS, LSM9DS1_M_CS);
int state = STOP;
int distance;
int vBat;
char txBuf[512];
char rxBuf[32];
int headLightbrightness;
sensors_event_t accel, mag, gyro, temp;
long timeMils = 0;


void forward();
void avoid(void);
void indicate(int direction);
void turn(int direction);
void reverse();
void turnStraight();
void stop();
void checkSensors();
void display();

void fnvdIndicateLeft();
void fnvdIndicateRight();
void turnServo(int32_t angle);
void deadState();

//void(* fnReset)(void) = 0;

void setup() {
  // delay(10000);
  Serial1.begin(115200);
  Serial.setTimeout(10);
  //  while(1){
  //    Serial.flush();
  //    Serial.print(state);
  //    //state = Serial.parseInt();
  //    Serial.readBytesUntil(';', rxBuf, 32);
  //    sscanf(rxBuf, "%d", &state);    
  // }
  //for(;;)
  Serial1.println("henlo");
  pinMode(HEADLIGHT, OUTPUT);
  pinMode(INDICATOR_LEFT, OUTPUT);
  pinMode(INDICATOR_RIGHT, OUTPUT);
  pinMode(MOTOR_INA, OUTPUT);
  pinMode(MOTOR_INB, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);;

  digitalWrite(MOTOR_INA, HIGH);
  digitalWrite(MOTOR_INB, HIGH);
  digitalWrite(HEADLIGHT, LOW);
  pwm_start(MOTOR_PWM_RAW, 50, 20, PERCENT_COMPARE_FORMAT);
  
  if(!imu.begin())
  {
    Serial.println("IMU initialisation failed.");
    //fnReset();
    deadState();
  }
  imu.setupAccel(imu.LSM9DS1_ACCELRANGE_2G);
  imu.setupMag(imu.LSM9DS1_MAGGAIN_4GAUSS);
  imu.setupGyro(imu.LSM9DS1_GYROSCALE_245DPS);


  Wire.begin();
  vcselSensor.setTimeout(500);
  if(!vcselSensor.init())
  {
    Serial1.print("VCSEL initialisation failed.");
    //fnReset();
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
  turnServo(1);
}


void loop() {
  checkSensors();
  // sscanf(txBuf, "X acceleration: %f\nY acceleration: %f\nZ acceleration: %f\n", accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);
  // Serial.printf("X acceleration: %f\nY acceleration: %f\nZ acceleration: %f\n", accel.acceleration.x, accel.acceleration.y, accel.acceleration.z); 
  if(timeMils < (long)(millis()/* - 1000*/)){
    display();
    timeMils = millis();
  }
  Serial.flush();
  Serial.readBytesUntil(';', rxBuf, 32);
  sscanf(rxBuf, "%d", &state);
  // if(distance > 300)
  //   state = FORWARD;
  // else
  //   state = OBSTACLE_AVOIDANCE;
  
  state = 0;
  switch(state)
  {
    case FORWARD:
      forward();
    break;
    case OBSTACLE_AVOIDANCE:
      avoid();
    break;
    case INDICATING_LEFT:
      indicate(INDICATING_LEFT);
    break;
    case INDICATING_RIGHT:
      indicate(INDICATING_RIGHT);
    break;
    case TURNING_LEFT:
      turn(TURNING_LEFT);
    break;
    case TURNING_RIGHT:
        turn(TURNING_RIGHT);
    break;
    case TURNING_STRAIGHT:
        turnStraight();
    break;
    case REVERSE:
        reverse();
    break;
    case STOP:
    case 0:
        stop();
    break;
  }
  
}

void forward()
{
  digitalWrite(INDICATOR_LEFT, LOW);
  digitalWrite(INDICATOR_RIGHT, LOW);
  digitalWrite(MOTOR_INA, LOW);
  digitalWrite(MOTOR_INB, HIGH);
}

void avoid(void)
{
  digitalWrite(MOTOR_INA, HIGH);
  digitalWrite(MOTOR_INB, HIGH);
}

void indicate(int direction)
{
    if(direction == INDICATING_LEFT)
    {
        digitalWrite(INDICATOR_LEFT, HIGH);
    }
    else
    {
        if(direction == INDICATING_RIGHT)
        {
            digitalWrite(INDICATOR_RIGHT, HIGH);
        }
        else
        {
            deadState();
        }
        
    }
}

void turn(int direction)
{
    if(direction == TURNING_LEFT)
    {
        turnServo(HARD_LEFT);
    }
    else
    {
        if(direction == TURNING_RIGHT)
        {
            turnServo(HARD_RIGHT);
        }
        else
        {
            deadState();
        }
        
    }
}

void turnStraight()
{
    turnServo(0);
}

void reverse()
{
    digitalWrite(HEADLIGHT, HIGH);
    digitalWrite(INDICATOR_LEFT, LOW);
    digitalWrite(INDICATOR_RIGHT, LOW);
    digitalWrite(MOTOR_INA, HIGH);
    digitalWrite(MOTOR_INB, LOW);
}

void stop()
{
    digitalWrite(INDICATOR_LEFT, LOW);
    digitalWrite(INDICATOR_RIGHT, LOW);
    digitalWrite(HEADLIGHT, LOW);
    digitalWrite(MOTOR_INA, HIGH);
    digitalWrite(MOTOR_INB, HIGH);
}

void checkSensors()
{
  imu.read();
  imu.getEvent(&accel, &mag, &gyro, &temp);
  distance = vcselSensor.readRangeSingleMillimeters();
  vBat = adc_read_value(BATTERY_RAW, 8);
}

/*
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
*/


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

void display(){
  /*
  Serial.print("X acceleration: ");
  Serial.print(accel.acceleration.x - 0.2);
  Serial.print("\t");
  Serial.print("Y acceleration: ");
  Serial.print(accel.acceleration.y);
  Serial.print("\t");
  Serial.print("Z acceleration: ");
  Serial.print((accel.acceleration.z - 9.46));
  Serial.print("\t");
  Serial.print("Gyro X: ");
  Serial.print(gyro.gyro.x);   
  Serial.print("\t");
  Serial.print("Gyro Y: ");
  Serial.print(gyro.gyro.y);
  Serial.print("\t");
  Serial.print("Gyro Z: ");
  Serial.print(gyro.gyro.z);      
  Serial.print("\t");
  */
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print("\t");
  // Serial.print("Battery Voltage: ");
  // Serial.print(vBat);
  // Serial.print("\t");
  Serial.print("State: ");
  Serial.print(state);
  Serial.print("\n");
}