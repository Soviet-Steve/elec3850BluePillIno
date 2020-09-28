#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_LSM9DS1.h>

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

#define LSM9DS1_M_CS	PA15
#define LSM9DS1_AG_CS	PB13

#define LSM9DS1_SCK PB3
#define LSM9DS1_MISO PB4
#define LSM9DS1_MOSI PB5

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
#define TURNING_RIGHT 4
#define TURNING_LEFT 5
#define TURNING_STRAIGHT 6
#define STOP 7
#define REVERSE 8

//#define HIGH_ACCURACY
//#define LONG_RANGE
Servo wheels;
VL53L0X vcselSensor;
Adafruit_LSM9DS1 imu = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_AG_CS, LSM9DS1_M_CS);
int state;
int distance;
int vBat;
char txBuf[512];
int headLightbrightness;
sensors_event_t accel, mag, gyro, temp;
long timeMils = 0;


void forward();
void avoid(void);
void indicate();
void turn();
void reverse();
void checkSensors();
void display();

void fnvdIndicateLeft();
void fnvdIndicateRight();
void turnServo(int32_t angle);
void deadState();

//void(* fnReset)(void) = 0;

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
  digitalWrite(HEADLIGHT, LOW);
  pwm_start(MOTOR_PWM_RAW, 50, 90, PERCENT_COMPARE_FORMAT);
  
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
  analogReadResolution(12);
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
  
  // if(distance > 300)
  //   state = FORWARD;
  // else
  //   state = OBSTACLE_AVOIDANCE;
  
  // switch(state)
  // {
  //   case FORWARD:
  //     forward();
                                      //   break;
  //   case OBSTACLE_AVOIDANCE:
  //     avoid();
  //   break;
  //   case INDICATING:
  //     indicate();
  //   break;
  //   case TURNING:
  //     turn();
  //   break;
  //   case REVERSE:
  //     reverse();
  //   break;
  // }

  // 
  // sprintf(txBuf, "DisF: %d Vbat: %d \n", distance, (vBat));
  // Serial.printf(txBuf);
  
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
  imu.read();
  imu.getEvent(&accel, &mag, &gyro, &temp);
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

void display(){
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
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print("\t");
  Serial.print("Battery Voltage: ");
  Serial.print(vBat);
  Serial.print("\n");
}