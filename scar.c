
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

#define RH_MOTOR_A 11
#define RH_MOTOR_B 10
#define RH_MOTOR_EN 9
#define LH_MOTOR_A 8
#define LH_MOTOR_B 7
#define LH_MOTOR_EN 6

#define F_MOTOR_A 3
#define F_MOTOR_B 4
#define F_MOTOR_EN 5 

#define RH_ENCODER_A 2
#define RH_ENCODER_B 4
#define LH_ENCODER_A 5
#define LH_ENCODER_B 3

volatile unsigned long leftCount = 0;
volatile unsigned long rightCount = 0;
int ERROR_CONSTANT_P = 6; //P = 6; D = 1
int ERROR_CONSTANT_I = 0;
int ERROR_CONSTANT_D = 1;
int ERROR_CONSTANT_Pe = 1; //P=6
int ERROR_CONSTANT_Ie = 6;
int ERROR_CONSTANT_De = 2;
int SPEED = 250;
static float errorEncoder1,errorEncoder2;

void leftEncoderEvent(void); 
void move_forward(void);
void rightEncoderEvent(void);
void counter(void);
void displaySensorDetails(void);


float a, b, c, temp;
int steps = 0;
int cur = 0;
int prev = 0;
bool minDeg = b > 10.00;
bool maxDeg = b < 50.00;
enum state{REST, CLIMB};


void setup() {
  //intialize encoders
  pinMode(LH_ENCODER_A, INPUT);
  pinMode(LH_ENCODER_B, INPUT);
  pinMode(RH_ENCODER_A, INPUT);
  pinMode(RH_ENCODER_B, INPUT);  
  // put your setup code here, to run once:
  pinMode(RH_MOTOR_A, OUTPUT);
  pinMode(RH_MOTOR_B, OUTPUT);
  pinMode(LH_MOTOR_A, OUTPUT);
  pinMode(LH_MOTOR_B, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(LH_ENCODER_B), leftEncoderEvent, CHANGE); //replaced leftEncoderEvent
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, CHANGE);  

  //Serial.begin(9600);

  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);
  /* Display some basic information on this sensor */
  displaySensorDetails();
}

void loop() {
  move_forward(); // put your main code here, to run repeatedly:
  //leftEncoderEvent();
  //Serial.print(leftCount);
  //Serial.print("\t");
  //Serial.println(rightCount);

  sensors_event_t event;
  bno.getEvent(&event);
  
  a = (float)event.orientation.x;
  b = (float)event.orientation.y;
  c = (float)event.orientation.z;
  
  Serial.print(F("Orientation: "));
  Serial.print((float)event.orientation.x);
  Serial.print(F(" "));
  Serial.print((float)event.orientation.y);
  Serial.print(F(" "));
  Serial.println((float)event.orientation.z);
  
  counter();

  if (cur != prev) {
    if (cur == CLIMB) {
      steps++;
    }
  }
  prev = cur;

  Serial.println(F("\n"));

  /* Also send calibration data for each sensor. */
  uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.print(F("Calibration: "));
  Serial.print(sys, DEC);
  Serial.print(F(" "));
  Serial.print(gyro, DEC);
  Serial.print(F(" "));
  Serial.print(accel, DEC);
  Serial.print(F(" "));
  Serial.println(mag, DEC);

  delay(BNO055_SAMPLERATE_DELAY_MS);

  
}

void move_forward() {
  //int motor_speed = 200;

  digitalWrite(RH_MOTOR_A, HIGH);
  digitalWrite(RH_MOTOR_B, LOW);
  digitalWrite(LH_MOTOR_B, HIGH);
  digitalWrite(LH_MOTOR_A, LOW);

  digitalWrite(F_MOTOR_A, HIGH);
  digitalWrite(F_MOTOR_B, LOW);

  analogWrite(F_MOTOR_EN, SPEED);
  analogWrite(LH_MOTOR_EN, SPEED);
    analogWrite(RH_MOTOR_EN, SPEED);
}

// encoder event for the interrupt call
void leftEncoderEvent() {
    leftCount++;
    //Serial.print("left encoder");
}
// encoder event for the interrupt call
void rightEncoderEvent() {
  rightCount++; //begin PID for encoder ticks
}

void encoderPID(){
  digitalWrite(RH_MOTOR_A, HIGH);
  digitalWrite(RH_MOTOR_B, LOW);
  digitalWrite(LH_MOTOR_B, HIGH);
  digitalWrite(LH_MOTOR_A, LOW);
    
  if (rightCount < leftCount){//speed right motor
    errorEncoder1 = (leftCount - rightCount);//error constant for speeding motor
    errorEncoder2 = 0; 
  }
  if (rightCount > leftCount){ //slow right motor
    errorEncoder2 = (rightCount - leftCount); //error constant for slowing motor
    errorEncoder1 = 0;    
  }
  int driveR = SPEED*ERROR_CONSTANT_Pe + errorEncoder1*ERROR_CONSTANT_Ie - errorEncoder2*ERROR_CONSTANT_Ie; //variable we are adjusting
  int driveL = SPEED; //reference, desired variable
    if((driveR) >= 255) { //cannot exceed systems capabilities
    driveR = 255; //max
  }
  if((driveR) <= 5) { 
    driveR = 5;
  }
  analogWrite(RH_MOTOR_EN, driveR);
  analogWrite(LH_MOTOR_EN, driveL);
  Serial.print(leftCount);
  Serial.print("\t");
  Serial.println(rightCount);
}

void counter() { 
  if ((b > 10.00) && (b < 50.00)) {
    cur = CLIMB;
  } else {
    cur = REST;
  }
  Serial.println(steps);
  Serial.println(cur);
}

void displaySensorDetails(void){
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

