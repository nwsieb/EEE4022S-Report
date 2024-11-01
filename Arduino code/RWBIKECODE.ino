#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h> 

#define BUTTON_PIN 4 
#define IN1 9
#define IN2 10
#define IN3 11
#define IN4 12

Servo esc;         
MPU6050 mpu;  

int escPin = 5;    
const int encoderPin = 2;  
const int encoder2Pin = 3;  
const int encoder3Pin = A1;  

int throttle = 1500;  
int midThrottle = 1500;  

// For encoders
volatile int stateD2 = 0;
volatile int stateD3 = 0;
volatile int stateA1 = 0;
volatile int prevStateD2 = 0;
volatile int prevStateD3 = 0;
volatile int prevStateA1 = 0;
volatile int stateChangeCount = 0;
unsigned long startTime = 0;
unsigned long revolutionTime = 0;
int motorSpeed = 0;
int direction = 1;
int lastState1, lastState2, lastState3;
const int pulsesPerRevolution = 8; 
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 50; // milliseconds

// MPU6050 data
float rollAcc, rollGyro, roll;
float elapsedTime, currentTime, previousTime;
float alpha = 0.8;  
float gyroXfilt = 0;  
float alphaFilter = 0.8;  
float gyroXBias = 0;
float accYBias = 0;
float accZBias = 0;
int calibrationSamples = 1000; 

// Gains
int K1 = 49;
int K2 = 2.8;
int K3 = 0.01;

bool vertical = true;
int i = 0;
const int debounceDelay = 300; 
unsigned long lastDebounceTime = 0; 

void setup() {
  esc.attach(escPin);
  pinMode(BUTTON_PIN, INPUT_PULLUP); 
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // ESC Calibration
  delay(1000);
  esc.writeMicroseconds(1500);  
  delay(2000);                  

  // Wait for button press before starting serial communication
  while (digitalRead(BUTTON_PIN) == HIGH) {
    
  }

  // Initialize serial communication
  Serial.begin(9600);
  Wire.begin();

  // Initialize the MPU6050
  mpu.initialize();

  // Set gyro scale range to 500 degrees per second
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  
  // Set accelerometer scale range to 2g
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

  pinMode(encoderPin, INPUT);
  pinMode(encoder2Pin, INPUT);
  pinMode(encoder3Pin, INPUT);

  // Attach an interrupt to the encoder pins
  attachInterrupt(digitalPinToInterrupt(encoderPin), encoder, CHANGE); // For speed
  attachInterrupt(digitalPinToInterrupt(encoder2Pin), encoder2, CHANGE);  // For direction
  attachInterrupt(digitalPinToInterrupt(encoder3Pin), encoder3, CHANGE);  // For direction

  calibrateGyroscope();
  calibrateAccelerometer();
  Serial.println("Done");

  // Initialize start time
  startTime = micros();
  previousTime = millis();  
}

void loop() {
  unsigned long currentTime = millis();

  AngleAndVelocity();
  MotorSpeed();


  if (abs(roll) > 15) vertical = false;
  if (abs(roll) < 1) vertical = true;

  if (vertical){
    int pwm = constrain(K1*roll + K2*gyroXfilt + K3*motorSpeed,-500,500);
    if (abs(pwm)<100){ 
      esc.writeMicroseconds(1600);
    } else {
      throttle = (midThrottle-pwm);
      esc.writeMicroseconds(throttle);
    }
  } else {
    esc.writeMicroseconds(midThrottle);
  }

 if (currentTime >= 30000) { // Start drive motors after 30 sec
    DriveMotors();
  }
  
  // Send data to laptop
  if (currentTime - lastUpdateTime >= updateInterval) {
    Serial.print(motorSpeed); 
    Serial.print(",");
    Serial.print(roll);
    Serial.print(",");
    Serial.println(throttle);
    lastUpdateTime = currentTime;  
  }
}

