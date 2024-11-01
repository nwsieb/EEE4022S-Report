void encoder() {
  stateD2 = digitalRead(2);
  stateChangeCount++;
}

void encoder2() {
  stateD3 = digitalRead(3);
  Direction();
}

void encoder3() {
  stateA1 = digitalRead(A1);
  Direction();
}

// Direction of Reaction Wheel motor
void Direction() {
  int currentState = (stateD2 << 2) | (stateD3 << 1) | stateA1;  // combining sensor states
  int prevState = (prevStateD2 << 2) | (prevStateD3 << 1) | prevStateA1;

  if ((prevState == 0b100 && currentState == 0b101) ||
      (prevState == 0b101 && currentState == 0b001) ||
      (prevState == 0b001 && currentState == 0b011) ||
      (prevState == 0b011 && currentState == 0b010) ||
      (prevState == 0b010 && currentState == 0b110) ||
      (prevState == 0b110 && currentState == 0b100)) {
    direction = 1;  // Clockwise
  } 
  else if ((prevState == 0b100 && currentState == 0b110) ||
           (prevState == 0b110 && currentState == 0b010) ||
           (prevState == 0b010 && currentState == 0b011) ||
           (prevState == 0b011 && currentState == 0b001) ||
           (prevState == 0b001 && currentState == 0b101) ||
           (prevState == 0b101 && currentState == 0b100)) {
    direction = -1;  // Counterclockwise
  } 

  // Update the previous state
  prevStateD2 = stateD2;
  prevStateD3 = stateD3;
  prevStateA1 = stateA1;
}

// Calibrate gyroscope 
void calibrateGyroscope() {
  long gyroXSum = 0;

  // Get average 
  for (int i = 0; i < calibrationSamples; i++) {
    gyroXSum += mpu.getRotationX();
    delay(3);
  }

  // Calculate the average bias
  gyroXBias = (gyroXSum / calibrationSamples) / 65.5;  // 65.5 is the sensitivity scale factor
}

// Calibrate accelerometer bias
void calibrateAccelerometer() {
  long accYSum = 0;
  long accZSum = 0;

  // Get average
  for (int i = 0; i < calibrationSamples; i++) {
    accYSum += mpu.getAccelerationY();
    accZSum += mpu.getAccelerationZ();
    delay(3); 
  }

  // Calculate the average bias
  accYBias = accYSum / (float)calibrationSamples / 16384;  // Converting to g value
  accZBias = (accZSum / (float)calibrationSamples / 16384) - 1;
}

void AngleAndVelocity() {
  int16_t accY, accZ, gyroX;

  // Read accelerometer and gyroscope data
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();
  gyroX = mpu.getRotationX();

  // Convert accelerometer data to g values and minus bias
  float accYf = (accY / 16384) - accYBias;
  float accZf = (accZ / 16384) - accZBias;

  // Convert gyroscope data to degrees per second and minus bias
  float gyroXf = (gyroX / 65.5) - gyroXBias; 

  // Accelerometer roll angle (degrees)
  rollAcc = atan2(accYf, accZf) * 180 / PI;

  // Time values needed for gyroscope calculations
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000; 
  previousTime = currentTime;

  // Gyroscope roll angle (degrees)
  rollGyro += gyroXf * elapsedTime;

  // Combine accelerometer and gyroscope data using complementary filter
  roll = alpha * rollGyro + (1 - alpha) * rollAcc;

  // Apply low pass filter to gyroscope data
  gyroXfilt = alphaFilter * gyroXf + (1 - alphaFilter) * gyroXfilt;
}

void MotorSpeed() {
  // 8 state changes are detected
  if (stateChangeCount >= pulsesPerRevolution) {
    // Time per revolution
    revolutionTime = micros() - startTime;

    motorSpeed = (1000000 / revolutionTime) * 60;  
    motorSpeed *= direction;
    // Reset count and start time
    stateChangeCount = 0;
    startTime = micros();

  }
}

void DriveMotors() {
  digitalWrite(IN1, LOW); 
  analogWrite(IN2, 150); 
  digitalWrite(IN3, LOW); 
  analogWrite(IN4, 150); 
}
