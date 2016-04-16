//------------
//-----------------------
//---------------------------------
// Initializations and Definitions
//---------------------------------
//-----------------------
//------------

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead
#define GUARD_GAIN 10.0

//Intialize Kalman instances
Kalman kalmanX;
Kalman kalmanY;

//Accelerometer and Gyroscope variables
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double gyroXangle, gyroYangle; // Angle calculate using the gyro only

//Kalman and Complementary Filter Variables
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
double compAngleX, compAngleY; // Calculated angle using a complementary filter
int16_t tempRaw;
uint32_t timer;

//Angles and Offset
const float upRight = 0.0;
const float upRightOffset = 2;
const float balanceAngle = 30;
float averageAngle = 0;
float CurrentAngle;

// PID
const float Kp = 4; 
const float Ki = 1;
const float Kd = 1;
float pTerm, iTerm, dTerm, integrated_error, last_error, error;
const float K = 1.9*1.12;

// Buffer for I2C data
uint8_t i2cData[14];


//------------
//-----------------------
//------------------------------
// General Functions
//------------------------------
//-----------------------
//------------


//Intialize I2C, Accelerometer and Gyroscope
void InitSensors() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  // Set kalman and gyro starting angle
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  // atan2 outputs the value of -π to π (radians)
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else //Restrict roll to ±90deg
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  // Set the starting angles 
  kalmanX.setAngle(roll);
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}

//Update the accelerometer and gyroscope values
//including the kalman and complementary filter angles
void updateValues(){
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // atan2 outputs the value of -π to π (radians)
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else 
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  // Calculate gyro angle without any filter
  //gyroXangle += gyroXrate * dt;
  //gyroYangle += gyroYrate * dt;
  // Calculate gyro angle using the unbiased rate
  gyroXangle += kalmanX.getRate() * dt;
  gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
}

//PID Controller
void Pid(){
  error =  upRight - kalAngleY;  // 180 = level
  pTerm = Kp * error;
  integrated_error += error;
  iTerm = Ki * constrain(integrated_error, -GUARD_GAIN, GUARD_GAIN);
  dTerm = Kd * (error - last_error);
  last_error = error;
  speed = constrain(K*(pTerm + iTerm + dTerm), -255, 255);
}

//Verify the state and call Pid and Motors functions
//or the stop function
void verify_and_drive() {
  error = upRight-kalAngleY;
  averageAngle = (19*averageAngle + kalAngleY)/20;
  if (abs(error) <= upRightOffset)
  {
    stop();
  } else {
    if(abs(error) < balanceAngle){
      Pid();
      Motors();
      
    } else {
      stop();
      
    }
    
  }
}

//Print all the angles
void printAngles()
{

#if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");

  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");

  Serial.print("\t");
#endif

  //Serial.print(kalAngleX); Serial.print("\t");

  //Serial.print("\t");

  //Serial.print(kalAngleY); Serial.print("\t");

  //Serial.print("\r\n");

  Serial.print(averageAngle); Serial.print("\r\n");
}

//Print error if found
void printError()
{
  Serial.print(error); Serial.print("\r\n");
}
