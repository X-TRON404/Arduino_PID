
#include <Wire.h>
#include <Servo.h> //incude servo library to control the speeds of ECSs
#include <math.h>

Servo right_prop;//make a servo class with name right_prop for right ESC
Servo left_prop;//make a servo class with name right_prop for right ESC

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

//caliberation variables
double rotX_,rotY_,rotZ_,gforceX,gforceY,gforceZ;
double offset_rotX,offset_rotY, offset_rotZ,offset_gforceX,offset_gforceY,offset_gforceZ;
//calculated offset
float tru_offset_gy_x=1.08,tru_offset_gy_y=-0.45,tru_offset_gy_z=0.29,tru_offset_gForceX=0.05,tru_offset_gForceZ=0.05;
;

int16_t iterations=0;

double rad_to_deg=180/3.141592654;
float input,output,lastError,error,intrgralError,diffrentialError;
float setPoint=0;
const float gyro_sensetivity_scl_fctr=131.0;//units LSB/(°/sec) for full scale range of ±250 °/sec
const float accel_sensetivity_scl_fctr=16384.0;//units LSB/(LSB/g) for full scale range of ±2g
float currentTime,previousTime,elapsedTime;
float PID, pwmLeft, pwmRight;
float  accelerationAngle_Y,totalAngle;
float throttle=1300;//initial value of throttle to motors

//=============================================================
//                     PID constants
//=============================================================
double Kp=1;
double Ki=1;
double Kd=1;
//=============================================================
//                      SETUP
//=============================================================
void setup() {
  Serial.begin(250000);
  Wire.begin();
  right_prop.attach(3); //attatch the right motor to pin 3
  left_prop.attach(5);  //attatch the left motor to pin 5
  currentTime=millis(); //time at the start of the program
  left_prop.writeMicroseconds(1000); // to start ESCs minimum value of PWM is 1000 micro seconds
  right_prop.writeMicroseconds(1000);// to start ESCs minimum value of PWM is 1000 micro seconds
  setupMPU();
}

void setupMPU(){
  Wire.beginTransmission(0b1101000); //// I2C address of the MPU6050 for AD0 low
  Wire.write(0x6B); //For accessing the power management register 6B of the MPU6050 
  Wire.write(0b00000000);//for waking up the MPU, beacause once powered up it goes in sleep mode
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); // I2C address of the MPU6050 for AD0 low
  Wire.write(0x1B); //This is the address of the gyroscope register to trigger self test 
  Wire.write(0x00000000); //Set gyro to full scale ±250 °/sec rotation
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); // I2C address of the MPU6050 for AD0 low
  Wire.write(0x1C);//This is the address of the accelerometer register to trigger self test
  Wire.write(0b00000000); //Set acceleromter to full scale acceleration of +-2g
  Wire.endTransmission(); 
}

//=============================================================
//                ACCELEROMETER
//=============================================================
void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6);//request 6 registers from MPU 6050  accel registers 3B-40
  while(Wire.available() < 6);//MPU may send less than requested

  /*As MPU 6050 returns 16 bit data but .read() only reads 8 bit at a time.Hence we add the two 8 bit data to get the 16 bit value and we store
Essentially it is multiplying the first 8 bit data by 256 and (left shift by 8) and adding the next 8 bits to it.
*/
   
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processAccelData();
}

void processAccelData(){
  gForceX = accelX / accel_sensetivity_scl_fctr-tru_offset_gForceX;// sensetivity value=16384.0
  gForceY = accelY / accel_sensetivity_scl_fctr; 
  gForceZ = accelZ / accel_sensetivity_scl_fctr+tru_offset_gForceZ;
}
//=============================================================
//                 Gyroscpoe
//=============================================================
void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //request 6 registers from MPU 6050  accel registers 3B-40
  while(Wire.available() < 6);//MPU may send less than requested
  
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processGyroData();
}

void processGyroData() {
  rotX = gyroX / gyro_sensetivity_scl_fctr-tru_offset_gy_x;// sensetivity value=131.0
  rotY = gyroY / gyro_sensetivity_scl_fctr-tru_offset_gy_y; 
  rotZ = gyroZ / gyro_sensetivity_scl_fctr-tru_offset_gy_z;
}
////=============================================================
////               Caliberation
////=============================================================
//void offsetCorrection(){
//  rotX_+=rotX;
//  rotY_+=rotY;
//  rotZ_+=rotZ;
//  gforceX+=gForceX;
//  gforceY+=gForceY;
//  gforceZ+=gForceZ;
//
//  offset_rotX=rotX_/iterations;
//  offset_rotY=rotY_/iterations;
//  offset_rotZ=rotZ_/iterations;
//  offset_gforceX=gForceX/iterations;
//  offset_gforceY=gForceY/iterations;
//  offset_gforceZ=gForceZ/iterations;
//}


//=============================================================
//                 PID
//=============================================================
void pidLoop(){
    
  previousTime = currentTime;// current time is defined in setup
  currentTime=millis();
  elapsedTime=(currentTime-previousTime)/1000;
  error = totalAngle- setPoint;

  /*Here we have considered IMU to be parallel to X axis and our PID acts in Y-axis */

  /*From Euler's formula to calculate angles*/
  /*The angles we calculate are in radians hence we have to apply radian to degree conversion*/
  

  
  accelerationAngle_Y = atan(-1*(gForceX)/sqrt(pow(gForceY,2) + pow(gForceZ,2)))*rad_to_deg;
  
  totalAngle = 0.98 *(totalAngle + (rotY)*elapsedTime) + 0.02*accelerationAngle_Y;//complementary filter

/* we want integral controller to come into play only when the angle is very small to eliminate the steady state error*/
  if(-3 <error <3)
{
  intrgralError += error * elapsedTime;
}
  
  diffrentialError= (error - lastError)/elapsedTime;
  
  output = Kp * error + Ki * intrgralError + Kd * diffrentialError;



/* Our PWM signal has a minimum value of 1000 and maximum value of 2000 micro seconds*/
/* to make sure that our arithmetic operation does not exceed the value  we added this if condition*/
/*So that when we will add or subtract this value from  our throttle to determine the PWM signal we wont exceed the range*/

  if(output < -1000)
  {
     output=-1000;
  }
  if(output > 1000)
  {
     output=1000;
  }


  pwmLeft = throttle + output;
  pwmRight = throttle - output;

 /*  Again to make sure that PWM signal is within the range of 1000 to 2000 micro seconds we output a conditional value*/
 
  //Right
  if(pwmRight < 1000)
  {
    pwmRight= 1000;
  }
  if(pwmRight > 2000)
  {
    pwmRight=2000;
  }
  //Left
  if(pwmLeft < 1000)
  {
    pwmLeft= 1000;
  }
  if(pwmLeft > 2000)
  {
    pwmLeft=2000;
  }


 /* Applying PWM pulses of calculated widths to our motors */
  left_prop.writeMicroseconds(pwmLeft);
  right_prop.writeMicroseconds(pwmRight);

    
  lastError = error;
  
 
  }
//=============================================================
//                 Print
//=============================================================
void printData() {
  Serial.print(" Gyro (deg)");
  Serial.print(" X=");
  Serial.print(rotX);
  Serial.print(" Y=");
  Serial.print(rotY);
  Serial.print(" Z=");
  Serial.print(rotZ);
  Serial.print(" Accel (g)");
  Serial.print(" X=");
  Serial.print(gForceX);
  Serial.print(" Y=");
  Serial.print(gForceY);
  Serial.print(" Z=");
  Serial.println(gForceZ);
  Serial.print("Angle with axis: ");
  Serial.print(totalAngle);
  Serial.print("(deg)");
}
//=============================================================
//                 Loop
//=============================================================
void loop() {
  iterations++;

  recordAccelRegisters();
  recordGyroRegisters();
  pidLoop();
  //offsetCorrection();
  printData();
  delayMicroseconds(1);
}
