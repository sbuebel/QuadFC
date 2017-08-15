#include<Wire.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

float accel_x_zero, accel_y_zero, accel_z_zero;
float real_accel_x, real_accel_y, real_accel_z; // Unit: G's (9.8 m/s^2)

float gyro_x_zero, gyro_y_zero, gyro_z_zero;
float real_gyro_x, real_gyro_y, real_gyro_z; // Unit: Degrees/second

float pitch_from_accel, roll_from_accel;
float pitch_from_gyro, roll_from_gyro, yaw_from_gyro;

float pitch, roll, yaw;

//=====================================================================
//START SETUP
//=====================================================================
void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);

  // Maybe add in a 20 sec delay here to let the MPU balance...

  compute_zero_accels();
  compute_zero_gyros();
}

//=====================================================================
//START LOOP
//=====================================================================
void loop(){
  // Ready in all the values
  unsigned long start_time = micros();
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  // these are temporary values, so they can be local variables (only used to compute real_accels)
  float raw_accel_x = AcX;
  float raw_accel_y = AcY;
  float raw_accel_z = AcZ;

  float raw_gyro_x = GyX;
  float raw_gyro_y = GyY;
  float raw_gyro_z = GyZ;

  /* Here, we get the real values for acceleration, in G's:
   * ACCEL_X -> (PITCH) This value measures acceleration front to back on the quad
   * ACCEL_Y -> (ROLL) This value measures acceleration side to side on the quad
   * ACCEL_Z -> (GRAVITY) This value measures acceleration up and down on the quad (default value is 1 due to gravity) */
  real_accel_x = (raw_accel_x - accel_x_zero) / 8192;
  real_accel_y = (raw_accel_y - accel_y_zero) / 8192;
  real_accel_z = (raw_accel_z - (accel_z_zero-8192))/8192; // subtract 8192 to account for gravity's influence

  /* Here, we get the real values for gyro, in degrees/second:
   * GYRO_X -> (ROLL) This value measures rotation side to side
   * GYRO_Y -> (PITCH) This value measures rotation front to back
   * GYRO_Z -> (YAW) This value measures rotation about the vertical axis
   */
  real_gyro_x = (raw_gyro_x - gyro_x_zero) / 65.5;
  real_gyro_y = (raw_gyro_y - gyro_y_zero) / 65.5;
  real_gyro_z = (raw_gyro_z - gyro_z_zero) / 65.5;

  // Now we have good values for both accel and gyro, let's use accel values to calculate pitch and roll angles

  // Take the inverse tangent of accel_y / accel_z -> this will output in radians, so translate to degrees...
  roll_from_accel = -1*(180 / PI) * atan2(real_accel_y, real_accel_z);
  // Take the inverse tangent of accel_x / accel_z -> this will output in radians, so translate to degrees...
  pitch_from_accel = -1*(180 / PI) * atan2(real_accel_x, real_accel_z);

  // Now we have accurate pitch and roll from accel, we need pitch and roll based on our gyro outputs (RUNNING AT 250 Hz)
  pitch_from_gyro += real_gyro_y*0.004; // since we are taking 4 ms for each loop, multiply by this...
  roll_from_gyro -= real_gyro_x*0.004;
  yaw_from_gyro += real_gyro_z*0.004;

  /* FIX THIS, MESSING UP THE REST...
  // Last important step here is to account for switchover when roll and pitch are affected by yaw
  float old_p_f_g = pitch_from_gyro;
  float old_r_f_g = roll_from_gyro;
  pitch_from_gyro -= old_r_f_g * sin(-real_gyro_z * PI / 180 * 0.004);
  roll_from_gyro += old_p_f_g * sin(-real_gyro_z * PI / 180 * 0.004);*/

  yaw = yaw_from_gyro;
  pitch = 0.96*pitch_from_gyro + 0.04*pitch_from_accel;
  roll = 0.96*roll_from_gyro + 0.04*roll_from_accel;

  // Print PITCH and ROLL:
  Serial.print(pitch);
  Serial.print(" ");
  Serial.println(roll);

  // Print pitch and roll (from gryo)
  /*Serial.print(pitch_from_gyro);
  Serial.print(" ");
  Serial.println(roll_from_gyro);*/

  // Print acceleration values (filtered from MPU)
  /*Serial.print(real_accel_x);
  Serial.print(" ");
  Serial.print(real_accel_y);
  Serial.print(" ");
  Serial.println(real_accel_z); */

  // Print gyro values (filtered from MPU)
  /*Serial.print(real_gyro_x);
  Serial.print(" ");
  Serial.print(real_gyro_y);
  Serial.print(" ");
  Serial.println(real_gyro_z);*/

  // Print roll and pitch based on acceleration
  /*Serial.print(roll_from_accel);
  Serial.print(" ");
  Serial.println(pitch_from_accel); */
}

void compute_zero_accels() {
  float accel_x_total = 0;
  float accel_y_total = 0;
  float accel_z_total = 0;

  for (int i = 0; i < 1000; i ++) {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

    float accel_x = AcX;
    float accel_y = AcY;
    float accel_z = AcZ;

    accel_x_total += accel_x;
    accel_y_total += accel_y;
    accel_z_total += accel_z;
  }

  accel_x_zero = accel_x_total / 1000;
  accel_y_zero = accel_y_total / 1000;
  accel_z_zero = accel_z_total / 1000;    
}

void compute_zero_gyros() {
  float gyro_x_total = 0;
  float gyro_y_total = 0;
  float gyro_z_total = 0;

  for (int i = 0; i < 1000; i ++) {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
    AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
    AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    float gyro_x = GyX;
    float gyro_y = GyY;
    float gyro_z = GyZ;

    gyro_x_total += gyro_x;
    gyro_y_total += gyro_y;
    gyro_z_total += gyro_z;
  }

  gyro_x_zero = gyro_x_total / 1000;
  gyro_y_zero = gyro_y_total / 1000;
  gyro_z_zero = gyro_z_total / 1000;
}

