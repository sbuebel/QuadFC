/* FLIGHT CONTROLLER MARK TWO:
 *    This is a more complex flight controller which takes inputs from
 *    a GPS and barometer to approximate position and altitude at all times.
 *    Implementation for altitude and position hold will be included in
 *    future flight controller versions.
 */

// Try blending results of barometer and accelerometer to get a more precise altitude!

// Maybe do the same for accelerometer x, y to get an accurate position with GPS?

#include <Wire.h>
#include <stdio.h>
#include <TinyGPS++.h>
#include <Adafruit_MPL3115A2.h>

// Position hold stuff
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
float alt, zero_alt, alt_set_point, alt_throttle_adjust;
int pos_hold_switch;
float alt_difference[3]; // track the last three differences to get a weak PID (without I)
float alt_p_gain = 10.5;
float alt_d_gain = 3.5;
int altitude_hold_throttle;
float alt_velocity;
float alt_from_accel;

// GPS stuff
bool use_gps = false;
TinyGPSPlus gps;
float gps_latitude, gps_longitude, gps_speed;
float gps_home_latitude, gps_home_longitude;
float gps_home_position[2]; // keeps track of home x, y, z position
float gps_position[2]; // keeps track of an x, y, z position
float distance_to_home;
float earth_equator_circ = 40030823; // METERS - just use these, estimate our portion of the Earth to be flat, and get x,y
float earth_pole_circ = 40022776; // METERS

// Gyro stuff
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
float angle_pitch, angle_roll, angle_yaw;
int angle_pitch_buffer, angle_roll_buffer, angle_yaw_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output, angle_yaw_output;

// Motor speeds
int esc_1, esc_2, esc_3, esc_4;

// Motor speed timers
unsigned long timer_1, timer_2, timer_3, timer_4, esc_loop_timer;

bool led_on;

// PID stuff
float pid_error_temp;
float human_roll_command;
float gyro_roll_input, gyro_pitch_input, gyro_yaw_input;

float pid_p_gain_roll = 1.8;                                            // Increment by 0.2 until overcompensates, then decrease by 50%
float pid_i_gain_roll = 0.0;                                            // Increase by 0.01 until unstable, then decrease by 50%
float pid_d_gain_roll = 10.4;                                           // Raise until quad is unsteady, then decrease until quad is steady, then subtract 25%
int pid_max_roll = 400;
float pid_p_gain_pitch = pid_p_gain_roll;                               // Make these all the same as the roll ones to simplify...
float pid_i_gain_pitch = pid_i_gain_roll;
float pid_d_gain_pitch = pid_d_gain_roll;
int pid_max_pitch = pid_max_roll;       
float pid_p_gain_yaw = 5.0;
float pid_i_gain_yaw = 0.0;
float pid_d_gain_yaw = 5.0;
int pid_max_yaw = 400;

float pid_i_total_roll, pid_roll_setpoint, pid_last_roll_d_error, pid_output_roll;
float pid_i_total_pitch, pid_pitch_setpoint, pid_last_pitch_d_error, pid_output_pitch;
float pid_i_total_yaw, pid_yaw_setpoint, pid_last_yaw_d_error, pid_output_yaw;

// Pin definitions
int status_led = 13;
int c_1_input = 8;                                                      // controls yaw
int c_2_input = 9;                                                      // controls pitch
int c_3_input = 10;                                                     // controls throttle
int c_4_input = 11;                                                     // controls roll
int c_5_input = 12;

int start = 0;

unsigned long timer[6];                                                 // Used for receiver
byte last_channel[5];                                                   // Used for receiver
int receiver_input[5];                                                  // Receiver input values!

/* QUAD Orientation + Wiring directions
    QUADCOPTER ORIENTATION PICTURE
        CCW m_3 .> 0     FRONT
                   0     /
                   0   /
                   0 /
  CW m_2 -> 00000000000000000 <- m_4 CW
                 / 0
               /   0
             /     0
         BACK      0 <- m_1 CCW

  List of Connections:
  RX:   CH1 -> Digital Pin 8
        CH2 -> Digital Pin 9
        CH3 -> Digital Pin 10
        CH4 -> Digital Pin 11
        CH5 -> Digital Pin 12
        + -> 5V
        - -> GND
  GYRO: VCC -> 5V
        GND -> GND
        SCL -> Analog 5
        SDA -> Analog 4
  ESC1:   Signal -> Digital Pin 4
  ESC2:   Singal -> Digital Pin 5
  ESC3:   Signal -> Digital Pin 6
  ESC4:   Singal -> Digital Pin 7
    [ all ESC's take 3.3 V input and GND ]
*/

void setup() {
  Serial.begin(9600);                                                   // Use for debugging and GPS (hopefully)
  Wire.begin();                                                         // Start I2C as master
  baro.begin();                                                         // Start the barometer library (cleaned up)
  
  TWBR = 12;                                                            // Set the I2C clock speed to 400kHz.

  pinMode(status_led, OUTPUT);

  // Set all the interrupt pins
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0); // pin 8
  PCMSK0 |= (1 << PCINT1); // pin 9
  PCMSK0 |= (1 << PCINT2); // pin 10
  PCMSK0 |= (1 << PCINT3); // pin 11
  PCMSK0 |= (1 << PCINT4); // pin 12

  setup_mpu_6050_registers();                                           // Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro
  DDRD |= B11110000;    

  // Set HIGH and LOW levels for ESCs! (calibrate ESCs)
  while (receiver_input[2] < 1900) {
    digitalWrite(status_led, HIGH);
  } 
  delay(50);
  Serial.println(receiver_input[2]);
  // Flash the LED
  for (int i = 0; i < 3; i ++) {
    digitalWrite(status_led, HIGH); delay(50); digitalWrite(status_led, LOW); delay(50);
  }
  for (int i = 0; i < 1500; i ++) {
    PORTD |= B11110000;                                                 // Set digital port 4, 5, 6 and 7 high.
    delayMicroseconds(receiver_input[2]);                               // Set HIGH threshold.
    PORTD &= B00001111;                                                 // Set digital port 4, 5, 6 and 7 low.
    delayMicroseconds(4000 - receiver_input[2]);
  }
  digitalWrite(status_led, LOW);

  // Calibrate everything
  calibrate_gyro_altimeter_gps();

  // Easy way to calc x, y - assume our Earth is flat
  gps_home_position[0] = (gps_home_longitude / 180) * 0.5 * earth_equator_circ;
  gps_home_position[1] = (gps_home_latitude / 90) * 0.25 * earth_pole_circ;
  
  Serial.print(gps_home_latitude, 6);
  Serial.print(" ");
  Serial.println(gps_home_longitude, 6);

  loop_timer = micros();                                                // Reset the loop timer
  digitalWrite(status_led, LOW);

  Serial.println("Quad is ready.");
}

void loop() {
  if (use_gps == true) {
    // Try to get GPS data in
    get_gps_data();
  }

  // Turn on POS HOLD if swivel is above 1500! - only set altitude the first time!
  if (receiver_input[4] > 1500 && pos_hold_switch == 0) {
    // POS HOLD ON!
    pos_hold_switch = 1;
    alt_set_point = alt;
    altitude_hold_throttle = receiver_input[2];

  // If POS HOLD swivel is below 1500, turn off POS HOLD
  } else if (receiver_input[4] <= 1500) {
    // POS HOLD OFF!
    pos_hold_switch = 0;
  }

  // Position hold algorithm
  if (pos_hold_switch == 1) { // if alt ctrl is on...
    if (receiver_input[2] > 1950) {
      alt_set_point += 0.01;
    } else if (receiver_input[2] < 1050) {
      alt_set_point -= 0.01;
    }
    // Update differences, newest is [2], oldest is [0]
    alt_difference[0] = alt_difference[1];
    alt_difference[1] = alt_difference[2];
    alt_difference[2] = alt_set_point - alt; // change in alt required!
    float alt_dif_avg = 0.5 * (alt_difference[2] - alt_difference[1] + alt_difference[1] - alt_difference[0]);
    alt_throttle_adjust = (-1*alt_dif_avg*alt_d_gain + alt_difference[2]*alt_p_gain);
  } else {
    alt_throttle_adjust = 0;
  }

  //Serial.println(alt);
  /*Serial.print(" ");
  Serial.print(alt_set_point);
  Serial.print(" ");
  Serial.println(alt_throttle_adjust);*/

  // 65.5 = 1 deg/sec (from MPU-6050 spec sheet) COMPLIMENTARY FILTER
  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_x/65.5) * 0.3);    // Gyro PID input is in degrees/second
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_y/65.5) * 0.3);
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_z/65.5) * 0.3);
  
  // Throttle bottom, yaw far left
  if (receiver_input[2] < 1050 && receiver_input[3] < 1050) start = 1;

  if (start == 1 && receiver_input[2] < 1050 && receiver_input[3] > 1450) {
    start = 2; // actually time to fly now

    angle_pitch = angle_pitch_acc;
    angle_roll = angle_roll_acc;
    set_gyro_angles = true;

    // Reset PID stuff for smooth start
    reset_pid();
  }

  // throttle low and yaw far right stops everything!
  if (start == 2 && receiver_input[2] < 1050 && receiver_input[3] > 1950) start = 0;
  
  // Get controller setpoint! Need this for PID calculations - SETPOINT is in degrees/second!!!
  pid_roll_setpoint = 0;
  if (receiver_input[0] > 1508) { // ROLL ROLL ROLL
    pid_roll_setpoint = (receiver_input[0] - 1508);
  } else if (receiver_input[0] < 1492) {
    pid_roll_setpoint = (receiver_input[0] - 1492);
  }
  pid_roll_setpoint -= angle_roll_output*15; // to autolevel
  pid_roll_setpoint /= 3;

  pid_pitch_setpoint = 0;
  if (receiver_input[1] > 1508) { // PITCH PITCH PITCH
    pid_pitch_setpoint = (receiver_input[1] - 1508);
  } else if (receiver_input[1] < 1492) {
    pid_pitch_setpoint = (receiver_input[1] - 1492);
  }
  pid_pitch_setpoint -= angle_pitch_output*15; // to autolevel
  pid_pitch_setpoint /= 3;

  pid_yaw_setpoint = 0;
  // make sure throttle is on
  if (receiver_input[2] > 1050) {
    if (receiver_input[3] > 1508) { // YAW YAW YAW
      pid_yaw_setpoint = (receiver_input[3] - 1508) / -3;
    } else if (receiver_input[3] < 1492) {
      pid_yaw_setpoint = (receiver_input[3] - 1492) / -3;
    }
  }

  // Now that we have all of our setpoints, calculate actions necessary
  calculate_pid();

  // Update motor speeds here
  if (start == 2) {
    if (receiver_input[2] > 1800) receiver_input[2] = 1800;

    /* Controller INPUTS:
     *  Roll Left: creates positive PID value (increase speed of 1, 4)
     *  Roll Right: creates negative PID value (increase speed of 2, 3)
     *  Pitch Up: creates negative PID value (move forwards, increase speed of 1, 2)
     *  Pitch Down: creates positive PID value (move backwards, increase speed of 3, 4)
     */

    if (pos_hold_switch == 0) {
      // default to throttle level but take into account PID settings!
      esc_1 = receiver_input[2] - pid_output_pitch + pid_output_roll + pid_output_yaw;
      esc_2 = receiver_input[2] - pid_output_pitch - pid_output_roll - pid_output_yaw;
      esc_3 = receiver_input[2] + pid_output_pitch - pid_output_roll + pid_output_yaw;
      esc_4 = receiver_input[2] + pid_output_pitch + pid_output_roll - pid_output_yaw;
    } else if (pos_hold_switch == 1) {
      // default to throttle level but take into account PID settings!
      esc_1 = altitude_hold_throttle - pid_output_pitch + pid_output_roll + pid_output_yaw + alt_throttle_adjust;
      esc_2 = altitude_hold_throttle - pid_output_pitch - pid_output_roll - pid_output_yaw + alt_throttle_adjust;
      esc_3 = altitude_hold_throttle + pid_output_pitch - pid_output_roll + pid_output_yaw + alt_throttle_adjust;
      esc_4 = altitude_hold_throttle + pid_output_pitch + pid_output_roll - pid_output_yaw + alt_throttle_adjust;
    }
    // Keep motors running
    int keep_value = 1100;
    if (esc_1 < keep_value) esc_1 = keep_value;
    if (esc_2 < keep_value) esc_2 = keep_value;
    if (esc_3 < keep_value) esc_3 = keep_value;
    if (esc_4 < keep_value) esc_4 = keep_value;

    // Keep signals below 2000
    if (esc_1 > 2000) esc_1 = 2000;
    if (esc_2 > 2000) esc_2 = 2000;
    if (esc_3 > 2000) esc_3 = 2000;
    if (esc_4 > 2000) esc_4 = 2000;
    
  } else {
    // Turn them all off
    esc_1 = 1000;
    esc_2 = 1000;
    esc_3 = 1000;
    esc_4 = 1000;
  }

  //print_motor_speeds();

  //Serial.println(micros() - loop_timer); // LOOP TIME
  if (micros() - loop_timer > 4050) {
    digitalWrite(status_led, HIGH); // We are taking too long!!!
  } else {
    while (micros() - loop_timer < 4000);                             // Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  }
  loop_timer = micros();                                            // Reset the loop timer

  // This really becomes the START of the next loop!
  
  PORTD |= B11110000;                                               // Set digital outputs 4,5,6 and 7 high
  timer_1 = esc_1 + loop_timer;                                     // Calculate the time of the faling edge of the esc-1 pulse
  timer_2 = esc_2 + loop_timer;                                     // Calculate the time of the faling edge of the esc-2 pulse
  timer_3 = esc_3 + loop_timer;                                     // Calculate the time of the faling edge of the esc-3 pulse
  timer_4 = esc_4 + loop_timer;                                     // Calculate the time of the faling edge of the esc-4 pulse
 
  // Get values from the gyro
  read_mpu_6050_data();                                                 // Read the raw acc and gyro data from the MPU-6050
  
  // Get new altitude value (small comp filter)
  float new_alt = baro.getAltitude();
  if (abs(alt - new_alt) < 1) { // dont let crazy values mess us up
    alt = 0.9*alt + 0.1*new_alt;
  }

  alt_velocity += acc_z * 0.04; // add accel_z * loop_time!
  alt_from_accel += alt_velocity * 0.04;
  
  //Serial.println(micros() - loop_timer); //debugging, timing stuff
  
  while(PORTD >= 16) {                                              // Stay in this loop until output 4,5,6 and 7 are low (B00010000 = 16 = lowest possible value)
    esc_loop_timer = micros();                                      // Read the start time
    if(timer_1 <= esc_loop_timer)PORTD &= B11101111;                // Set digital output 4 to low if the time is expired
    if(timer_2 <= esc_loop_timer)PORTD &= B11011111;                // Set digital output 5 to low if the time is expired
    if(timer_3 <= esc_loop_timer)PORTD &= B10111111;                // Set digital output 6 to low if the time is expired
    if(timer_4 <= esc_loop_timer)PORTD &= B01111111;                // Set digital output 7 to low if the time is expired
  }
  
  // make caclulations for angles in...
  gyro_signal_in();
}






void print_motor_speeds() {
  Serial.print(esc_1); Serial.print(" "); Serial.print(esc_2); Serial.print(" ");
  Serial.print(esc_3); Serial.print(" "); Serial.println(esc_4);
}

void calculate_pid() { 
  // Roll calculations first:
  pid_error_temp = gyro_roll_input - pid_roll_setpoint; // error = gyro - receiver
  pid_i_total_roll += (pid_i_gain_roll * pid_error_temp);

  // LIMIT extreme output from i term
  if (pid_i_total_roll > pid_max_roll) {
    pid_i_total_roll = pid_max_roll;
  } else if (pid_i_total_roll < -1 * pid_max_roll) {
    pid_i_total_roll = -1 * pid_max_roll;
  }

  pid_output_roll = (pid_error_temp * pid_p_gain_roll) + pid_i_total_roll + pid_d_gain_roll*(pid_error_temp - pid_last_roll_d_error);
  pid_last_roll_d_error = pid_error_temp;

  // LIMIT extreme output
  if (pid_output_roll > pid_max_roll) {
    pid_output_roll = pid_max_roll;
  } else if (pid_output_roll < -1 * pid_max_roll) {
    pid_output_roll = -1 * pid_max_roll;
  }


  

  // Pitch calculations next:
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint; // error = gyro - receiver
  pid_i_total_pitch += (pid_i_gain_pitch * pid_error_temp);

  // LIMIT extreme output
  if (pid_i_total_pitch > pid_max_pitch) {
    pid_i_total_pitch = pid_max_pitch;
  } else if (pid_i_total_pitch < -1 * pid_max_pitch) {
    pid_i_total_pitch = -1 * pid_max_pitch;
  }

  pid_output_pitch = (pid_error_temp * pid_p_gain_pitch) + pid_i_total_pitch + pid_d_gain_pitch*(pid_error_temp - pid_last_pitch_d_error);
  pid_last_pitch_d_error = pid_error_temp;

  // LIMIT extreme output
  if (pid_output_pitch > pid_max_pitch) {
    pid_output_pitch = pid_max_pitch;
  } else if (pid_output_pitch < -1 * pid_max_pitch) {
    pid_output_pitch = -1 * pid_max_pitch;
  }




  // Yaw calculations last:
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint; // error = gyro - receiver
  pid_i_total_yaw += (pid_i_gain_yaw * pid_error_temp);

  // LIMIT extreme output
  if (pid_i_total_yaw > pid_max_yaw) {
    pid_i_total_yaw = pid_max_yaw;
  } else if (pid_i_total_yaw < -1 * pid_max_yaw) {
    pid_i_total_yaw = -1 * pid_max_yaw;
  }

  pid_output_yaw = (pid_error_temp * pid_p_gain_yaw) + pid_i_total_yaw + pid_d_gain_yaw*(pid_error_temp - pid_last_yaw_d_error);
  pid_last_yaw_d_error = pid_error_temp;

  // LIMIT extreme output
  if (pid_output_yaw > pid_max_yaw) {
    pid_output_yaw = pid_max_yaw;
  } else if (pid_output_yaw < -1 * pid_max_yaw) {
    pid_output_yaw = -1 * pid_max_yaw;
  }
}

void read_mpu_6050_data() {
  Wire.beginTransmission(0x68);                                         // Start communicating with the MPU-6050
  Wire.write(0x3B);                                                     // Send the requested starting register
  Wire.endTransmission();                                               // End the transmission
  Wire.requestFrom(0x68, 14);                                           // Request 14 bytes from the MPU-6050
  while (Wire.available() < 14);                                        // Wait until all the bytes are received
  acc_x = Wire.read() << 8 | Wire.read();                               // Add the low and high byte to the acc_x variable
  acc_y = Wire.read() << 8 | Wire.read();                               // Add the low and high byte to the acc_y variable
  acc_z = Wire.read() << 8 | Wire.read();                               // Add the low and high byte to the acc_z variable
  temperature = Wire.read() << 8 | Wire.read();                         // Add the low and high byte to the temperature variable
  gyro_x = Wire.read() << 8 | Wire.read();                              // Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read() << 8 | Wire.read();                              // Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read() << 8 | Wire.read();                              // Add the low and high byte to the gyro_z variable
}

void setup_mpu_6050_registers() {
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                         // Start communicating with the MPU-6050
  Wire.write(0x6B);                                                     // Send the requested starting register
  Wire.write(0x00);                                                     // Set the requested starting register
  Wire.endTransmission();                                               // End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                         // Start communicating with the MPU-6050
  Wire.write(0x1C);                                                     // Send the requested starting register
  Wire.write(0x10);                                                     // Set the requested starting register
  Wire.endTransmission();                                               // End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                         // Start communicating with the MPU-6050
  Wire.write(0x1B);                                                     // Send the requested starting register
  Wire.write(0x08);                                                     // Set the requested starting register
  Wire.endTransmission();                                               // End the transmission
}

ISR(PCINT0_vect) {
  // if interrupt occurs, save the time
  timer[0] = micros();

  // check which pin changed

  // CHANNEL 1 (yaw)
  if (last_channel[0] == 0 && PINB & B00000001) {
    last_channel[0] = 1;
    timer[1] = timer[0];
  } else if (last_channel[0] == 1 && !(PINB & B00000001)) {
    last_channel[0] = 0;
    receiver_input[0] = timer[0] - timer[1];
  }
  // CHANNEL 2 (pitch)
  if (last_channel[1] == 0 && PINB & B00000010) {
    last_channel[1] = 1;
    timer[2] = timer[0];
  } else if (last_channel[1] == 1 && !(PINB & B00000010)) {
    last_channel[1] = 0;
    receiver_input[1] = timer[0] - timer[2];
  }
  // CHANNEL 3 (throttle)
  if (last_channel[2] == 0 && PINB & B00000100) {
    last_channel[2] = 1;
    timer[3] = timer[0];
  } else if (last_channel[2] == 1 && !(PINB & B00000100)) {
    last_channel[2] = 0;
    receiver_input[2] = timer[0] - timer[3];
  }
  // CHANNEL 4 (roll)
  if (last_channel[3] == 0 && PINB & B00001000) {
    last_channel[3] = 1;
    timer[4] = timer[0];
  } else if (last_channel[3] == 1 && !(PINB & B00001000)) {
    last_channel[3] = 0;
    receiver_input[3] = timer[0] - timer[4];
  }

  // CHANNEL 5 (cutoff)
  if (last_channel[4] == 0 && PINB & B00010000) {
    last_channel[4] = 1;
    timer[5] = timer[0];
  } else if (last_channel[4] == 1 && !(PINB & B00010000)) {
    last_channel[4] = 0;
    receiver_input[4] = timer[0] - timer[5];
  }
}

void gyro_signal_in() {
  gyro_x -= gyro_x_cal;                                                 // Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                                                 // Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                                                 // Subtract the offset calibration value from the raw gyro_z value

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz * 65.5)
  angle_roll += gyro_x * 0.0000611;                                     // Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_pitch += gyro_y * 0.0000611;                                    // Calculate the traveled roll angle and add this to the angle_roll variable
  angle_yaw += gyro_z * 0.0000611;                                      // Will drift over time, but shouldn't be a big deal because it's all relative

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_roll += angle_pitch * sin(gyro_z * 0.000001066);                // If the IMU has yawed transfer the pitch angle to the roll angel
  angle_pitch -= angle_roll * sin(gyro_z * 0.000001066);                // If the IMU has yawed transfer the roll angle to the pitch angel

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); // Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_roll_acc = asin((float)acc_y / acc_total_vector) * 57.296;      // Calculate the pitch angle
  angle_pitch_acc = asin((float)acc_x / acc_total_vector) * -57.296;    // Calculate the roll angle

  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  angle_roll_acc += 0.66;                                               // Accelerometer calibration value for pitch
  angle_pitch_acc += 0.21;                                             // Accelerometer calibration value for roll

  if (set_gyro_angles) {                                                // If the IMU is already started
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;      // Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;         // Correct the drift of the gyro roll angle with the accelerometer roll angle
  } else {                                                              // At first start
    angle_pitch = angle_pitch_acc;                                      // Set the gyro pitch angle equal to the accelerometer pitch angle
    angle_roll = angle_roll_acc;                                        // Set the gyro roll angle equal to the accelerometer roll angle
    set_gyro_angles = true;                                             // Set the IMU started flag
  }

  // To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;    // Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;       // Take 90% of the output roll value and add 10% of the raw roll value
  angle_yaw_output = angle_yaw_output * 0.9 + angle_yaw * 0.1;          // Might as well stick a comp filter on the yaw too!
}

void reset_pid() {
  // Reset PID stuff for smooth start
  pid_i_total_roll = 0; 
  pid_last_roll_d_error = 0;
  pid_i_total_pitch = 0;
  pid_last_pitch_d_error = 0;
  pid_i_total_yaw = 0;
  pid_last_yaw_d_error = 0;
}

void get_gps_data() {
  if (gps_home_latitude != 0) {
    if (Serial.available()) {
      if (gps.encode(Serial.read())) {
        gps_latitude = gps.location.lat();
        gps_longitude = gps.location.lng();
        gps_speed = gps.speed.mph();
  
        /* OTHER WAY TO CALC X, Y
        gps_position[0] = (earth_radius + alt) * cos(gps_latitude*PI/180) * cos(gps_longitude*PI/180);
        gps_position[1] = (earth_radius + alt) * cos(gps_latitude*PI/180) * sin(gps_longitude*PI/180); */
  
        gps_position[0] = (gps_longitude / 180) * 0.5 * earth_equator_circ;
        gps_position[1] = (gps_latitude / 90) * 0.25 * earth_pole_circ;
              
        distance_to_home = gps.distanceBetween(gps_latitude, gps_longitude, gps_home_latitude, gps_home_longitude);
        //distance_to_home = sqrt(pow((gps_position[0] - gps_home_position[0]), 2) + pow((gps_position[1] - gps_home_position[1]), 2));
        Serial.println(distance_to_home);
        //Serial.print(gps_latitude); Serial.print(" "); Serial.print(gps_longitude); Serial.print(" "); Serial.println(gps_speed);
      }
    }
  }
}

void calibrate_gyro_altimeter_gps() {
  // CALIBRATE GYRO!
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++) {                  // Run this code 2000 times
    if (cal_int % 125 == 0) {
      led_on = !led_on;                                                 // Light up the LED every 125 times
    }
    if (led_on == true) {
      digitalWrite(status_led, HIGH);
    } else {
      digitalWrite(status_led, LOW);
    }
    read_mpu_6050_data();                                               // Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                                               // Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                                               // Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                                               // Add the gyro z-axis offset to the gyro_z_cal variable
    
    // Give ESC's a 1000us pulse
    PORTD |= B11110000;                                                 // Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                            // Wait 1000us.
    PORTD &= B00001111;                                                 // Set digital poort 4, 5, 6 and 7 low.
    delayMicroseconds(3000);                                            // Delay 3000 micros to simulate the 250 Hz program loop
  } 
  gyro_x_cal /= 2000;                                                   // Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                   // Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;                                                   // Divide the gyro_z_cal variable by 2000 to get the avarage offset

  // Calibrate altimeter
  for (int cal_int = 0; cal_int < 100; cal_int ++) {
    zero_alt += baro.getAltitude();
   
    // Give ESC's a 1000us pulse
    PORTD |= B11110000;                                                 // Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                            // Wait 1000us.
    PORTD &= B00001111;                                                 // Set digital poort 4, 5, 6 and 7 low.
    delayMicroseconds(3000);                                            // Delay 3000 micros to simulate the 250 Hz program loop
  }
  zero_alt /= 100;
  alt = zero_alt; // to start
  pos_hold_switch = 0;
  alt_from_accel = zero_alt; // start the accel alt at the level of the barometer alt!

  // Get a HOME position for gps stuff
  if (use_gps == true) {
    bool get_home_pos = true;
    while (get_home_pos == true) {
      if (Serial.available()) {
        if (gps.encode(Serial.read())) {
          gps_home_latitude = gps.location.lat();
          gps_home_longitude = gps.location.lng();
          if (gps_home_latitude != 0 && gps_home_longitude != 0) {
            get_home_pos = false;
          }
        }
        // Give ESC's a 1000us pulse
        PORTD |= B11110000;                                                 // Set digital poort 4, 5, 6 and 7 high.
        delayMicroseconds(1000);                                            // Wait 1000us.
        PORTD &= B00001111;                                                 // Set digital poort 4, 5, 6 and 7 low.
        delayMicroseconds(3000);                                            // Delay 3000 micros to simulate the 250 Hz program loop
      }
    }
  }
}
