/* FLIGHT CONTROLLER MARK FOUR:
 *    This is a more complex flight controller which takes inputs from
 *    a GPS and barometer to approximate position and altitude at all times.
 *    Implementation for altitude and position hold will be included in
 *    future flight controller versions.
 */

// Try blending results of barometer and accelerometer to get a more precise altitude!
// Maybe do the same for accelerometer x, y to get an accurate position with GPS?

#include <Wire.h>
#include <NMEAGPS.h>
#include <Adafruit_MPL3115A2.h>
#include <StandardCplusplus.h>
#include <vector>

// Position hold stuff
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
float alt, alt_from_accel, alt_from_baro, alt_set_point, alt_throttle_adjust, alt_velocity, old_alt_from_baro;
int pos_hold_switch;
float alt_difference[3]; // track the last three differences to get a weak PID (without I)
float alt_p_gain = 10.5;
float alt_d_gain = 3.5;
int altitude_hold_throttle;

// GPS stuff
bool use_gps = true;
NMEAGPS gps;
gps_fix fix;
NeoGPS::Location_t home;
float distance_to_home;
int gps_count; // number of gps locations we have so far
bool got_home_pos = false;
float gps_home_position[2]; // keeps track of home x, y, z position
float gps_position[2]; // keeps track of an x, y, z position
float earth_equator_circ = 40030823; // METERS - just use these, estimate our portion of the Earth to be flat, and get x,y
float earth_pole_circ = 40022776; // METERS
std::vector<std::vector<float>> gps_waypoints;

// Gyro stuff
int gyro_x, gyro_y, gyro_z, temperature;
long acc_x, acc_y, acc_z, acc_total_vector;
float gyro_x_cal, gyro_y_cal, gyro_z_cal, acc_z_cal;
long loop_timer, start_time;
float angle_pitch, angle_roll, angle_yaw;
int angle_pitch_buffer, angle_roll_buffer, angle_yaw_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output, angle_yaw_output;

// Motor speeds and timers
int esc_1, esc_2, esc_3, esc_4;
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
  GPS:  RX -> TX
        TX -> RX
  BARO: SCL -> Analog 5
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
  DDRD |= B11110000; // Set motor pins as outputs

  // Get high and low signals from controller
  set_motor_limits();
  flash_LED();

  // Calibrate everything
  calibrate_gyro_altimeter_gps();
  flash_LED();

  loop_timer = micros(); start_time = millis(); // Set loop timer and start time
}

void loop() {
  if (use_gps) {
    // Try to get GPS data in
    int waypoint_wait = 5000; // new waypoint every 5 seconds
    if (millis() - start_time > waypoint_wait) {
      get_gps_data(true);
      start_time = millis();
      print_waypoints();
    } else {
      get_gps_data(false);
    }
  }

  // 65.5 = 1 deg/sec (from MPU-6050 spec sheet) COMPLIMENTARY FILTER
  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_x/65.5) * 0.3);    // Gyro PID input is in degrees/second
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_y/65.5) * 0.3);
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_z/65.5) * 0.3);
  
  if (receiver_input[2] < 1050 && receiver_input[3] < 1050) { // Throttle low, yaw far left
    start = 1;
  }
  if (start == 1 && receiver_input[2] < 1050 && receiver_input[3] > 1450) { // Throttle low, yaw middle
    start = 2; // actually time to fly now

    // Reset PID stuff for smooth start
    reset_controller();
  }

  // throttle low and yaw far right stops everything!
  if (start == 2 && receiver_input[2] < 1050 && receiver_input[3] > 1950) start = 0;

  /* Controller INPUTS:
   *  Roll Left: creates positive PID value (increase speed of 1, 4)
   *  Roll Right: creates negative PID value (increase speed of 2, 3)
   *  Pitch Up: creates negative PID value (move forwards, increase speed of 1, 2)
   *  Pitch Down: creates positive PID value (move backwards, increase speed of 3, 4)
   */
  
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
  if (receiver_input[2] > 1050 || pos_hold_switch == 1) {
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
    update_motor_speeds();
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

  // This really becomes the START of the next loop!
  loop_timer = micros();                                            // Reset the loop timer
  
  PORTD |= B11110000;                                               // Set digital outputs 4,5,6 and 7 high
  timer_1 = esc_1 + loop_timer;                                     // Calculate the time of the falling edge of the esc-1 pulse
  timer_2 = esc_2 + loop_timer;                                     // Calculate the time of the falling edge of the esc-2 pulse
  timer_3 = esc_3 + loop_timer;                                     // Calculate the time of the falling edge of the esc-3 pulse
  timer_4 = esc_4 + loop_timer;                                     // Calculate the time of the falling edge of the esc-4 pulse
 
  // Get values from the gyro
  read_mpu_6050_data();                                                 // Read the raw acc and gyro data from the MPU-6050
  
  read_altitude_data();

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




void read_altitude_data() {
  // Get new altitude value (small comp filter)
  old_alt_from_baro = alt_from_baro;
  alt_from_baro = baro.getAltitude();
  if (old_alt_from_baro == alt_from_baro) { // reset accel stuff if we aren't changing altitude
    //alt_from_accel = alt_from_baro;
    alt_velocity *= 0.5;
    if (alt_velocity < 0) {
      acc_z_cal += 0.05*alt_velocity;
    } else {
      acc_z_cal += 0.05*alt_velocity;
    }
  }
  
  // NEED TO FIGURE OUT AN OFFSET VALUE TO MAKE THIS ZERO AT BASELINE! (definitely during gyro calibration...)
  //0.00024414062 = 1 / 4096
  alt_velocity += ((float)acc_z*0.00024414062 - acc_z_cal) * 0.04; // add accel_z * loop_time!
  alt_from_accel += alt_velocity * 0.04;
  alt = 0.25*alt_from_baro + 0.75*alt_from_accel; // comp filter
}

void update_motor_speeds() {
  if (receiver_input[2] > 1800) receiver_input[2] = 1800;

  // default to throttle level but take into account PID settings! and pos hold
  esc_1 = receiver_input[2] - pid_output_pitch + pid_output_roll + pid_output_yaw + alt_throttle_adjust;
  esc_2 = receiver_input[2] - pid_output_pitch - pid_output_roll - pid_output_yaw + alt_throttle_adjust;
  esc_3 = receiver_input[2] + pid_output_pitch - pid_output_roll + pid_output_yaw + alt_throttle_adjust;
  esc_4 = receiver_input[2] + pid_output_pitch + pid_output_roll - pid_output_yaw + alt_throttle_adjust;

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
}

void position_hold() {
  // See if we want position hold on:
  if (receiver_input[4] > 1500 && pos_hold_switch == 0) {
    pos_hold_switch = 1; // POS HOLD ON!
    alt_set_point = alt;
    altitude_hold_throttle = receiver_input[2];
  } else if (receiver_input[4] <= 1500) {
    pos_hold_switch = 0; // POS HOLD OFF!
    alt_set_point = 0;
  }

  // Position hold algorithm
  if (pos_hold_switch == 1) { // if alt ctrl is on...
    if (receiver_input[2] > 1950) {
      alt_set_point += 0.01;
    } else if (receiver_input[2] < 1050) {
      alt_set_point -= 0.01;
    }
    // Update differences, newest is [2], oldest is [0]
    alt_difference[1] = alt_difference[0];
    alt_difference[0] = alt_set_point - alt; // change in alt required!
    float alt_dif_dif = alt_difference[0] - alt_difference[1];
    alt_throttle_adjust = altitude_hold_throttle + (-1*alt_dif_dif*alt_d_gain + alt_difference[0]*alt_p_gain) - receiver_input[2];
  } else {
    alt_throttle_adjust = 0;
  }
  
}

void set_motor_limits() {
  // Set HIGH and LOW levels for ESCs! (calibrate ESCs)
  while (receiver_input[2] < 1900) {
    digitalWrite(status_led, HIGH);
  }
  delay(250);

  flash_LED();

  for (int i = 0; i < 1500; i ++) {
    PORTD |= B11110000;                                                 // Set digital port 4, 5, 6 and 7 high.
    delayMicroseconds(receiver_input[2]);                               // Set HIGH threshold.
    PORTD &= B00001111;                                                 // Set digital port 4, 5, 6 and 7 low.
    delayMicroseconds(4000 - receiver_input[2]);
  }
  digitalWrite(status_led, LOW);
}

void print_waypoints() {
  for (int i; i < gps_waypoints.size(); i ++) {
    Serial.print(gps_waypoints[i][0]);
    Serial.print(" ");
    Serial.println(gps_waypoints[i][1]);
  }
}

void print_motor_speeds() {
  Serial.print(esc_1); Serial.print( ' ' ); Serial.print(esc_2); Serial.print( ' ' );
  Serial.print(esc_3); Serial.print( ' ' ); Serial.print(esc_4); Serial.print( ' ' );
  Serial.print(alt); Serial.print( ' ' ); Serial.println(alt_set_point);
}

void flash_LED() {
  delay(50);

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

void reset_controller() {
  // Reset PID stuff for smooth start
  pid_i_total_roll = 0; 
  pid_last_roll_d_error = 0;
  pid_i_total_pitch = 0;
  pid_last_pitch_d_error = 0;
  pid_i_total_yaw = 0;
  pid_last_yaw_d_error = 0;

  // reset gyro values (no vibration, so accelerometer values should be accurate)
  angle_pitch = angle_pitch_acc;
  angle_roll = angle_roll_acc;
  set_gyro_angles = true;
    
  // Reset accelerometer alt variables
  alt_velocity = 0;
  alt_from_accel = alt_from_baro;

}

void get_gps_data(bool add_waypoint) {
  if (gps.available( Serial )) {
    fix = gps.read();

    if (fix.valid.location) {
      gps_position[0] = (fix.longitude() / 180) * 0.5 * earth_equator_circ;
      gps_position[1] = (fix.latitude() / 90) * 0.25 * earth_pole_circ;

      if (add_waypoint) {
        std::vector<float> temp_gps_location;
        temp_gps_location.push_back(gps_position[0]);
        temp_gps_location.push_back(gps_position[1]);
  
        gps_waypoints.push_back(temp_gps_location);
      }
      distance_to_home = fix.location.DistanceKm( home ) * 1000;

      //Serial.println(distance_to_home);
      //Serial.print(fix.latitude()); Serial.print( ' ' );
      //Serial.print(fix.longitude()); Serial.print( ' ' );
      //Serial.println(fix.speed());
    }
  }
}

void calibrate_gyro_altimeter_gps() {
  // CALIBRATE GYRO!
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++) {                  // Run this code 2000 times
    if (cal_int % 125 == 0) {
      led_on = !led_on;                                                 // Light up the LED every 125 times
    }
    if (led_on) {
      digitalWrite(status_led, HIGH);
    } else {
      digitalWrite(status_led, LOW);
    }
    read_mpu_6050_data();                                               // Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += (float)gyro_x;                                               // Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += (float)gyro_y;                                               // Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += (float)gyro_z;                                               // Add the gyro z-axis offset to the gyro_z_cal variable
    acc_z_cal += (float)acc_z/4096;
    
    // Give ESC's a 1000us pulse
    PORTD |= B11110000;                                                 // Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                            // Wait 1000us.
    PORTD &= B00001111;                                                 // Set digital poort 4, 5, 6 and 7 low.
    delayMicroseconds(3000);                                            // Delay 3000 micros to simulate the 250 Hz program loop
  } 
  gyro_x_cal /= 2000;                                                   // Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                   // Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;                                                   // Divide the gyro_z_cal variable by 2000 to get the avarage offset
  acc_z_cal /= 2000;

  // Calibrate altimeter
  for (int cal_int = 0; cal_int < 100; cal_int ++) {
    alt_from_baro += baro.getAltitude();
   
    // Give ESC's a 1000us pulse
    PORTD |= B11110000;                                                 // Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                            // Wait 1000us.
    PORTD &= B00001111;                                                 // Set digital poort 4, 5, 6 and 7 low.
    delayMicroseconds(3000);                                            // Delay 3000 micros to simulate the 250 Hz program loop
  }
  alt_from_baro /= 100;
  pos_hold_switch = 0;
  alt_from_accel = alt_from_baro; // start the accel alt at the level of the barometer alt!

  // Get a HOME position for gps stuff
  unsigned long gps_pos_attempt = millis();
  Serial.print( F("Waiting for GPS signal...") );

  while (not got_home_pos) {
    if (gps.available(Serial)) { // reading from the Serial port (for now)
      fix = gps.read();

      if (fix.valid.location) {
        home = fix.location;
        Serial.print( home.latF() ); Serial.print( ' ' );
        Serial.println( home.lonF() );
        got_home_pos = true;
        use_gps      = true;
      } else {
        led_on = !led_on;
        if (led_on) {
          digitalWrite(status_led, HIGH);
        } else {
          digitalWrite(status_led, LOW);
        }
        Serial.print( '.' );
      }
    }

    // Give ESC's a 1000us pulse
    PORTD |= B11110000;                                                 // Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                            // Wait 1000us.
    PORTD &= B00001111;                                                 // Set digital poort 4, 5, 6 and 7 low.
    delayMicroseconds(3000);                                            // Delay 3000 micros to simulate the 250 Hz program loop
    // Give it 5 seconds to connect to GPS, no more
    if (millis() - gps_pos_attempt > 5000) {
      use_gps = false;
      break; // out of the while loop
    }
  }

  if (got_home_pos) {
    // Easy way to calc x, y - assume our Earth is flat
    gps_home_position[0] = (home.lonF() / 180) * 0.5 * earth_equator_circ;
    gps_home_position[1] = (home.latF() / 90) * 0.25 * earth_pole_circ;

    std::vector<float> temp_gps_location;
    temp_gps_location.push_back(gps_home_position[0]);
    temp_gps_location.push_back(gps_home_position[1]);
    gps_waypoints.push_back(temp_gps_location);
    
    Serial.print(gps_home_position[0], 6);
    Serial.print(' ');
    Serial.println(gps_home_position[1], 6);
  } else {
    Serial.print( F("Home position not available.") );
  }
  Serial.println( F("Quad is ready.") );
  digitalWrite(status_led, LOW);
}
