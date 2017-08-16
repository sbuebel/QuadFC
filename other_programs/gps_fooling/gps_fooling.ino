#include <TinyGPS++.h>
#include <NeoSWSerial.h>
#include <EnableInterrupt.h>

TinyGPSPlus gps;
NeoSWSerial neo_serial(2, 3);

float x_coordinate, y_coordinate, zero_x_coordinate, zero_y_coordinate;
float last_x, last_y;
float r = 3959; // miles!
float velocity; // mph!

void nssPortISR()
{
  NeoSWSerial::rxISR( *portInputRegister( digitalPinToPort(2) ) );
}

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  neo_serial.begin(9600); // Start software serial for GPS at 9600 baud rate (default for GPS)
  enableInterrupt(2, nssPortISR, CHANGE );
}

void loop() {
  bool got_data = false;
  //Serial.println("LOOP");
  // put your main code here, to run repeatedly:
  if (neo_serial.available()) {
    //Serial.println("AVAILABLE");
    if (gps.encode(neo_serial.read())) {
        //Serial.println(alt_serial.read());
        float latitude = gps.location.lat();
        float longitude = gps.location.lng();
        last_x = x_coordinate;
        last_y = y_coordinate;
        x_coordinate = r * cos(latitude*PI/180) * cos(longitude*PI/180);
        y_coordinate = r * cos(latitude*PI/180) * sin(longitude*PI/180); 
        
        Serial.print(latitude, 6);
        Serial.print(" ");
        Serial.println(longitude, 6);

        float distance = sqrt(pow((x_coordinate - last_x), 2) + pow((y_coordinate - last_y), 2));

        //Serial.print(x_coordinate)
        //Serial.println(y_coordinate);

        //Serial.println(distance);
    }  
  }
}
