#include <TinyGPS++.h>
#include <AltSoftSerial.h>

TinyGPSPlus gps;
AltSoftSerial alt_serial;

float x_coordinate, y_coordinate, zero_x_coordinate, zero_y_coordinate;
float last_x, last_y;
float r = 3959; // miles!
float velocity; // mph!


void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  alt_serial.begin(9600); // Start software serial for GPS at 9600 baud rate (default for GPS)
}

void loop() {
  bool got_data = false;

  // put your main code here, to run repeatedly:
  if (alt_serial.available()) {
    if (gps.encode(alt_serial.read())) {
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
