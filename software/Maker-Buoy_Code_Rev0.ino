/* Maker Buoy Source Code

The MIT License (MIT)

Copyright (c) 2018 Johns Hopkins University Applied Physics Laboratory

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

#include <math.h> 
#include <HardwareSerial.h>
#include <Arduino.h>   // Required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function
#include <Wire.h>

//Adafruit, BSD License, https://github.com/adafruit/Adafruit_MCP9808_Library
#include "Adafruit_MCP9808.h"

//Adafruit, MIT License, https://github.com/adafruit/Adafruit_BNO055
#include <Adafruit_BNO055.h>  //IMU Sensor Library
#include <utility/imumaths.h>  //IMU Math Library

//Adafruit, Apache License 2.0, https://github.com/adafruit/Adafruit_Sensor
#include <Adafruit_Sensor.h>

Adafruit_BNO055 bno = Adafruit_BNO055();

//Adafruit, Lesser GPL, https://github.com/arduino-libraries/RTCZero
#include <RTCZero.h>  //Feather M0 RTC (for standby function only)
RTCZero rtc1;
int Alarm_Time;  //Used for Feather M0 Standby function

// Mikal Hart, Lesser GPL, https://github.com/mikalhart/TinyGPS
#include <TinyGPS.h> 

// Mikal Hart, Lesser GPL, https://github.com/mikalhart/IridiumSBD
#include <IridiumSBD.h> 

//Defines HW Serial on Feather MO
//For SERCOM5 to function with Feather, YOU MUST COMMENT OUT SERCOM5 lines in Adafruit variants/feather_m0/variant.cpp file
Uart Serial2 (&sercom1, 12, 11, SERCOM_RX_PAD_3, UART_TX_PAD_0);
Uart Serial3 (&sercom5, 19, 6, SERCOM_RX_PAD_0, UART_TX_PAD_2);

//Defines HW Serial on Feather MO
void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}

//Defines HW Serial on Feather MO
//For SERCOM5 to function with Feather, YOU MUST COMMENT OUT SERCOM5 lines in Adafruit variants/feather_m0/variant.cpp file
void SERCOM5_Handler()
{
  Serial3.IrqHandler();
}

//Define GPS Interface on Serial 1
TinyGPS gps;
#define ss Serial1

//Define Iridium Interface on Serial 3
#define IridiumSerial Serial3
#define DIAGNOSTICS false // Change this to see diagnostics

// Declare the IridiumSBD object
IridiumSBD isbd(IridiumSerial);

// Create the MCP9808 temperature sensor object
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

//CONSTANTS DECLARATION
const float deg2rad = 0.01745329;

//GLOBAL VARIABLE DECLARATION

int signalQuality = -1;

int reboot_day = 0;
int current_day = 0; //Last Day Up
boolean set_reboot_day = true;

//Default Reporting Interval is 120 min to allow time for battery charging after a reboot
int reporting_interval = 120;

//Default Collection Time is 5 minutes
int collection_time = 5;

float max_tilt = 0;
float max_accelerometer = 0;
float average_accelerometer = 0;
float max_magnetometer = 0;
float average_magnetometer = 0;

int average_counter = 1;

long initial_time = 0;
float initial_latitude = 0;
float initial_longitude = 0;

float latitude = 99;
float longitude = 99;
float velocity = 99;
int angle = 99;

boolean strobe = false;

//Default Feather RTC Date/Time
int gps_year = 1;
byte gps_month = 1;
byte gps_day = 1;
byte gps_hour = 0;
//Minute value of 1 prevents immediate collection attempt after a reboot
byte gps_minute = 1;
byte gps_second = 0;
byte hundredths = 0;

unsigned long fix_age;

boolean store_location = true;

unsigned long last_msg_time = 0;

// Define SBD message format ------------------------------------------

typedef union {

  struct {
    int32_t   latitude;
    int32_t   longitude;
    uint16_t  average_velocity;
    uint16_t  average_direction;
    uint8_t   max_tilt;
    uint16_t  extra_two_byte_1;
    uint8_t   extra_one_byte_1;;
    uint16_t  max_accelerometer;
    uint16_t  average_accelerometer;
    uint16_t  max_magnetometer;
    uint16_t  average_magnetometer;
    uint16_t  max_light;
    uint16_t  extra_two_byte_2;
    uint16_t  extra_two_byte_3;
    uint16_t  extra_two_byte_4;
    uint16_t  extra_two_byte_5;
    uint16_t  extra_two_byte_6;
    int16_t   int_temperature;
    int16_t   extra_two_byte_7;
    uint16_t  battery_voltage;
    uint16_t  extra_two_byte_8;
    uint8_t   extra_one_byte_2;
    uint8_t   reporting_interval;
    uint8_t   collection_time;
    uint8_t   strobe;
    uint16_t  days_since_reboot;
    
  } __attribute__((packed));

  uint8_t bytes[];

} SBDMessage;

SBDMessage message;

//*****************************Setup****************************************

void setup()  
{
  memset(message.bytes, 0x80, sizeof(message));

//Begin Serial for outputting debug info
  Serial.begin(9600);

//Status LED
  pinMode(13, OUTPUT);

//Strobe Pin
  pinMode(5, OUTPUT);

//Watchdog "Petting" Pin
  pinMode(15, OUTPUT);

//GPS and AIS MOSFET
  pinMode(17, OUTPUT);
  digitalWrite(17, LOW);
  
//Iridium MOSFET
  pinMode(18, OUTPUT);
  digitalWrite(18, LOW);

//GPS and AIS MOSFET
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

//Begin BNO055 IMU
  bno.begin();

//Begin MCP9808 Temp Sensor
  tempsensor.begin();

//Figure out what this does!
bno.setExtCrystalUse(true);

//Begin Feather M0 RTC
  rtc1.begin();
  rtc1.setTime(0,1,0);
  rtc1.setDate(1,1,1);

//Begin Ultimate GPS
  ss.begin(9600);

//Required to use hardware serial on Feather M0  
  pinPeripheral(11, PIO_SERCOM);
  pinPeripheral(12, PIO_SERCOM);

}  //End setup()

//*****************************Begin main loop****************************************

void loop()  
{
    FlashStatus();

    if (strobe) {
      digitalWrite(5, HIGH);
    }
    else {
      digitalWrite(5, LOW);
    }

//GPS flag required for speed and direction calculations
    bool newData = false;
    unsigned long age;

//******If time is at collection start time, turn on sensors and collect long duration data*******
  
   if (minutes_past_midnight()%reporting_interval == 0) { 

     //Turn On BNO055
     bno.enterNormalMode();
     delay(100);

     //Turn On GPS and AIS
     digitalWrite(17, HIGH);
     delay(100);  

     //Turn On GPS and AIS
     digitalWrite(13, HIGH);
     delay(100);  

     //Begin GPS
     ss.begin(9600);
     delay(100);  

//****************Start of Long Duration Data Collection Loop**********************

     int start_min = rtc1.getMinutes();

     while(rtc1.getMinutes() <= start_min + collection_time) {  
     
//Read GPS
     while (ss.available())
     {
       char c = ss.read();
       if (gps.encode(c)) // Did a new valid sentence come in?
         newData = true;
     }
     
//Read GPS
    if (newData) {
      gps.f_get_position(&latitude, &longitude, &age);
      gps.crack_datetime(&gps_year, &gps_month, &gps_day, &gps_hour, &gps_minute, &gps_second, &hundredths, &fix_age);  
      velocity = gps.f_speed_knots();
      angle = gps.f_course();
     }

//Store Initial Lat Lon after 2 min of GPS collection--Used to calculate speed and direction during collection interval
  if (newData && rtc1.getMinutes() >= start_min + 2) {
    if (store_location) {
    initial_time = seconds_past_midnight();
    initial_latitude = latitude;
    initial_longitude = longitude;
    store_location = false;
      }
   }

//Get BNO055 IMU Data
  imu::Vector<3> linear_acceleration = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  imu::Vector<3> magnetometer = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

//Compute Tilt, Accel, and Mag 
  float tilt = (1/deg2rad)*acos(gravity.z()/sqrt(pow(gravity.x(),2)+pow(gravity.y(),2)+pow(gravity.z(),2))); 
  float accelerometer_mag = sqrt(pow(linear_acceleration.x(),2) + pow(linear_acceleration.y(),2) + pow(linear_acceleration.z(),2)); 
  float magnetometer_mag = sqrt(pow(magnetometer.x(),2) + pow(magnetometer.y(),2) + pow(magnetometer.z(),2));


//Update Averages for Accel, Mag and Pressure
  average_accelerometer = average_accelerometer + (accelerometer_mag - average_accelerometer)/average_counter; 
  average_magnetometer = average_magnetometer + (magnetometer_mag - average_magnetometer)/average_counter; 
  average_counter++;

//Store min & max values for Tilt, Accel, Mag and Pressure

  if (tilt > max_tilt) {
    max_tilt = tilt;
  }

  if (accelerometer_mag > max_accelerometer) {
    max_accelerometer = accelerometer_mag;
  }     

  if (magnetometer_mag > max_magnetometer) {
    max_magnetometer = magnetometer_mag;
  }     

  //Purpose of check is to force reboot if no Iridium message within last 24 hrs 
  //Known issue that millis are slowed down during sleep; Need to reconsider this approach
  if (millis() - last_msg_time < 86400000) {ResetWatchdog();}
  
  delay(100);

}
//******************End Long Duration Data Collection While Loop*******************

// Set Feather RTC with GPS Time
    rtc1.setDate(gps_day,gps_month,gps_year);
    rtc1.setTime(gps_hour,gps_minute,gps_second);

    if (set_reboot_day) {
      reboot_day = rtc1.getEpoch()/86400L;
      set_reboot_day = false;
    }

    current_day = rtc1.getEpoch()/86400L;

float delta_lat = latitude - initial_latitude;
float delta_lon = longitude - initial_longitude;
float a_portion = sin(delta_lat*deg2rad/2)*sin(delta_lat*deg2rad/2)+cos(initial_latitude*deg2rad)*cos(latitude*deg2rad)*sin(delta_lon*deg2rad/2)*sin(delta_lon*deg2rad/2);
float c_portion = 2*atan2(sqrt(a_portion), sqrt(1-a_portion));
float distance = 3440*c_portion;

float average_velocity = 3600*distance/(seconds_past_midnight() - initial_time);

float average_direction = (1/deg2rad)*atan2(sin(longitude*deg2rad-initial_longitude*deg2rad)*cos(latitude*deg2rad), cos(initial_latitude*deg2rad)*sin(latitude*deg2rad) - sin(initial_latitude*deg2rad)*cos(latitude*deg2rad)*cos(longitude*deg2rad-initial_longitude*deg2rad));

float average_direction_360 = average_direction;

if (average_direction < 0) {
    average_direction_360 = average_direction + 360;
}

float temp_celsius = tempsensor.readTempC();
float temp_fahrenheit = temp_celsius * 9.0 / 5.0 + 32;

//Turn on Iridium MOSFET
   digitalWrite(18, HIGH);
   delay(1000);


//Get Ready to Send Iridium Message

//Connect to Iridium Serial    
   IridiumSerial.begin(19200);

//Required for HW Serial to function on Feather M0
  pinPeripheral(19, PIO_SERCOM_ALT);
  pinPeripheral(6, PIO_SERCOM);

//Begin Iridium Object

    int err = isbd.begin();

//Loop until Iridium begin returns no error--required to deal with spurious cases where begin() returns error
    int IR_counter = 0;
    while(err != 0 && IR_counter <= 4)
    {delay(100);
     err = isbd.begin();
     IR_counter++;
     if (millis() - last_msg_time < 86400000) {ResetWatchdog();}
     }

   err = isbd.getSignalQuality(signalQuality);

//Define binary message for Iridium SBD

   message.latitude = int32_t(latitude*10000 + .5);
   message.longitude = int32_t(longitude*10000 + .5);
   message.average_velocity = uint16_t(average_velocity*10 + .5);
   message.average_direction = uint16_t(average_direction_360);
   message.max_tilt = uint8_t(max_tilt + .5);
   message.max_accelerometer = uint16_t(max_accelerometer*10 +.5);
   message.average_accelerometer = uint16_t(average_accelerometer*10 +.5);
   message.max_magnetometer = uint16_t(max_magnetometer*10 +.5);
   message.average_magnetometer = uint16_t(average_magnetometer*10 +.5);
   message.int_temperature = int16_t(temp_fahrenheit*10 + .5);
   message.battery_voltage = uint16_t(analogRead(A7)*3.3*2*100/1024);
   message.reporting_interval = uint8_t(reporting_interval/15);
   message.collection_time = uint8_t(collection_time);
   
   if (strobe) {message.strobe = uint8_t(1);}
   else {message.strobe = uint8_t(0);}
   
   message.days_since_reboot = uint16_t(current_day - reboot_day);

   uint8_t inBuffer[4] = {0, 0, 0, 0};
   size_t inBuffersize = sizeof(inBuffer);

   if (millis() - last_msg_time < 86400000) {ResetWatchdog();}

//Make three sets of Iridium message attempts--time for each must be less than Watchdog timeout
   boolean send_successful = false;
   int iridium_counter = 0;
   while(!send_successful && iridium_counter <= 2)
   {
    delay(1000);

      err = isbd.sendReceiveSBDBinary(message.bytes, sizeof(message), inBuffer, inBuffersize);

    if (err == 0) {
    send_successful = true;
    }
    iridium_counter++;
  
    if (millis() - last_msg_time < 86400000) {ResetWatchdog();}

    
   }  //End Iridium Send While Loop

    if (send_successful) {
      
    //Process inbound Iridium messages
    //Send HEX values from Rockblock Core
    //InBuffer contains Decimal equivalent

    //Send HEX value from 1 to 96 to specify reporting interval from 15 to 1440 min

    if(inBuffer[0] >= 1 && inBuffer[0] <= 96){
    reporting_interval = inBuffer[0]*15;
    }

    //Send HEX value from 5 to 50 to specify collection time
    
    if(inBuffer[1] >= 5 && inBuffer[1] <= 50){
    collection_time = inBuffer[1];
    } 

    //Send HEX value 01 to turn on Strobe. Any value > 1 turns off

    if (inBuffer[2] == 1){
    strobe = true;}
    else {
    if (inBuffer[2] > 1 ){
    strobe = false;}
    }
    
    last_msg_time = millis();

    }
    

//Reset collection variables
    max_tilt = 0;
    max_accelerometer = 0;
    max_magnetometer = 0;
    average_accelerometer = 0;
    average_magnetometer = 0;
    average_counter = 1;
    store_location = true;

      
} //End Data Collect conditional

   //Turn off MOSFET-driven sensors
   digitalWrite(17, LOW); //GPS & AIS
   digitalWrite(18, LOW); //Iridium
   digitalWrite(13, LOW); //Iridium
   
   //Put BNO055 IMU to sleep
   bno.enterSuspendMode();

   if (millis() - last_msg_time < 86400000) {ResetWatchdog();}
   
   //Put Feather M0 in Standby for 30 sec

   Alarm_Time += 30;
   Alarm_Time = Alarm_Time % 60;
   rtc1.setAlarmSeconds(Alarm_Time);
   rtc1.enableAlarm(rtc1.MATCH_SS);
   rtc1.standbyMode();    // Sleep until next alarm match
   
 }  //*****************************End main loop****************************************


// ***** Fcn to Reset Freetronics Watchdog ****** //

void ResetWatchdog() { 
  digitalWrite(15, HIGH);
  delay(20);
  digitalWrite(15, LOW);
}

// ***** Status light flashes battery level*****

void FlashStatus() { 
  
  int voltage = int(analogRead(A7)*3.3*2*10/1024 + .5);

  if (voltage >= 40) {
  for (int i = 0; i <= 3; i++) {  
  digitalWrite(13, HIGH);
  delay(75);
  digitalWrite(13, LOW);
  delay(250);
  }
  } else if (voltage >= 39) {
  for (int i = 0; i <= 2; i++) {  
  digitalWrite(13, HIGH);
  delay(75);
  digitalWrite(13, LOW);
  delay(250);
  }
  } else if (voltage >= 38) {
  for (int i = 0; i <= 1; i++) {  
  digitalWrite(13, HIGH);
  delay(75);
  digitalWrite(13, LOW);
  delay(250);
  }
  } else {
  digitalWrite(13, HIGH);
  delay(75);
  digitalWrite(13, LOW);
  }
}

unsigned int minutes_past_midnight() {
 return (rtc1.getHours()*60 + rtc1.getMinutes());
}

unsigned int seconds_past_midnight() {
 return (rtc1.getHours()*3600 + rtc1.getMinutes()*60 + rtc1.getSeconds());
}
