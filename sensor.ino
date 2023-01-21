#include "pin.h"
#include <TinyGPS++.h>

TinyGPSPlus gps;

void init_sensor() {
  serial_SIM.begin(9600);
  pinMode(sensor_VIB, INPUT);
  pinMode(sensor_PHOTO, INPUT);
  pinMode(sensor_CJMCU, INPUT);
  pinMode(sensor_LM35, INPUT);
  pinMode(sensor_TENSION, INPUT);
}

long lastSensor = 0;
uint8_t LM35 = 0;
uint8_t current = 0;
uint8_t photo = 0;
uint8_t vibration= 0;
uint8_t tension= 0;

const float VCC   = 3.3;
const int model = 0; 

float cutOffLimit = 1.00;

float sensitivity[] ={
          40.0,// for ACS758LCB-050B
          60.0,// for ACS758LCB-050U
          20.0,// for ACS758LCB-100B
          40.0,// for ACS758LCB-100U
          13.3,// for ACS758KCB-150B
          16.7,// for ACS758KCB-150U
          10.0,// for ACS758ECB-200B
          20.0,// for ACS758ECB-200U     
         }; 

float quiescent_Output_voltage [] ={
          0.5,// for ACS758LCB-050B
          0.12,// for ACS758LCB-050U
          0.5,// for ACS758LCB-100B
          0.12,// for ACS758LCB-100U
          0.5,// for ACS758KCB-150B
          0.12,// for ACS758KCB-150U
          0.5,// for ACS758ECB-200B
          0.12,// for ACS758ECB-200U            
          };

const float FACTOR = sensitivity[model]/1000;// set sensitivity for selected model
const float QOV =   quiescent_Output_voltage [model] * VCC;// set quiescent Output voltage for selected model
float voltage;// internal variable for voltage
float cutOff = FACTOR/cutOffLimit;
float longitude = 0;
float latitude = 0;

void update_sensor() {
   if(millis() - lastSensor > 245) {  

    LM35 = (analogRead(sensor_LM35)*330)/1023;


    float voltage_raw =   (5.0 / 1023.0)* analogRead(sensor_CJMCU);// Read the voltage from sensor
    voltage =  voltage_raw - QOV + 0.007 ;// 0.007 is a value to make voltage zero when there is no current
    current = voltage / FACTOR - 20;



    vibration = speed + random(0, 30);

    while (serial_SIM.available() > 0)
    {
      gps.encode(serial_SIM.read());
    }
    if (gps.location.isValid()) {
      latitude = gps.location.lat();
      longitude = gps.location.lng();
    }

    if(nrf24l01_connexion) {
          radio.stopListening();
          payload = float(LM35) + 0.2;
          radio.write(&payload, sizeof(float));

          payload = float(current) + 0.3;
          radio.write(&payload, sizeof(float));

          payload = float(vibration) + 0.4;
          radio.write(&payload, sizeof(float));

          payload = float(tension) + 0.5;
          radio.write(&payload, sizeof(float));

          payload = float(longitude*10000000000) + 0.6;
          radio.write(&payload, sizeof(float));
          
          payload = float(latitude*10000000000) + 0.7;
          radio.write(&payload, sizeof(float));

          radio.startListening();
    }

    #if is_test_mode == true
      serial_FTDI.println("LM35:"+String(LM35) +" CURRENT:"+String(current)+" VIBRA:"+String(vibration)+" TENSION:"+String(tension)+" LAT:"+String(latitude)+" LONG:"+String(longitude));
    #endif
    lastSensor=millis();

   }
}