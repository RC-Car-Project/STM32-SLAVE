#include <Wire.h>
#include "pin.h"

#include "RF24.h"

#include <Servo.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

HardwareSerial Serial2(PA3, PA2);
TwoWire Wire1(PB9, PB8);
TwoWire Wire2(PB11, PB10);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(1);

// instantiate an object for the nRF24L01 transceiver
RF24 radio(PA11, PA12); 

bool radioNumber = 1;
float payload = 0.0;
uint8_t address[][6] = { "1Node", "2Node" };
long lastSentMsgDelay = 0;
bool nrf24l01_connexion = false;


Servo ESC,SERVO;

uint8_t dist1 = 255, dist2 = 255, dist3 = 255, dist4 = 255;
uint8_t speed = 0;
uint8_t astTh = 0;

bool hasSlave = false;

void setup() {
  pinMode(PC13, OUTPUT);

  digitalWrite(PC13, HIGH);
  delay(100);
  digitalWrite(PC13, LOW);

  init_sensor();
  

  ESC.attach(pin_SERVO_ESC );
  SERVO.attach(pin_SERVO_SERVO);
  

  #if is_test_mode == true
    digitalWrite(PC13, HIGH);
    serial_FTDI.begin(115200);
    delay(500);
    serial_FTDI.println("start debug");
    digitalWrite(PC13, LOW);
  #endif
    delay(500);
    while(!mag.begin(&Wire1, 400000))
    {
      digitalWrite(PC13, HIGH);
      delay(100);
      digitalWrite(PC13, LOW);
      delay(100);
    }

  Wire2.begin();
  Wire2.setClock(400000);

 
  
 

  while (!radio.begin()) {
    digitalWrite(PC13, HIGH);
    delay(30);
    digitalWrite(PC13, LOW);
    delay(30);
  }

  radio.setAutoAck(false);

  radio.setPALevel(RF24_PA_MAX);


  radio.setPayloadSize(sizeof(payload)); 
  radio.openWritingPipe(address[radioNumber]);
  radio.openReadingPipe(1, address[!radioNumber]);

  radio.startListening();

  delay(1000);
  Wire2.beginTransmission(I2C_BUS_Addr);
  if(Wire2.endTransmission() == 0) {
    hasSlave = true;
  } else {
    Wire2.end();
  }

}


long lastMag = 0;
long lastBus = 0;
void loop() {
  if(hasSlave && lastBus + 100 < millis()) {
    
    long start = micros();
    Wire2.requestFrom(I2C_BUS_Addr, 5);
    
      speed = Wire2.read();
      dist1 = Wire2.read();
      dist2 = Wire2.read();
      dist3 = Wire2.read();
      dist4 = Wire2.read();

      
    #if is_test_mode == true
      serial_FTDI.println("new value: dist1="+String(dist1)+" dist2="+String(dist2)+" dist3="+String(dist3)+" dist4="+String(dist4)+" speed="+String(speed)+" t="+String(micros() - start));
    #endif */

    lastBus = millis();
  }
  if(lastBus + 100 < millis()) {
    if(nrf24l01_connexion) {
            radio.stopListening();
            payload = float(speed) + (float(1) / 10);
            radio.write(&payload, sizeof(float));
            radio.startListening();
      }
      lastBus = millis();
  }

  if(lastMag + 250 < millis()) {


    #if is_test_mode == true
      long start = micros();
      sensors_event_t event; 
      mag.getEvent(&event);
    
      float heading = atan2(event.magnetic.y, event.magnetic.x) + 0.22;
      
      if(heading < 0)
        heading += 2*PI;
        
      if(heading > 2*PI)
        heading -= 2*PI;
      
      float headingDegrees = heading * 180/M_PI; 
      
     // Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
      //Serial.println(micros() - start);
    
        
    #endif
    lastMag = millis();
  }

  //nrf24l01_update();
    uint8_t pipe;
    while(radio.available(&pipe)) {              // is there a payload? get the pipe number that recieved it
      uint8_t bytes = radio.getPayloadSize();  // get the size of the payload
      radio.read(&payload, bytes);             // fetch payload from FIFO
      uint16_t value = payload;
      uint8_t type = ((payload - value + 0.01) *10);
      #if is_test_mode == true
        serial_FTDI.println("NRF recieve: type:"+String(type)+" value:"+String(value));
      #endif
      switch(type) {
        case 0:
        {
          lastSentMsgDelay = millis();
          radio.stopListening();
          payload = float(value);
          serial_FTDI.println("sending back");
          bool Ret = radio.write(&payload, sizeof(float));
          radio.startListening();
          if(!nrf24l01_connexion) {
            nrf24l01_connexion = true;
            digitalWrite(PC13, HIGH);
            delay(200);
            digitalWrite(PC13, LOW);
          }
          break;
        }
        case 1:
        {
          serial_FTDI.println("throttle");
          ESC.write(value);
          astTh = value;
          break;
        }
        case 2:
        {
          serial_FTDI.println("steering");
          uint16_t nv = 180 - value;
          SERVO.write(nv);
          break;
        }

      }

    }
    if(millis() - lastSentMsgDelay > 750) {
      ESC.write(90);
      SERVO.write(90);
      nrf24l01_connexion = false;
    } 
    update_sensor();

}

bool nrf24l01_send(uint8_t type, uint16_t value) {
    radio.stopListening();
    payload = float(value) + (float(type) / 10);
    bool Ret = radio.write(&payload, sizeof(float));
    radio.startListening();
    return Ret;
}

bool nrf24l01_s(uint8_t type, uint16_t value) {
    payload = float(value) + (float(type) / 10);
    bool Ret = radio.write(&payload, sizeof(float));
    return Ret;
}

void stopl() {
  radio.stopListening();
}

void startl() {
  radio.startListening();
}

bool nrf_connected() {
  return nrf24l01_connexion;
}

