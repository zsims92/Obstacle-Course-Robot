#include <Servo.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <avr/io.h>
#include <stdlib.h>
#include <math.h>

#define VCNL4000_ADDRESS 0x13  // 0x26 write, 0x27 read

// VCNL4000 Register Map
#define COMMAND_0 0x80  // starts measurments, relays data ready info
#define PRODUCT_ID 0x81  // product ID/revision ID, should read 0x11
#define IR_CURRENT 0x83  // sets IR current in steps of 10mA 0-200mA
#define AMBIENT_PARAMETER 0x84  // Configures ambient light measures
#define AMBIENT_RESULT_MSB 0x85  // high byte of ambient light measure
#define AMBIENT_RESULT_LSB 0x86  // low byte of ambient light measure
#define PROXIMITY_RESULT_MSB 0x87  // High byte of proximity measure
#define PROXIMITY_RESULT_LSB 0x88  // low byte of proximity measure
#define PROXIMITY_FREQ 0x89  // Proximity IR test signal freq, 0-3
#define PROXIMITY_MOD 0x8A  // proximity modulator timing

#define SSerialRX 10
#define SSerialTX 11
#define Pin13LED  13
Servo left, right, front;

uint8_t stat, angle, correction, i=0, direct[] = {
  50,130};
int counter, ambientValue, proximityValue, timeToTurnLeft = 1015, timeToTurnRight = 1050, leftSensor= 15, rightSensor = 16, byteReceived, byteSend, button = 3;
boolean straight = false, stopFlag = false, turning = false, onLine = false;
int distanceFollow = 2650, maxProximity = 2800, minProximity = 2500, minSensor1Val = 2000, minSensor2Val = 2000, maxSensor1Val = 0, maxSensor2Val = 0;
SoftwareSerial mySerial(SSerialRX, SSerialTX); // RX, TX


void setup(){
  left.attach(5);
  right.attach(6);
  front.attach(3);

  left.write(93);
  right.write(90);
  front.write(50);

  pinMode(Pin13LED, OUTPUT);

  Serial.begin(9600);
  cli();
  TCNT2=0x00;
  OCR2A = 250;              // 1 ms @ Fosc = 16 MHz
  TCCR2A= 0x02;              // WGM: No wave generation
  TCCR2B= 0x04;              // START Timer, prescaler = 64
  TIMSK2 = (1 << OCIE2A); 
  sei();

  mySerial.begin(9600);

  //Enable the sensors as inputs
  DDRC &= ~((1 << 1) | (1 << 0));

  /*Enable the analog converter*/
  ADMUX &= ~(1 << REFS1);
  ADMUX |= (1 << REFS0);
  ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

  Wire.begin();
  byte temp = readByte(PRODUCT_ID);
  if (temp != 0x11)  // Product ID Should be 0x11
  {
    Serial.print("Something's wrong. Not reading correct ID: 0x");
    Serial.println(temp, HEX);
  }
  else
    Serial.println("VNCL4000 Online...");

  /* Now some VNCL400 initialization stuff
   Feel free to play with any of these values, but check the datasheet first!*/
  writeByte(AMBIENT_PARAMETER, 0x0F);  // Single conversion mode, 128 averages
  writeByte(IR_CURRENT, 20);  // Set IR current to 200mA
  writeByte(PROXIMITY_FREQ, 2);  // 781.25 kHz
  writeByte(PROXIMITY_MOD, 0x81);  // 129, recommended by Vishay

  left.write(180);
  right.write(0);
}

void loop(){
  front.write(120);
  int follow = 2650;
  proximityValue = readProximity();
  proximityValue = constrain(proximityValue, 0, 2850);
  Serial.println(proximityValue);
  if(proximityValue == 2650){// - 10) || proximityValue < (distanceFollow + 10)){
    left.write(180);
    right.write(0);
     Serial.println(follow);
  }
  else if(proximityValue < (2650 - 10)){
    int speedSlowingWheel = map(proximityValue, 2650, maxProximity, 0, 90);
    left.write(180);
    right.write(speedSlowingWheel);

    /*
    if(front.read() == 50 ){
     int speedSlowingWheel = map(proximityValue, distanceFollow, maxProximity, 0, 90);
     left.write(180);
     right.write(speedSlowingWheel);
     }
     else if(front.read() == 130){
     int speedSlowingWheel = map(proximityValue, distanceFollow, maxProximity, 0, 90);
     left.write(180 - speedSlowingWheel);
     right.write(0);
     }*/
  }
  else if(proximityValue > (2650 + 10)){
    int speedSlowingWheel = map(proximityValue, minProximity, 2650, 90, 0);
    left.write(180 - speedSlowingWheel);
    right.write(0);

    /*    if(front.read() == 50 ){
     int speedSlowingWheel = map(proximityValue, minProximity, distanceFollow, 90, 0);
     left.write(180 - speedSlowingWheel);
     right.write(0);
     }
     else if(front.read() == 130){
     int speedSlowingWheel = map(proximityValue, minProximity, distanceFollow, 90, 0);
     left.write(180);
     right.write(speedSlowingWheel);
     }*/
  }
}

unsigned int readProximity()
{
  unsigned int data;
  byte temp;

  temp = readByte(COMMAND_0);
  writeByte(COMMAND_0, temp | 0x08);  // command the sensor to perform a proximity measure

  while(!(readByte(COMMAND_0)&0x20)) 
    ;  // Wait for the proximity data ready bit to be set
  data = readByte(PROXIMITY_RESULT_MSB) << 8;
  data |= readByte(PROXIMITY_RESULT_LSB);

  return data;
}

// writeByte(address, data) writes a single byte of data to address
void writeByte(byte address, byte data)
{
  Wire.beginTransmission(VCNL4000_ADDRESS);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}

// readByte(address) reads a single byte of data from address
byte readByte(byte address)
{
  byte data;

  Wire.beginTransmission(VCNL4000_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(VCNL4000_ADDRESS, 1);
  while(!Wire.available())
    ;
  data = Wire.read();

  return data;
}


