#include <Servo.h>
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

#define Pin13LED  13
Servo left, right;


void setup(){
  left.attach(5);
  right.attach(6);

  left.write(93);
  right.write(90);

  pinMode(2, INPUT_PULLUP);
  digitalWrite(2, HIGH);
  pinMode(3, INPUT_PULLUP);
  digitalWrite(3, HIGH);
  
  Serial.begin(9600);

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

int proximityValue = 0;
void loop(){
  
  proximityValue = readProximity();
  Serial.println(proximityValue);
  
  int rightSpeed = map(proximityValue, 2450, 2600, 90, 0);
  if (proximityValue > 2600) rightSpeed = 0;
  if (proximityValue < 2450) rightSpeed = 70;
  
  int leftSpeed = map(proximityValue, 2600, 3100, 0, 90);
  if (proximityValue < 2600) leftSpeed = 0;
  if (proximityValue > 3100) leftSpeed = 90;
  right.write(leftSpeed);
  left.write(180-rightSpeed);
  
  if(digitalRead(2) == 0){
    left.write(180);
    right.write(85);
    delay(400);
  }
  if(digitalRead(3) == 0){
    left.write(100);
    right.write(160);
    delay(500);    
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

// readAmbient() returns a 16-bit value from the VCNL4000's ambient light data registers
unsigned int readAmbient()
{
  unsigned int data;
  byte temp;

  temp = readByte(COMMAND_0);
  writeByte(COMMAND_0, temp | 0x10);  // command the sensor to perform ambient measure

  while(!(readByte(COMMAND_0)&0x40)) 
    ;  // wait for the proximity data ready bit to be set
  data = readByte(AMBIENT_RESULT_MSB) << 8;
  data |= readByte(AMBIENT_RESULT_LSB);

  return data;
}

