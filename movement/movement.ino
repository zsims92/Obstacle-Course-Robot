#include <Servo.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <avr/io.h>
#include <stdlib.h>
#include <math.h>

#define SSerialRX 10
#define SSerialTX 11
#define Pin13LED  13

Servo left;
Servo right;
void sensorCharacteristics();
SoftwareSerial mySerial(SSerialRX, SSerialTX); // RX, TX
/*-----( Declare Variables )-----*/
int byteReceived;
int byteSend;

void setup(){
  left.attach(5);
  right.attach(6);
  left.write(93);
  right.write(90);
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
  DDRC &= ~((1 << 1) | (1 << 2));

  /*Enable the analog converter*/
  ADMUX &= ~(1 << REFS1);
  ADMUX |= (1 << REFS0);
  ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  digitalWrite(13, HIGH);
  delay(1000);
  sensorCharacteristics();
  delay(1000);
  digitalWrite(13, LOW);

  left.write(180);
  right.write(0);
}
int minSensor1Val = 2000, minSensor2Val = 2000, maxSensor1Val = 0, maxSensor2Val = 0;

void sensorCharacteristics(){
  int i = 200;
  for(i = 0; i < 2000; i++){
    ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX1));
    ADMUX |= (1 << MUX0);

    ADCSRA |= (1 << ADSC);

    while(ADCSRA & 0b1000000){

    }
    int sens1CurrVal = ADC;

    if(sens1CurrVal < minSensor1Val)
      minSensor1Val = sens1CurrVal;
    if(sens1CurrVal > maxSensor1Val)
      maxSensor1Val = sens1CurrVal;

    /*Set up to read from ADC0*/
    ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX0) | (1 << MUX1));
    ADCSRA |= (1 << ADSC);

    while(ADCSRA & 0b1000000){

    }
    int sens2CurrVal = ADC;

    if(sens2CurrVal < minSensor2Val)
      minSensor2Val = sens2CurrVal;
    if(sens2CurrVal > maxSensor2Val)
      maxSensor2Val = sens2CurrVal;
  }
}

uint8_t stat, angle;
int counter;

int timeToTurnLeft = 1015;
int timeToTurnRight = 1150;
boolean turning = false, stillTurning, onLine = false;
byte correction;
int oldStat = 2;
uint8_t direct[] = {
  45, 90, 135}
, i=0;
int ambientValue, proximityValue;
long x;
int leftSensor = 15;
int rightSensor = 16;
boolean newCorrection = false, straight = false, stopFlag = false, ignore = false, far = false, moveForward = false;
int reverseSpeed = 0;
boolean reverse = false, finding = false;
int totalAngle = 0, side = 0, oldAngle, oldTotalAngle, maxCounter;
int y = 20, reve = 0, forward = 0;
void loop(){
  //Has Jacobs robot sent me a correction
  if(mySerial.available() && counter == 0){
    correction = (uint8_t) mySerial.read();
    angle = correction & 0b00011111;
    stat = (correction & 0b11100000) >> 5;

    int temp = map(angle, 0, 32, 0, 90);

    if(stopFlag){
      reverse = true;
      reve = map(angle, 0, 31, 80, 200);
      stopFlag = false; 
    }

    if(stat == 2){
      counter = map(angle, 0, 32, 0, timeToTurnLeft);
      totalAngle += temp;
      forward = 300;
      stopFlag = false;
    }
    else if(stat == 1){
      counter = map(angle, 0, 32, 100, timeToTurnRight);
      totalAngle -= temp;
      forward = 300;
      stopFlag = false;
    }
    else if(stat == 6 && !ignore){
      stopFlag = true;
    }
    else if(stat == 7){
      straight = true;
    }
    
    moveForward = true;
    maxCounter = counter;
    newCorrection = false;
    oldAngle = angle;
  }

  if(totalAngle < 0){
    side = 1;
    if(totalAngle < -90)
      totalAngle = -90;
  }
  else if(totalAngle > 0){
    side = 0;
    if(totalAngle > 90)
      totalAngle = 90;
  }
  //Am I still turning?  Has the direction changed?
  if(stopFlag){
    left.write(93);
    right.write(90);
    
  }
  else if(reverse){
    left.write(0);
    right.write(180);
  }
  else if(straight){
    left.write(180);
    right.write(7);
  }
  else if(turning){
    if(stat == 2){
      left.write(93);
      right.write(0);
    }
    else if(stat == 1){
      right.write(90);
      left.write(180);
    }
    stillTurning = true;
    oldStat = stat;
    onLine = false;
    finding = true;
  }
  else if(!turning){
    oldTotalAngle = totalAngle;

    if(onLine){
      followLine();
      totalAngle = 0;
    }
    else{
      findLine();
    }
    stillTurning = false;
  }

}

void followLine(){
  /*Set up to read from ADC1*/
  ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX1));
  ADMUX |= (1 << MUX0);

  ADCSRA |= (1 << ADSC);

  while(ADCSRA & 0b1000000){

  }
  int sens1CurrVal = ADC;

  /*Set up to read from ADC3*/
  ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX0) | (1 << MUX1));
  ADCSRA |= (1 << ADSC);

  while(ADCSRA & 0b1000000){

  }
  int sens2CurrVal = ADC;

  int leftSpeed = map(sens2CurrVal, 48, 830, 90, 0);//minSensor1Val, maxSensor1Val, 90, 0);
  int rightSpeed = map(sens1CurrVal, 30, 930, 90, 0);//minSensor2Val, maxSensor2Val, 90, 0);

  left.write(180 - leftSpeed);
  right.write(rightSpeed);
  /*
  int leftSpeed = map(sens2CurrVal, 0, 1048, 100, 0);
   int rightSpeed = map(sens1CurrVal, 0, 1048, 100, 0);
   */
  Serial.print(sens2CurrVal);
  Serial.print(" ");
  Serial.println(sens1CurrVal);
  /*  
   left.write(180 - leftSpeed);
   right.write(rightSpeed);
   */
}

void findLine(){
  ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX1));
  ADMUX |= (1 << MUX0);

  ADCSRA |= (1 << ADSC);

  while(ADCSRA & 0b1000000){

  }
  int sens1CurrVal = ADC;

  /*Set up to read from ADC2*/
  ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX0) | (1 << MUX1));
  ADMUX |= (1 << MUX1) | (1 << MUX0);

  ADCSRA |= (1 << ADSC);
  while(ADCSRA & 0b1000000){

  }

  int sens2CurrVal = ADC;

  if(sens2CurrVal > 500 || sens1CurrVal > 500){
    onLine = true;
    left.write(180);
    right.write(90);
    return; 
  }  

  else{
    if(side == 0){
      int rightSpeed = map(abs(totalAngle), 0, 90, 20, 80);
      left.write(180);
      right.write(rightSpeed);  
    }
    else if(side == 1){
      int leftSpeed = map(abs(totalAngle), 0, 90, 20, 80);
      left.write(leftSpeed);
      right.write(0);
    }
  }
}

ISR(TIMER2_COMPA_vect) {
  TCNT2=0x00;                // No way without this stupid operation!

  if(reve){
    reve--;
    reverse = true;
  }
  else{
    reverse = false;
    if(counter){
      counter--;
      turning = true;
    }
    else{
      turning = false;
    }
    
    if(forward){
      forward--;
      straight = true;
    }
    else{
      straight = false;
    }
  }
  x++;
}

































