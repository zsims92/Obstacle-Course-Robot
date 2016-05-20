#include <Servo.h>


Servo left;
Servo right;
int button1 = 4;

void setup(){
  left.attach(9);
  right.attach(10);

  left.write(90);
  right.write(90);

  Serial.begin(9600);

  //Enable the sensors as inputs
  DDRC &= ~((1 << 1) | (1 << 2));

  /*Enable the analog converter*/
  ADMUX &= ~(1 << REFS1);
  ADMUX |= (1 << REFS0);
  ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

  //  pinMode(button1, INPUT_PULLUP);
  //while(digitalRead(button1) == ){
  //}
}
boolean turning = true;
int count;
long timeStart = 0;
void loop(){
  left.write(93);
  right.write(91);
  if(turning){
    test2();
  } 

}


//Test left turn 90 degrees
void test1(){
  timeStart=millis();
  right.write(0);
  while(turning){

    ADMUX &= ~((1 << MUX3) |(1 << MUX2) | (1 << MUX1));
    ADMUX |= (1 << MUX0);

    ADCSRA |= (1 << ADSC);
    while(ADCSRA & 0b1000000){

    }
    int sens2CurrVal = ADC;    

    if(sens2CurrVal > 900){
      turning = false;
      break;
    }

  }
  count = millis();
  Serial.println(count-timeStart);
}

void test2(){
  timeStart = millis();
  left.write(180);
  while(turning){
    ADMUX &= ~((1 << MUX3) | (1 << MUX2) | (1 << MUX1));
    ADMUX |= (1 << MUX0);

    ADCSRA |= (1 << ADSC);

    while(ADCSRA & 0b1000000){

    }
    int sens2CurrVal = ADC;    

    if(sens2CurrVal > 900){
      turning = false;
      break;
    }

  }
  count = millis();
  Serial.println(count-timeStart);
}





