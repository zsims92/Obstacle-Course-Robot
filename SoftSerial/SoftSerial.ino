/*-----( Import needed libraries )-----*/
#include <SoftwareSerial.h>
/*-----( Declare Constants and Pin Numbers )-----*/
#define SSerialRX 10
#define SSerialTX 11
#define Pin13LED  13
/*-----( Declare objects )-----*/
SoftwareSerial mySerial(SSerialRX, SSerialTX); // RX, TX
/*-----( Declare Variables )-----*/
int byteReceived;
int byteSend;

void setup()   /****** SETUP: RUNS ONCE ******/
{
  // Start the built-in serial port, probably to Serial Monitor
  Serial.begin(9600);
  Serial.println("YourDuino.com SoftwareSerial remote loop example");
  Serial.println("Use Serial Monitor, type in upper window, ENTER");
  pinMode(Pin13LED, OUTPUT);    
  
  // Start the software serial port, to another device
  mySerial.begin(9600);   // set the data rate 
}//--(end setup )---


void loop()   /****** LOOP: RUNS CONSTANTLY ******/
{
  digitalWrite(Pin13LED, HIGH);  // Show activity
  if (Serial.available())
  {
    byteReceived = Serial.read();
    mySerial.write(byteReceived);  // Send byte to Remote Arduino
    digitalWrite(Pin13LED, LOW);  // Show activity    
    delay(25);
  }
  
  if (mySerial.available())  //Look for data from other Arduino
   {
    digitalWrite(Pin13LED, HIGH);  // Show activity
    byteSend = mySerial.read();    // Read received byte
    Serial.write(byteSend);        // Show on Serial Monitor
    delay(10);
    digitalWrite(Pin13LED, LOW);  // Show activity   
   }  

}
