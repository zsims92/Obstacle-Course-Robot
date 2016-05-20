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
  Serial.println("SerialRemote");  // Can be ignored
  pinMode(Pin13LED, OUTPUT);   
  
  // Start the software serial port, to another device
  mySerial.begin(9600);   // set the data rate 
}//--(end setup )---


void loop()   /****** LOOP: RUNS CONSTANTLY ******/
{
  //Copy input data to output  
  if (mySerial.available()) 
  {
    byteSend = mySerial.read();   // Read the byte 
    
    digitalWrite(Pin13LED, HIGH);  // Show activity
    delay(25);              
    digitalWrite(Pin13LED, LOW);   
    Serial.write(byteSend);
    mySerial.write(byteSend); // Send the byte back
//    delay(100);
  }// End If MySerialAvailable
  
}//--(end main loop )---

/*-----( Declare User-written Functions )-----*/
//NONE
