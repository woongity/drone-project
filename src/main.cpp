/*
 *  ESC Calibration Code
 *  dronix.kr
 */
 
#include <Servo.h>
 
#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000

#define ESC_PIN_A 9
#define ESC_PIN_B 6
#define ESC_PIN_C 5
#define ESC_PIN_D 3
 
Servo a, b, c, d;
 
void setup() {
  Serial.begin(9600);
  Serial.println("Program begin...");
  Serial.println("This program will calibrate the ESC.");

  a.attach(ESC_PIN_A);
  b.attach(ESC_PIN_B);
  c.attach(ESC_PIN_C);
  d.attach(ESC_PIN_D);

 
  Serial.println("Now writing maximum output.");
  Serial.println("Turn on power source, then wait 2 seconds and press any key.");
  

  a.writeMicroseconds(MAX_SIGNAL);
  b.writeMicroseconds(MAX_SIGNAL);
  c.writeMicroseconds(MAX_SIGNAL);
  d.writeMicroseconds(MAX_SIGNAL);
 
  // Wait for input
  while (!Serial.available());
  Serial.read();
 
  // Send min output
  Serial.println("Sending minimum output");
  
  a.writeMicroseconds(MIN_SIGNAL);
  b.writeMicroseconds(MIN_SIGNAL);
  c.writeMicroseconds(MIN_SIGNAL);
  d.writeMicroseconds(MIN_SIGNAL);


  // Finish Message
  Serial.println("Done");
 
}
 
void loop() {  
 
}