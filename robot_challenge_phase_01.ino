// Calvin Barajas, SacRobotics.com, (CC) 2019
// Note: All "left" and "right" perspectives come from looking at the car from 
// the rear as if you were looking at the rear license plate on a real car and
// as if you were sitting on the driver's seat driving the car.

#include <Servo.h>

//=============== DEFINE VARIABLES ====================

// H-Bridge Motor Controller (L298H)
// Arduino Sensor Shield (ASSH)
int in1 = 18;         // in1 (L298H) corresponds to 18 on UNO and A4 on ASSH - Left DC Motor
int in2 = 17;         // in2 (L298H) corresponds to 17 on UNO and A3 on ASSH - Left DC Motor
int in3 = 16;         // in3 (L298H) corresponds to 16 on UNO and A2 on ASSH - Right DC Motor
int in4 = 15;         // in4 (L298H) corresponds to 15 on UNO and A1 on ASSH - Right DC Motor
int ENA = 5;          // Enable A (ENA) used to control PWM (S5 on ASSH)
int ENB = 6;          // Enable B (ENB) used to control PWM (S6 on ASSH)
int wheelSpeed = 120;  // Ex: 90 with full-charge on batt, 250 with low charge on batt (used for testing).

// HC-SC04 Ultrasonic Proximity Sensor (UPS)
int trigPin = 8;        // UPS output pin (TRIG)
int echoPin = 9;        // UPS input pin (ECHO)
float angle45Right = 0.0;
float angle90Straight = 0.0;
float angle135Left = 0.0;
float angleRight = 0.0;
float angleLeft = 0.0;

// SG90 UPS Servo
int sg90 = 11;        // SG90 Servo connecting to pin 11 on ASSH
int timeDelay = 500;  // Stay at goal position on the SG90 this long
Servo myServo;        // Instantiate the SG90

int x = 1;
int y = 1;
int z = 1;

//==================== SETUP ==========================

void setup()                // Mandatory Function For UNO Boards
 {
  Serial.begin(9600);       // Set serial communication speed
  pinMode(in1, OUTPUT);     // Left DC Motor
  pinMode(in2, OUTPUT);     // Left DC Motor
  pinMode(in3, OUTPUT);     // Right DC Motor
  pinMode(in4, OUTPUT);     // Right DC Motor
  pinMode(ENA,  OUTPUT);    // ENA (Left Motor) on L298H (PWM Speed Control, analogWrite() requires this)
  pinMode(ENB,  OUTPUT);    // ENB (Right Motor) on L298H (PWM Speed Control, analogWrite() requires this)
  pinMode(trigPin, OUTPUT); // UPS output pin (TRIG)
  pinMode(echoPin, INPUT);  // UPS input pin (ECHO)
  myServo.attach(sg90);     // Set pin 11 on ASSH for SG90
 }

//============= LOCOMOTION FUNCTIONS ==================

 /* ------------------------------------------------------------------------- /*
  The following section below is based on the H-Bridge Truth Table.
  Example:
  (Motor A) Input 1: HIGH = Forward
  (Motor A) Input 2: LOW = Forward
  (Motor B) Input 1: HIGH = Forward
  (Motor B) Input 2: LOW = Forward
 /* ------------------------------------------------------------------------- */    

void wheelForward(float a) {          // Motors Rotate Forward
  Serial.print("----------wheelForward");
  Serial.println("");
  digitalWrite(in1, HIGH);          // ON - Forward
  digitalWrite(in2, LOW);           // OFF - Forward
  analogWrite(ENB, wheelSpeed);     // Set the output speed(PWM)
  digitalWrite(in3, HIGH);          // ON - Forward 
  digitalWrite(in4, LOW);           // OFF - Forward 
  analogWrite(ENA, wheelSpeed);     // Set the output speed(PWM)
  delay(a * 50);
 }

void wheelLeft(int d) {             // Rotate Left (Hard Turn)
  Serial.print("----------wheelLeft");
  Serial.println("");
  digitalWrite(in1,LOW);            // OFF - Reverse
  digitalWrite(in2,HIGH);           // ON - Reverse
  analogWrite(ENB, wheelSpeed);     // Set the output speed(PWM)
  digitalWrite(in3,HIGH);           // ON - Forward
  digitalWrite(in4,LOW);            // OFF - Forward
  analogWrite(ENA, wheelSpeed);     // Set the output speed(PWM)
  delay(d * 50);
 }
 
void wheelRight(int e) {            // Rotate Right (Hard Turn)
  Serial.print("----------wheelRight");
  Serial.println("");
  digitalWrite(in1,HIGH);           // ON - Forward
  digitalWrite(in2,LOW);            // OFF - Forward
  analogWrite(ENB, wheelSpeed);     // Set the output speed(PWM)
  digitalWrite(in3,LOW);            // OFF - Reverse
  digitalWrite(in4,HIGH);           // ON - Reverse
  analogWrite(ENA, wheelSpeed);     // Set the output speed(PWM)     
  delay(e * 50);
 }

void softRight(int b)               //Move Right (Soft Turn)
 {
  digitalWrite(in1,HIGH);           // ON - Forward 
  digitalWrite(in2,LOW);            // OFF - Forward 
  analogWrite(ENB, wheelSpeed);     // Set the output speed(PWM)
  digitalWrite(in3,LOW);            // OFF - Stop
  digitalWrite(in4,LOW);            // OFF - Stop
  analogWrite(ENA, wheelSpeed);     // Set the output speed(PWM)
  delay(b * 100);
 }

void softLeft(int c)                //Move Left (Soft Turn)
 {
  digitalWrite(in1,LOW);            // OFF - Stop
  digitalWrite(in2,LOW);            // OFF - Stop
  analogWrite(ENB, wheelSpeed);     // Set the output speed(PWM)
  digitalWrite(in3,HIGH);           // ON - Forward 
  digitalWrite(in4,LOW);            // OFF - Forward 
  analogWrite(ENA, wheelSpeed);     // Set the output speed(PWM)
  delay(c * 100);
 }

void wheelStop(int f) {             //Both Motors Stop
  Serial.print("----------wheelStop");
  Serial.println("");
  digitalWrite(in1,LOW);            // OFF - Stop
  digitalWrite(in2,LOW);            // OFF - Stop
  analogWrite(ENB, wheelSpeed);     // Set the output speed(PWM)
  digitalWrite(in3,LOW);            // OFF - Stop
  digitalWrite(in4,LOW);            // OFF - Stop
  analogWrite(ENA, wheelSpeed);     // Set the output speed(PWM)
  delay(f * 50);
 }

void wheelReverse(int g) {          // Motors Rotate Reverse
  Serial.print("----------wheelReverse");
  Serial.println("");
  digitalWrite(in1,LOW);            // OFF - Reverse
  digitalWrite(in2,HIGH);           // ON - Reverse
  analogWrite(ENB, wheelSpeed);     // Set the output speed(PWM)     
  digitalWrite(in3,LOW);            // OFF - Reverse
  digitalWrite(in4,HIGH);           // ON - Reverse
  analogWrite(ENA, wheelSpeed);     // Set the output speed(PWM)          
  delay(g * 50);     
 }

//=============== MEASURE DISTANCE ====================

float measureDistance(int angle)
 {
  myServo.write(angle);                       // Move to specific angle (e.g., 0°, 45°, 90°, 135°, 180°) on SG90
  delay(timeDelay);                           // Allow motor to reach goal position comfortably
  digitalWrite(trigPin, LOW);                 // Ensure that signal is low before going high to ensure proper reading
  delayMicroseconds(2);                       // Quiescence period
  digitalWrite(trigPin, HIGH);                // Tell UPS to send sonic burst for 10 microseconds (us)
  delayMicroseconds(10);                      // Tell UPS to send sonic burst for 10 microseconds (us)
  digitalWrite(trigPin, LOW);                 // Set voltage back to low to let UPS pickup ECHO
  float timeDifference = pulseIn(echoPin, HIGH)/5.8/10;   // Read time difference and calculate in cm
  Serial.print("timeDifference for ");     // Print statatements used for troubleshooting
  Serial.print(angle);                      // Print statatements used for troubleshooting
  Serial.print(" is the following: ");        // Print statatements used for troubleshooting
  Serial.print(timeDifference);             // Print statatements used for troubleshooting
  Serial.println("");
  return timeDifference;
 }

//================= INFINITE LOOP =====================

void loop() // Mandatory Function For UNO Boards
 {
   if(x == 1){
    delay(5000);
   }
   x++;

   if(y%7 == 0){
    softRight(1.5);
   }
   y++;
   
   wheelForward(7);
   wheelStop(1);
   delay(500);
   
   //wheelStop(1);
   //delay(5000);
   angle90Straight = measureDistance(90);
   Serial.print("__angle90Straight__");
   Serial.println(angle90Straight);
   //delay(1000);
  
  if(angle90Straight > 10) { // if ID:005
   z = 100;
   wheelStop(5);
   delay(500);
   wheelReverse(10);
   wheelStop(5);
   delay(500);
   wheelRight(13);
   wheelStop(3);
   delay(500);
  } // close if ID:005

  while(z == 100) {
   wheelForward(30);
   softLeft(1.5);
  }

 } // close void loop









 
