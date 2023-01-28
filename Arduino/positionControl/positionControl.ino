

#include <Arduino.h>
#define PROVIDE_ONLY_LINEAR_MOVEMENT
#include "ServoEasing.hpp"

// Motion profile
ServoEasing s1;
ServoEasing s2;
ServoEasing s3;
ServoEasing s4;
ServoEasing s5;
ServoEasing s6;
ServoEasing s7; 
//ServoEasing servo8;
// --- Define global variables 
int left = 215;
int right = -42;
int leftN = 226;
int rightN = -42;
//nt left = 180;
//int right = 0;
int a1 = map(90,right,left,0,180);
int a2 = map(90,right,left,0,180);
int a3 = map(90,right,left,0,180);
int a4 = map(90,right,left,0,180);
int a5 = map(90,right,left,0,180);
int a6 = map(90,right,left,0,180);
int a7 = map(90,right,left,0,180);
int a8 = map(90,right,left,0,180);
void setup() {
  Serial.begin(9600);
// Pins in arduino
  s1.attach(3,a1);
  s2.attach(4,a2);
  s3.attach(5,a3);
  s4.attach(6,a4);
  s5.attach(7,a5);
  s6.attach(8,a6);
  s7.attach(9,a7); 
  //servo8.attach(10,a8); 
  delay(500);
 Serial.println("ready");
}


void loop() {

   if(Serial.available()>0) {
  String command = Serial.readString();
  int i1 = command.indexOf(',');
  int i2 = command.indexOf(',', i1+1);
  int i3 = command.indexOf(',', i2+1);
  int i4 = command.indexOf(',', i3+1);
  int i5 = command.indexOf(',', i4+1);
  int i6 = command.indexOf(',', i5+1);
  int i7 = command.indexOf(',', i6+1);
  int i8 = command.indexOf(',', i7+1);

  String c1 = command.substring(0, i1);
  String c2 = command.substring(i1 + 1, i2);
  String c3 = command.substring(i2 + 1, i3);
  String c4 = command.substring(i3 + 1, i4);
  String c5 = command.substring(i4 + 1, i5);
  String c6 = command.substring(i5 + 1, i6);
  String c7 = command.substring(i6 + 1, i7);
  String c8 = command.substring(i7 + 1, i8);

  a1 = map(c1.toInt(),right,left,0,180);
  a2 = map(c2.toInt(),rightN,leftN,0,180);
  a3 = map(c3.toInt(),rightN,leftN,0,180);
  a4 = map(c4.toInt(),right,left,0,180);
  a5 = map(c5.toInt(),rightN,leftN,0,180);
  a6 = map(c6.toInt(),right,left,0,180);
  a7 = map(c7.toInt(),right,left,0,180);
  a8 = map(c8.toInt(),right,left,0,180);
  
 // Update the servo position at regular intervals
  setSpeedForAllServos(20);
  s1.setEaseTo(a1);
  s2.setEaseTo(a2);
  s3.setEaseTo(a3);
  s4.setEaseTo(a4);
  s5.setEaseTo(a5);
  s6.setEaseTo(a6);
  s7.startEaseTo(a7);
  while (ServoEasing::areInterruptsActive()) {


   }
   
  Serial.println("ready");
  }
  
}
