#include <Arduino.h>

#define LED1  PC6

int dataIn;

void setup() {
  pinMode(LED1, OUTPUT);
  Serial.begin(9600); // baudrate is not actually used
}

void loop() {
  digitalWrite(LED1, HIGH); // turn the LED on (HIGH is the voltage level)
  //delay(10);              // wait for a second
  digitalWrite(LED1, LOW);  // turn the LED off by making the voltage LOW
  //delay(10);              // wait for a second
  
  //Serial.println("abcdefghijklmnopqrstuvwxyz0123456789ABCDEFGHIJKLMNOPQRSTUVW");
  //delay(2);
  dataIn = Serial.read();
  if (dataIn >= 0) Serial.write(dataIn);
}
