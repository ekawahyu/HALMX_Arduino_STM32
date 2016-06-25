#include <Arduino.h>

#define LED1   PB15
#define LED2   PB2
#define LED3   PA5
#define LED4   PA4
#define LED5   PB9
#define LED6   PB8
#define LED7   PB3
#define LED8   PA15

int dataIn;
uint32_t counter;
uint32_t myindex;

void setup() {
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED5, OUTPUT);
  pinMode(LED6, OUTPUT);
  pinMode(LED7, OUTPUT);
  pinMode(LED8, OUTPUT);
  
  Serial.begin(9600); // baudrate is not actually used
  Serial1.begin(9600, SERIAL_HALF_DUPLEX);
}

void loop() {
  if (counter++ == 50000) {
    counter = 0;
    myindex++;
    digitalWrite(LED1, HIGH); // turn the LED on (HIGH is the voltage level)
    delay(100);              // wait for a second
    digitalWrite(LED1, LOW);  // turn the LED off by making the voltage LOW
    delay(100);              // wait for a second
    Serial1.write("Hello Sender!\n");
    Serial1.println(myindex, DEC);
  }

  dataIn = Serial1.read();
  if (dataIn >= 0)
  Serial.write(dataIn);
}
