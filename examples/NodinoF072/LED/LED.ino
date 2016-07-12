#include <Arduino.h>

#define LED1   PB15
#define LED2   PB2
#define LED3   PA5
#define LED4   PA4
#define LED5   PB9
#define LED6   PB8
#define LED7   PB3
#define LED8   PA15

void setup() {
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED5, OUTPUT);
  pinMode(LED6, OUTPUT);
  pinMode(LED7, OUTPUT);
  pinMode(LED8, OUTPUT);
}

void loop() {
  digitalWrite(LED1, HIGH);
  delay(100);
  digitalWrite(LED2, HIGH);
  delay(100);
  digitalWrite(LED3, HIGH);
  delay(100);
  digitalWrite(LED4, HIGH);
  delay(100);
  digitalWrite(LED5, HIGH);
  delay(100);
  digitalWrite(LED6, HIGH);
  delay(100);
  digitalWrite(LED7, HIGH);
  delay(100);
  digitalWrite(LED8, HIGH);
  delay(100);

  digitalWrite(LED1, LOW);
  delay(100);
  digitalWrite(LED2, LOW);
  delay(100);
  digitalWrite(LED3, LOW);
  delay(100);
  digitalWrite(LED4, LOW);
  delay(100);
  digitalWrite(LED5, LOW);
  delay(100);
  digitalWrite(LED6, LOW);
  delay(100);
  digitalWrite(LED7, LOW);
  delay(100);
  digitalWrite(LED8, LOW);
  delay(100);
}
