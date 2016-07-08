#include <Arduino.h>

#define LED1      PB15
#define MOTOR1A   PB0
#define MOTOR1B   PB1
#define MOTOR2A   PB8
#define MOTOR2B   PB9

int dataIn;
uint32_t counter;
uint32_t myindex;
uint16_t motor1, motor2;

void robot_stop(void)
{
  /* stop */
  motor1 = 0;
  analogWrite (MOTOR1B, motor1);
  analogWrite(MOTOR1A, 0);
  
  motor2 = 0;
  analogWrite (MOTOR2B, motor2);
  analogWrite(MOTOR2A, 0);

  delay(200);
}

void robot_forward(void)
{
  /* go forward */
  motor1 = 4095;
  analogWrite (MOTOR1B, motor1);
  analogWrite(MOTOR1A, 0);
  
  motor2 = 4095;
  analogWrite (MOTOR2B, motor2);
  analogWrite(MOTOR2A, 0);

  delay(500);
}

void robot_backward(void)
{
  /* go forward */
  motor1 = 4095;
  analogWrite (MOTOR1B, 0);
  analogWrite(MOTOR1A, motor1);
  
  motor2 = 4095;
  analogWrite (MOTOR2B, 0);
  analogWrite(MOTOR2A, motor2);

  delay(500);
}

void robot_turn_left(void)
{
  /* turn 90 degree to the left */
  motor1 = 4095;
  analogWrite (MOTOR1B, motor1);
  analogWrite(MOTOR1A, 0);
  
  motor2 = 0;
  analogWrite (MOTOR2B, motor2);
  analogWrite(MOTOR2A, 0);

  delay(250);
}

void robot_rotate_left(void)
{
  /* rotate 90 degree to the left */
  motor1 = 4095;
  analogWrite (MOTOR1B, motor1);
  analogWrite(MOTOR1A, 0);
  
  motor2 = 4095;
  analogWrite(MOTOR2B, 0);
  analogWrite (MOTOR2A, motor2);

  delay(135);
}

void robot_turn_right(void)
{
  /* turn 90 degree to the right */
  motor1 = 0;
  analogWrite (MOTOR1B, motor1);
  analogWrite(MOTOR1A, 0);
  
  motor2 = 4095;
  analogWrite (MOTOR2B, motor2);
  analogWrite(MOTOR2A, 0);

  delay(250);
}

void robot_rotate_right(void)
{
  /* rotate 90 degree to the right */
  motor1 = 4095;
  analogWrite(MOTOR1B, 0);
  analogWrite (MOTOR1A, motor1);
  
  motor2 = 4095;
  analogWrite (MOTOR2B, motor2);
  analogWrite(MOTOR2A, 0);

  delay(135);
}

void setup() {
  pinMode(LED1, OUTPUT);

  analogWriteResolution(12);
  analogWrite(MOTOR1A, 0);
  analogWrite(MOTOR1B, 0);
  analogWrite(MOTOR2A, 0);
  analogWrite(MOTOR2B, 0);
  
  Serial.begin(9600); // baudrate is not actually used
  Serial1.begin(9600, SERIAL_HALF_DUPLEX);
  Serial3.begin(1200); // bottom sensor
  Serial4.begin(1200); // top sensor
}

void loop() {
  if (counter++ == 20) {
    counter = 0;
    myindex++;
    digitalWrite(LED1, HIGH);
    delay(100);
    digitalWrite(LED1, LOW);
    delay(100);
    Serial1.print("S1:Hello Receiver!\n");
    Serial1.println(myindex, DEC);
    Serial4.write("S4:Hello Receiver!\n");
    Serial4.println(myindex, DEC);
  }
  
  robot_forward();
  //robot_stop();
  
  robot_backward();
  //robot_stop();
  
  dataIn = Serial4.read();
  if (dataIn >= 0)
  Serial.write(dataIn);
}
