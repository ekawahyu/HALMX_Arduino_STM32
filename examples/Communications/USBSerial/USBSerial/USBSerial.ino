#define LED1  PC6

USBSerial mySerial;
int dataIn;

void setup() {
  pinMode(LED1, OUTPUT);
  mySerial.begin(9600); // baudrate is not actually used
}

void loop() {
  digitalWrite(LED1, HIGH); // turn the LED on (HIGH is the voltage level)
  delay(380);              // wait for a second
  digitalWrite(LED1, LOW);  // turn the LED off by making the voltage LOW
  delay(768);              // wait for a second
  
  mySerial.print("Hello USBSerial on Arduino\n");
  delay(100);
}
