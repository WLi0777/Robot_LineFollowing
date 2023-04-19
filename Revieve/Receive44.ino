#include <SPI.h>
#include "RF24.h"
#include <Wire.h> 
#include <LiquidCrystal.h>

RF24 rf24(9,10); // CE, CSN
float data;
float implementation;
const byte addr[] = "10003";
const byte pipe = 1;
const int rs = 4, en = 3, d4 = 2, d5 = 6, d6 = 7, d7 = 8;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
  Serial.begin(9600);
  rf24.begin();
  rf24.setChannel(103);
  rf24.setPALevel(RF24_PA_MAX);
  rf24.setDataRate(RF24_2MBPS);
  rf24.openReadingPipe(pipe, addr);
  rf24.startListening();
  Serial.println("nRF24L01 ready!");
    // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.setCursor(0, 0);
  lcd.print("implementation");
}
void loop() {
  rf24.read(&data, sizeof(data));
  delay(1000);
  lcd.setCursor(0, 1);
  lcd.print(data);
  Serial.println(data);
}
