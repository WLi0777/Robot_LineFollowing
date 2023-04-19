#include <SPI.h>
#include "RF24.h"
#include <Wire.h>

RF24 rf24(9,10); // CE, CSN

const byte addr[6] = "10003";


float Ax,Ay,Az,T,Gx,Gy,Gz;

float anglex,angley,anglez;

void setup() {
  // nRF24
  rf24.begin();
  rf24.setChannel(103);       
  rf24.openWritingPipe(addr); 
  rf24.setPALevel(RF24_PA_MAX);   
  rf24.setDataRate(RF24_2MBPS); 
  rf24.stopListening();
  
  //mpu6050
  Wire.begin();        // join i2c bus (address optional for master)
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Wire.beginTransmission(0x68); 
  Wire.write(0x1C); 
  Wire.requestFrom(0x68, 1, true);
  unsigned char acc_conf = Wire.read();
  acc_conf = ((acc_conf & 0xE7) | (3 << 3));
  Wire.write(acc_conf);
  Wire.endTransmission(true); 

  Wire.beginTransmission(0x68); 
  Wire.write(0x1B); 
  Wire.requestFrom(0x68, 1, true);
  unsigned char abc_conf = Wire.read();
  abc_conf = ((abc_conf & 0xE7) | (0 << 3));
  Wire.write(abc_conf);
  Wire.endTransmission(true); 

  Serial.begin(9600); 
}

void loop() {
  //mpu6050
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.requestFrom(0x68, 14,true);
  Wire.endTransmission(true);

  Ax=(Wire.read() <<8 | Wire. read())/209;
  Ay=(Wire.read() <<8 | Wire. read())/209;
  Az=(Wire.read() <<8 | Wire. read())/209;
  T=Wire.read() <<8 | Wire. read();
  Gx=(Wire.read() <<8 | Wire. read())/131;
  Gy=(Wire.read() <<8 | Wire. read())/131;
  Gz=(Wire.read() <<8 | Wire. read())/131;

  anglex=(atan(sqrt(Ax*Ax+Ay*Ay)/Az))*180/3.14;
  angle=(atan(sqrt(Ax*Ax+Ay*Ay)/Az))*180/3.14;
  angle=(atan(sqrt(Ax*Ax+Ay*Ay)/Az))*180/3.14;
    

  //Print the results (first byte is Status register and is ignored)
  
  Serial.print("Ax= "); Serial.print(Ax);
  Serial.print(" | Ay= "); Serial.print(Ay);
  Serial.print(" | Az= "); Serial.println(Az);
  //Serial.print(" | T= "); Serial.print(T);
  Serial.print("Gx= "); Serial.print(Gx);
  Serial.print(" | Gy= "); Serial.print(Gy);
  Serial.print(" | Gy= "); Serial.println(Gz);
  Serial.print("inclination="); Serial.println(angle);
  
 
  //nRF24

  
  rf24.write(&angle,16);  
  delay(1000);
}
