#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>       //display

#include <LiquidCrystal.h>  //display
#include <RH_NRF24.h>           //radio
#include <RHReliableDatagram.h> //radio

//Radio - transceiver
//#define SERVER_ADDRESS 2  //base
//delcare the radio driver to use
//RH_NRF24 driver(48, 53);   //CE, CSN
RH_NRF24 driver(53, 48);   //CE, CSN
//class to manage message delivery and receipt, using the driver declared
//RHReliableDatagram manager(driver, SERVER_ADDRESS);

//LCD panel
const int rs=22, en=24, d4=26, d5=28, d6=30, d7=32;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//Initialize the servos
#define SERVO_1   7
Servo servo1;
int servo1_angle = 90;

//Declare the return data: link status, base state
uint8_t data[2];
//Declare the message buffer
uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];

void setup() {
  Serial.begin(9600);

  pinMode(13, OUTPUT);

  //initialize LCD and set up the number of columns and rows:
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Initializing...");

  //initialize nrf24 object
  if (!driver.init())
  //if (!manager.init())
    Serial.println("Radio init failed");
    lcd.clear();
    lcd.print("Radiot init failed");
  //defaults after init are 2.402 Ghz (channel 2), 2Mbps, 0dBm
  //nrf24.setChannel(1);
    //Serial.println("nrf24 set channel failed");
  //nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm);
    //Serial.println("nrf24 setRF failed");

  //attach the servos
  servo1.attach(SERVO_1);
  servo1.write(servo1_angle);

  Serial.println("Initialized and receiving...");
  lcd.clear();
  lcd.print("LK:");
}

void loop() {
  if (driver.available()) {
  //if (manager.available()) {
    digitalWrite(13, HIGH);
    //wait for message addressed to us from the client
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (driver.recv(buf, &len)) {
    //if (manager.recvfromAck(buf, &len, &from)) {
      Serial.print("Response from 0x");
      Serial.print(from, HEX);
      Serial.print(" - LX:");
      Serial.print(buf[0]);
      Serial.print("\tLY:");
      Serial.print(buf[1]);
      Serial.print("\tLSW:");
      Serial.print(buf[2]);
      Serial.print("\tRX:");
      Serial.print(buf[3]);
      Serial.print("\tRY:");
      Serial.print(buf[4]);
      Serial.print("\tRSW:");
      Serial.print(buf[5]);

      lcd.setCursor(4, 0);
      lcd.print("            ");  //clear row
      lcd.setCursor(4, 0);
      lcd.print("LIVE");
      lcd.setCursor(0, 1);
      lcd.print("                ");  //clear row
      lcd.setCursor(0, 1);
      lcd.print(buf[0]);
      lcd.print(";");
      lcd.print(buf[1]);
      lcd.print(";");
      lcd.print(buf[3]);
      lcd.print(";");
      lcd.print(buf[4]);

      if (buf[0] < 124 || buf[0] > 130) {
        int left_joy_x_delta = map(buf[0], 0, 255, -8, 8);
        if (servo1_angle + left_joy_x_delta > 0 &&
            servo1_angle + left_joy_x_delta < 180) {
          servo1_angle += left_joy_x_delta;
        }
      }
      Serial.print("\t");
      Serial.println(servo1_angle);

      data[0] = 1;
      data[1] = 2;

      //send reply back to client
      data[0] = 1;  //connection
      //data[1] = 2;  //status
      driver.send(data, sizeof(data));
      driver.waitPacketSent();
      //if (!manager.sendtoWait(data, sizeof(data), from)) {
      //  Serial.print("sendtoWait failed");
      //}
    } else {
      Serial.println("Recv failed");
      lcd.setCursor(6, 0);
      lcd.print("          ");  //clear row
      lcd.setCursor(6, 0);
      lcd.print("FAILED");
    }
    digitalWrite(13, LOW);
  //} else {
  //  Serial.println("no signal");
  //  lcd.setCursor(4, 0);
  //  lcd.print("            ");  //clear row
  //  lcd.setCursor(4, 0);
  //  lcd.print("NO SIGNAL");
  //  delay(1000);
  }
  servo1.write(servo1_angle);
}