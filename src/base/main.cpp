#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>       //display

#include <LiquidCrystal.h>  //display
#include <RH_NRF24.h>           //radio
#include <RHReliableDatagram.h> //radio

#include <LcdChar.h>  //LCD characters

//Radio - transceiver
//#define CLIENT_ADDRESS 1  //controller
//#define SERVER_ADDRESS 2  //base
//delcare the radio driver to use
//RH_NRF24 driver(48, 53);   //CE, CSN
RH_NRF24 driver(53, 48);   //CE, CSN
//class to manage message delivery and receipt, using the driver declared
//RHReliableDatagram manager(driver, SERVER_ADDRESS);

//LCD panel
const int rs=22, en=24, d4=26, d5=28, d6=30, d7=32;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//Initialize the servos - PCD silkscreen error 8, 9, 10 are actually 11, 12, 13
#define SERVO_1   11  //8
#define SERVO_2   12  //9
#define SERVO_3   13  //10
#define SERVO_4   8   //11
#define SERVO_5   9   //12
#define SERVO_6   10  //13
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;
uint16_t servo1_angle = 1500;   //waist rotation
uint16_t servo2_angle = 900;    //shoulder elevation
uint16_t servo3_angle = 1800;   //elbow elevation
uint16_t servo4_angle = 1800;   //wrist elevation
uint16_t servo5_angle = 1600;   //wrist rotation
uint16_t servo6_angle = 1100;   //gripper open

//Declare the return data: link status, base state
uint8_t data[2];
//Declare the message buffer
uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];

//should debug serial lines be printed?
bool debug = true;

void setup() {
  Serial.begin(9600);

  pinMode(13, OUTPUT);

  //initialize LCD and set up the number of columns and rows:
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Initializing...");
  //create lcd characters
  lcd.createChar(0, CharUp);
  lcd.createChar(1, CharDown);
  lcd.createChar(2, CharLeft);
  lcd.createChar(3, CharRight);
  lcd.createChar(4, CharSignalFull);
  lcd.createChar(5, CharSignalPartial);
  lcd.createChar(6, CharX);

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
  servo1.attach(SERVO_1, 550, 2400);
  servo2.attach(SERVO_2, 600, 2100);
  servo3.attach(SERVO_3, 1000, 2400);
  servo4.attach(SERVO_4, 900, 2400);
  servo5.attach(SERVO_5, 550, 2400);
  servo6.attach(SERVO_6, 1000, 2000);
  //move servos to home position
  servo1.write(servo1_angle);
  servo2.write(servo2_angle);
  servo3.write(servo3_angle);
  servo4.write(servo4_angle);
  servo5.write(servo5_angle);
  servo6.write(servo6_angle);

  Serial.println("Initialized and receiving...");
  lcd.clear();
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
      if (debug) {
        Serial.print("Response from 0x");
        Serial.print(from, HEX);
        Serial.print(" - 1: ");
        Serial.print(servo1_angle);
        Serial.print("\t2: ");
        Serial.print(servo2_angle);
        Serial.print("\t3: ");
        Serial.print(servo3_angle);
        Serial.print("\t4: ");
        Serial.print(servo4_angle);
        Serial.print("\t5: ");
        Serial.print(servo5_angle);
        Serial.print("\t6: ");
        Serial.println(servo6_angle);
        //Serial.print(" - LX:");
        //Serial.print(buf[0]);
        //Serial.print("\tLY:");
        //Serial.print(buf[1]);
        //Serial.print("\tLSW:");
        //Serial.print(buf[2]);
        //Serial.print("\tRX:");
        //Serial.print(buf[3]);
        //Serial.print("\tRY:");
        //Serial.print(buf[4]);
        //Serial.print("\tRSW:");
        //Serial.println(buf[5]);
      }

      //process all servo commands
      if (buf[0] < 120 || buf[0] > 135) {
        int left_joy_x_delta = map(buf[0], 0, 254, -50, 50);
        if (servo5_angle + left_joy_x_delta > 550 &&
            servo5_angle + left_joy_x_delta < 2400) {
          servo5_angle += left_joy_x_delta;
        }
      }
      if (buf[1] < 120 || buf[1] > 135) {
        int left_joy_y_delta = map(buf[1], 0, 254, -50, 50);
        if (servo4_angle + left_joy_y_delta > 1000 &&
            servo4_angle + left_joy_y_delta < 2400) {
          servo4_angle += left_joy_y_delta;
        }
      }
      if (buf[3] < 120 || buf[3] > 135) {
        int right_joy_x_delta = map(buf[3], 0, 254, -50, 50);
        if (servo1_angle + right_joy_x_delta > 550 &&
            servo1_angle + right_joy_x_delta < 2400) {
          servo1_angle += right_joy_x_delta;
        }
      }
      if (buf[4] < 120 || buf[4] > 135) {
        int right_joy_y_delta = map(buf[4], 0, 254, -50, 50);
        if (servo2_angle + right_joy_y_delta > 600 &&
            servo2_angle + right_joy_y_delta < 2100) {
          servo2_angle += right_joy_y_delta;
        }
      }

      //lcd.setCursor(15, 0);
      //lcd.write(byte(4));
      //lcd.setCursor(0, 1);
      //lcd.print("                ");  //clear row
      //lcd.setCursor(0, 1);
      //lcd.print(servo1_angle);
      //lcd.setCursor(4, 1);
      //lcd.print(servo2_angle);
      //lcd.setCursor(8, 1);
      //lcd.print(servo3_angle);
      //lcd.setCursor(12, 1);
      //lcd.print(servo4_angle);

      //send reply back to client
      data[0] = 1;  //connection
      //data[1] = 2;  //status
      data[1] = map(servo1_angle, 544, 2400, 0, 180);
      driver.send(data, sizeof(data));
      driver.waitPacketSent();
      //if (!manager.sendtoWait(data, sizeof(data), from)) {
      //  Serial.print("sendtoWait failed");
      //}
    } else {
      Serial.println("Recv failed");
      lcd.setCursor(15, 0);
      lcd.write(byte(6));
    }
    digitalWrite(13, LOW);
  }
  servo1.writeMicroseconds(servo1_angle);
  servo2.writeMicroseconds(servo2_angle);
  servo3.writeMicroseconds(servo3_angle);
  servo4.writeMicroseconds(servo4_angle);
  servo5.writeMicroseconds(servo5_angle);
  servo6.writeMicroseconds(servo6_angle);
}