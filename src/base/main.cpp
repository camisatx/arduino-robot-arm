#include <Arduino.h>
#include <Wire.h>       //display

#include <Adafruit_PWMServoDriver.h>  //servo driver
#include <LiquidCrystal.h>  //display
#include <RH_NRF24.h>           //radio
#include <RHReliableDatagram.h> //radio

#include <LcdChar.h>  //LCD characters

//Radio - transceiver
//#define CLIENT_ADDRESS 1  //controller
//#define SERVER_ADDRESS 2  //base
//delcare the radio driver to use
RH_NRF24 driver(48, 53);   //CE, CSN  // mega2560 direct
//RH_NRF24 driver(53, 48);   //CE, CSN  //sainsmart meta2560 servo shield
//class to manage message delivery and receipt, using the driver declared
//RHReliableDatagram manager(driver, SERVER_ADDRESS);

//LCD panel
const int rs=22, en=24, d4=26, d5=28, d6=30, d7=32;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//I2C PCA9687 PWM servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#define SERVO_FREQ  50  //frequency of the servos; 50hz

//waist rotation
#define SERVO_1   0
uint16_t servo1_angle = 1500;
uint16_t servo1_min = 500;
uint16_t servo1_max = 2500;

//shoulder elevation
#define SERVO_2   1
uint16_t servo2_angle = 1200;
uint16_t servo2_min = 800;
uint16_t servo2_max = 1900;

//elbow elevation
#define SERVO_3   2
uint16_t servo3_angle = 1900;
uint16_t servo3_min = 1000;
uint16_t servo3_max = 2400;

//wrist elevation
#define SERVO_4   3
uint16_t servo4_angle = 1500;
uint16_t servo4_min = 700;
uint16_t servo4_max = 2400;

//wrist rotation
#define SERVO_5   4
uint16_t servo5_angle = 1600;
uint16_t servo5_min = 500;
uint16_t servo5_max = 2500;

//gripper open
#define SERVO_6   5
uint16_t servo6_angle = 510;
uint16_t servo6_min = 510;
uint16_t servo6_max = 900;

//Declare the return data: link status, base state
uint8_t data[7];
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

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  //move servos to home position
  pwm.setPWM(SERVO_1, 0, int(float(servo1_angle)/1000000*SERVO_FREQ*4096));
  pwm.setPWM(SERVO_2, 0, int(float(servo2_angle)/1000000*SERVO_FREQ*4096));
  pwm.setPWM(SERVO_3, 0, int(float(servo3_angle)/1000000*SERVO_FREQ*4096));
  pwm.setPWM(SERVO_4, 0, int(float(servo4_angle)/1000000*SERVO_FREQ*4096));
  pwm.setPWM(SERVO_5, 0, int(float(servo5_angle)/1000000*SERVO_FREQ*4096));
  pwm.setPWM(SERVO_6, 0, int(float(servo6_angle)/1000000*SERVO_FREQ*4096));

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
      //end-effector claw
      if (buf[2] == 0) {
        //pinMode(SERVO_6, OUTPUT);
        servo6_angle = servo6_max;   //close
      } else if (buf[2] == 1) {
        //pinMode(SERVO_6, OUTPUT);
        servo6_angle = servo6_min;   //open
      } else {
        //pinMode(SERVO_6, INPUT);
      }
      //wrist rotation
      if (buf[0] < 120 || buf[0] > 135) {
        //pinMode(SERVO_5, OUTPUT);
        int left_joy_x_delta = map(buf[0], 0, 254, -20, 20);
        if (servo5_angle + left_joy_x_delta > servo5_min &&
            servo5_angle + left_joy_x_delta < servo5_max) {
          servo5_angle += left_joy_x_delta;
        }
      } else {
        //pinMode(SERVO_5, INPUT);
      }
      //wrist elevation
      if (buf[1] < 120 || buf[1] > 135) {
        //pinMode(SERVO_4, OUTPUT);
        int left_joy_y_delta = map(buf[1], 0, 254, 20, -20);
        if (servo4_angle + left_joy_y_delta > servo4_min &&
            servo4_angle + left_joy_y_delta < servo4_max) {
          servo4_angle += left_joy_y_delta;
        }
      } else {
        //pinMode(SERVO_4, INPUT);
      }
      //shoulder angle
      if (buf[3] < 120 || buf[3] > 135) {
        //pinMode(SERVO_1, OUTPUT);
        int right_joy_x_delta = map(buf[3], 0, 254, 30, -30);
        if (servo1_angle + right_joy_x_delta > servo1_min &&
            servo1_angle + right_joy_x_delta < servo1_max) {
          servo1_angle += right_joy_x_delta;
        }
      } else {
        //pinMode(SERVO_1, INPUT);
      }
      //shoulder and elbow elevation
      if (buf[4] < 120 || buf[4] > 135) {
        //pinMode(SERVO_2, OUTPUT);
        //pinMode(SERVO_3, OUTPUT);
        int right_joy_y_delta = map(buf[4], 0, 254, 20, -20);
        //if (servo2_angle + right_joy_y_delta > servo2_min &&
        //    servo2_angle + right_joy_y_delta < servo2_max) {
        //  servo2_angle += right_joy_y_delta;
        if (servo3_angle + right_joy_y_delta > servo3_min &&
            servo3_angle + right_joy_y_delta < servo3_max) {
          servo3_angle += right_joy_y_delta;
        }
      } else {
        //pinMode(SERVO_2, INPUT);
        //pinMode(SERVO_3, INPUT);
      }

      if (debug) {
        Serial.print("1: ");
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
        Serial.print(servo6_angle);
        //Serial.print("Response from 0x");
        //Serial.print(from, HEX);
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
        //Serial.print(buf[5]);
        Serial.println();
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
      data[1] = map(servo1_angle, servo1_min, servo1_max, 0, 180);
      data[2] = map(servo2_angle, servo2_min, servo2_max, 0, 180);
      data[3] = map(servo3_angle, servo3_min, servo3_max, 0, 180);
      data[4] = map(servo4_angle, servo4_min, servo4_max, 0, 180);
      data[5] = map(servo5_angle, servo5_min, servo5_max, 0, 180);
      data[6] = map(servo6_angle, servo6_min, servo6_max, 0, 180);
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
  pwm.setPWM(SERVO_1, 0, int(float(servo1_angle)/1000000*SERVO_FREQ*4096));
  pwm.setPWM(SERVO_2, 0, int(float(servo2_angle)/1000000*SERVO_FREQ*4096));
  pwm.setPWM(SERVO_3, 0, int(float(servo3_angle)/1000000*SERVO_FREQ*4096));
  pwm.setPWM(SERVO_4, 0, int(float(servo4_angle)/1000000*SERVO_FREQ*4096));
  pwm.setPWM(SERVO_5, 0, int(float(servo5_angle)/1000000*SERVO_FREQ*4096));
  pwm.setPWM(SERVO_6, 0, int(float(servo6_angle)/1000000*SERVO_FREQ*4096));
}