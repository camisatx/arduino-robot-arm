#include <Arduino.h>
#include <SPI.h>        //radio
#include <Wire.h>       //display

#include <LiquidCrystal_I2C.h>  //display
#include <RH_NRF24.h>           //radio
#include <RHReliableDatagram.h> //radio

#include <LcdChar.h>  //LCD characters

//Radio - transceiver
#define CLIENT_ADDRESS 1  //controller
#define SERVER_ADDRESS 2  //base
//delcare the radio driver to use
RH_NRF24 driver(8, 7); // CE, CSN
//class to manage message delivery and receipt, using the driver declared
//RHReliableDatagram manager(driver, CLIENT_ADDRESS);

//LCD panel
LiquidCrystal_I2C lcd(0x3F, 16, 2);

//Left joystick
#define LEFT_JOY_SW  3
#define LEFT_JOY_X   A2
#define LEFT_JOY_Y   A3
//Right joystick
#define RIGHT_JOY_SW  2
#define RIGHT_JOY_X   A0
#define RIGHT_JOY_Y   A1
//Joystick buffer zones; ignore analog values between these
#define JOY_BUF_LOW    472    // 1024/2 - 30
#define JOY_BUF_HIGH   552    // 1024/2 + 30

//Declare the data array: LSW, LX, LY, RSW, RX, RY
uint8_t data[6];
uint8_t dataOld[6];
//Define the message buffer
uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];

//Keep track of time between receiver responses
uint32_t time;

//should debug serial lines be printed?
bool debug = false;

//declare functions
void checkButtons();
void checkJoySticks();
void sendData();

void setup() {
  Serial.begin(9600);

  pinMode(13, OUTPUT);
  
  // initialize LCD and set up the number of columns and rows:
  lcd.init();
  lcd.backlight();
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

  //initialize manager object
  if (!driver.init())
  //if (!manager.init())
    Serial.println("Radio init failed");
    lcd.clear();
    lcd.print("Radio init failed");
  //defaults after init are 2.402 Ghz (channel 2), 2Mbps, 0dBm
  //nrf24.setChannel(1);
    //Serial.println("nrf24 set channel failed");
  //nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm);
    //Serial.println("nrf24 setRF failed");

  //initialize the joystick buttons
  pinMode(LEFT_JOY_SW, INPUT);
  digitalWrite(LEFT_JOY_SW, HIGH);
  pinMode(RIGHT_JOY_SW, INPUT);
  digitalWrite(RIGHT_JOY_SW, HIGH);

  time = millis();  //initialize the time

  Serial.println("Initialized");
  lcd.clear();
  lcd.print("L:");
  lcd.setCursor(7, 0);
  lcd.print("R:");
  lcd.setCursor(0, 1);
  lcd.print("ST:");
}

void loop() {
  checkJoySticks();
  checkButtons();
}

void checkButtons() {
  data[2] = digitalRead(LEFT_JOY_SW);
  data[5] = digitalRead(RIGHT_JOY_SW);

  //only send data if something changed
  if (data[2] != dataOld[2] || data[5] != dataOld[5]) {
    sendData();
  }
  //replace the dataOld items the current data values
  for (int i=0; i<6; ++i) {
    dataOld[i] = data[i];
  }
}

void checkJoySticks() {
  //read potentiometers from joysticks

  //left joystick x
  int left_joy_x_read = analogRead(LEFT_JOY_X);
  if (left_joy_x_read < JOY_BUF_LOW || left_joy_x_read > JOY_BUF_HIGH) {
    //convert analog 0-1024 to byte 0-255
    data[0] = max(1, left_joy_x_read >> 2);
    lcd.setCursor(3, 0);
    if (data[0] < 127) {
      lcd.write(byte(2));
    } else {
      lcd.write(byte(3));
    }
  } else {
    data[0] = 127;
    lcd.setCursor(3, 0);
    lcd.print(" ");
  }

  //left joystick y
  int left_joy_y_read = analogRead(LEFT_JOY_Y);
  if (left_joy_y_read < JOY_BUF_LOW || left_joy_y_read > JOY_BUF_HIGH) {
    data[1] = max(1, left_joy_y_read >> 2);
    lcd.setCursor(5, 0);
    if (data[1] < 127) {
      lcd.write(byte(1));
    } else {
      lcd.write(byte(0));
    }
  } else {
    data[1] = 127;
    lcd.setCursor(5, 0);
    lcd.print(" ");
  }
  
  //right joystick x
  int right_joy_x_read = analogRead(RIGHT_JOY_X);
  if (right_joy_x_read < JOY_BUF_LOW || right_joy_x_read > JOY_BUF_HIGH) {
    data[3] = max(1, right_joy_x_read >> 2);
    lcd.setCursor(10, 0);
    if (data[3] < 127) {
      lcd.write(byte(2));
    } else {
      lcd.write(byte(3));
    }
  } else {
    data[3] = 127;
    lcd.setCursor(10, 0);
    lcd.print(" ");
  }

  //right joystick y
  int right_joy_y_read = analogRead(RIGHT_JOY_Y);
  if (right_joy_y_read < JOY_BUF_LOW || right_joy_y_read > JOY_BUF_HIGH) {
    data[4] = max(1, right_joy_y_read >> 2);
    lcd.setCursor(12, 0);
    if (data[4] < 127) {
      lcd.write(byte(1));
    } else {
      lcd.write(byte(0));
    }
  } else {
    data[4] = 127;
    lcd.setCursor(12, 0);
    lcd.print(" ");
  }

  if (debug) {
    //Display the joystick data
    Serial.print("LX: ");
    Serial.print(data[0]);
    Serial.print("\tLY: ");
    Serial.print(data[1]);
    Serial.print("\tLSW: ");
    Serial.print(data[2]);
    Serial.print("\tRX: ");
    Serial.print(data[3]);
    Serial.print("\tRY: ");
    Serial.print(data[4]);
    Serial.print("\tRSW: ");
    Serial.println(data[5]);
  }
  sendData();
}

void sendData() {
  //transmit data to the base receiver
  digitalWrite(13, HIGH);
  driver.send(data, sizeof(data));
  driver.waitPacketSent();
  uint8_t len = sizeof(buf);
  //only wait 1 millisec before continuing
  if (driver.waitAvailableTimeout(1)) {
    if (driver.recv(buf, &len)) {
  //if (manager.sendtoWait(data, sizeof(data), SERVER_ADDRESS)) {
  //  uint8_t len = sizeof(buf);
  //  uint8_t from;
  //  if (manager.recvfromAckTimeout(buf, &len, 1000, &from)) {
      //Serial.print("Got reply from 0x");
      //Serial.print(from, HEX);
      lcd.setCursor(15, 0);
      if (buf[0] == 1) {
        lcd.write(byte(4));
      } else {
        lcd.write(byte(5));
      }
      lcd.setCursor(4, 1);
      lcd.print("    ");
      lcd.setCursor(4, 1);
      lcd.print(buf[1]);
      time = millis();
    } else {
      Serial.println("No reply. Is base receiver active?");
    }
  } else if (millis() - time > 2000) {
    Serial.println("Failed to receive data");
    lcd.setCursor(15, 0);
    lcd.write(byte(6));
    lcd.setCursor(4, 1);
    lcd.print("NA");
    delay(1000);  //no signal, so slow down loop
  }
  digitalWrite(13, LOW);
  //delay(10);
}