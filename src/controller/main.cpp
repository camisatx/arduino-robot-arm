#include <Arduino.h>
#include <SPI.h>        //radio
#include <Wire.h>       //display

#include <LiquidCrystal_I2C.h>  //display
#include <RH_NRF24.h>           //radio
#include <RHReliableDatagram.h> //radio

//Radio - transceiver
#define CLIENT_ADDRESS 1  //controller
#define SERVER_ADDRESS 2  //base
//delcare the radio driver to use
RH_NRF24 driver(8, 7); // CE, CSN
//class to manage message delivery and receipt, using the driver declared
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

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

//Declare the data array: LSW, LX, LY, RSW, RX, RY
uint8_t data[6];
//Define the message buffer
uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];

//LCD characters
byte CharUp[8] = {
  0b00000,
  0b00100,
  0b01110,
  0b11111,
  0b00100,
  0b00100,
  0b00100,
  0b00100
};
byte CharDown[8] = {
  0b00100,
  0b00100,
  0b00100,
  0b00100,
  0b11111,
  0b01110,
  0b00100,
  0b00000
};
byte CharLeft[8] = {
  0b00000,
  0b00100,
  0b01100,
  0b11111,
  0b01100,
  0b00100,
  0b00000,
  0b00000
};
byte CharRight[8] = {
  0b00000,
  0b00100,
  0b00110,
  0b11111,
  0b00110,
  0b00100,
  0b00000,
  0b00000
};

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

  //initialize manager object
  if (!manager.init())
    Serial.println("Radio manager init failed");
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

  Serial.println("Initialized");
  lcd.clear();
  lcd.print("L:");
  lcd.setCursor(8, 0);
  lcd.print("R:");
}

void loop() {
  //Read the analog input, map range from 0-1023 to 0-255
  int left_joy_x_reading = analogRead(LEFT_JOY_X);
  if (left_joy_x_reading < 470 || left_joy_x_reading > 550) {
    data[0] = map(left_joy_x_reading, 0, 1023, 0, 255);
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
  int left_joy_y_reading = analogRead(LEFT_JOY_Y);
  if (left_joy_y_reading < 470 || left_joy_y_reading > 550) {
    data[1] = map(left_joy_y_reading, 0, 1023, 0, 255);
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
  int right_joy_x_reading = analogRead(RIGHT_JOY_X);
  if (right_joy_x_reading < 470 || right_joy_x_reading > 550) {
    data[3] = map(right_joy_x_reading, 0, 1023, 0, 255);
    lcd.setCursor(11, 0);
    if (data[3] < 127) {
      lcd.write(byte(2));
    } else {
      lcd.write(byte(3));
    }
  } else {
    data[3] = 127;
    lcd.setCursor(11, 0);
    lcd.print(" ");
  }
  int right_joy_y_reading = analogRead(RIGHT_JOY_Y);
  if (right_joy_y_reading < 470 || right_joy_y_reading > 550) {
    data[4] = map(right_joy_y_reading, 0, 1023, 0, 255);
    lcd.setCursor(13, 0);
    if (data[4] < 127) {
      lcd.write(byte(1));
    } else {
      lcd.write(byte(0));
    }
  } else {
    data[4] = 127;
    lcd.setCursor(13, 0);
    lcd.print(" ");
  }
  data[2] = digitalRead(LEFT_JOY_SW);
  data[5] = digitalRead(RIGHT_JOY_SW);

  //Display the joystick data
  Serial.println("--------------------");
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
  Serial.println("Sending joystick data");

  digitalWrite(13, HIGH);
  if (manager.sendtoWait(data, sizeof(data), SERVER_ADDRESS)) {
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAckTimeout(buf, &len, 2000, &from)) {
      Serial.print("Got reply from 0x");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.println((char*)buf);
      lcd.setCursor(0, 1);
      lcd.print("LK: ");  //link to base status
      lcd.print(buf[0]);
      lcd.setCursor(8, 1);
      lcd.print("ST: ");  //base state
      lcd.print(buf[1]);
    } else {
      Serial.println("No reply. Is base nrf24 running?");
    }
  } else {
    Serial.println("Recv failed");
    lcd.setCursor(0, 1);
    lcd.print("LK: NA  ST: NA");
  }

  digitalWrite(13, LOW);
  delay(100);
}