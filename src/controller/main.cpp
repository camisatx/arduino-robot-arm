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

void setup() {
  Serial.begin(9600);

  pinMode(13, OUTPUT);
  
  // initialize LCD and set up the number of columns and rows:
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Initializing...");

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
  pinMode(RIGHT_JOY_SW, INPUT);

  Serial.println("Initialized");
  lcd.clear();
  lcd.print("Initialized...");
}

void loop() {
  digitalWrite(13, HIGH);

  //Read the analog input, map range from 0-1023 to 0-255
  data[0] = map(analogRead(LEFT_JOY_X), 0, 1023, 0, 255);
  data[1] = map(analogRead(LEFT_JOY_Y), 0, 1023, 0, 255);
  data[2] = digitalRead(LEFT_JOY_SW);
  data[3] = map(analogRead(RIGHT_JOY_X), 0, 1023, 0, 255);
  data[4] = map(analogRead(RIGHT_JOY_Y), 0, 1023, 0, 255);
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
  lcd.setCursor(0, 0);
  lcd.print("                ");  //clear row
  lcd.setCursor(0, 0);
  lcd.print(data[0]);
  lcd.print(";");
  lcd.print(data[1]);
  lcd.print(";");
  lcd.print(data[3]);
  lcd.print(";");
  lcd.print(data[4]);

  if (manager.sendtoWait(data, sizeof(data), SERVER_ADDRESS)) {
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAckTimeout(buf, &len, 2000, &from)) {
      Serial.print("Got reply from 0x");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.println((char*)buf);
      lcd.setCursor(0, 1);
      lcd.print("R:");
      lcd.print((char*)buf);
    } else {
      Serial.println("No reply. Is base nrf24 running?");
    }
  } else {
    Serial.println("Recv failed");
    lcd.setCursor(0, 1);
    lcd.print("Recv failed");
  }

  digitalWrite(13, LOW);
  delay(50);
}