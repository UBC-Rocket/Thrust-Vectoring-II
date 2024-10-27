//This example code is in the Public Domain (or CC0 licensed, at your option.)
//By Evandro Copercini - 2018
//
//This example creates a bridge between Serial and Classical Bluetooth (SPP)
//and also demonstrate that SerialBT have the same functionalities of a normal Serial

#include "BluetoothSerial.h"

//#define CONFIG_BT_ENABLED 1
//#define CONFIG_BLUEDROID_ENABLED 1

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error (Bluetooth is not enabled! Please run `make menuconfig` to and enable it)
#endif

int Toggle = 0;
char incString;
String message = "";
#define OUTPUTPIN 27

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  pinMode(OUTPUTPIN,OUTPUT);
}

void loop() {
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    
    incString = SerialBT.read();
    if(incString != '\n') {
      //Serial.write(incString);
      message += incString;
    }
    else{
      message = "";
    }
    Serial.write(incString);
  }
  Serial.println(message);
    if (message == "LED_Toggle"){
    Toggle = !Toggle;
    Serial.print(Toggle);
    digitalWrite(27,Toggle);
  }

  delay(20);
}