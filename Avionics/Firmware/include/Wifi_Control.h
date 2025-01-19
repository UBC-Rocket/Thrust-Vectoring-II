#ifndef WIFI_CONTROL_H
#define WIFI_CONTROL_H

#include<WiFi.h>

extern const char *ssid;
extern const char *password;
extern WiFiServer wifiServer;
extern String receivedMessage;
extern bool done;

//Initialize Wifi as an Access Point
void initWifiAccessPoint();

// Start the Wi-Fi server for remote ignition control and .csv upload
void startWifiServer();

// ======== Remote Ignition Control & Remote CSV Download (Wi-Fi Command Listening) ======== //
void remoteControl(void (*beginFlight)());

#endif // WIFI_CONTROL.H