#include "Wifi_Control.h"
#include "flightdata.h"

// Wi-Fi credentials and server setup
const char *ssid = "TVR";
const char *password = "12345678";
WiFiServer wifiServer(80);
String receivedMessage = "";

void initWifiAccessPoint(){
    Serial.println("Setting up Wi-Fi Access Point...");
    WiFi.softAP(ssid, password);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("Access Point IP: ");
    Serial.println(IP);
}

void startWifiServer(){
    wifiServer.begin();
    Serial.println("Wi-Fi server started.");
}

void remoteControl(void (*beginFlight)()){
  WiFiClient client = wifiServer.available();
  if (client) {
    Serial.println("Client connected");
    String request = "";  // To store the HTTP request
    String currentLine = "";  // Buffer for incoming data

    // Wait for a request from the client
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();  // Reading each character
        request += c;

        if (c == '\n') {
          if (currentLine.length() == 0) {
            // End of client request, process it
            if (request.indexOf("GET /download") != -1) {
              done = true;
              currentData.serve_csv(client);  // Serve the CSV file
            } else {
              // Send a 404 Not Found response if the request is not for /download
              client.println("HTTP/1.1 404 Not Found");
              client.println("Content-Type: text/plain");
              client.println("Connection: close");
              client.println();
              client.println("File not found");
            }
            break;
          } else {
              currentLine = "";
          }
        } else if (c != '\r') {
            currentLine += c;
        }

        if (c == 'A')
            beginFlight();
      }
    }

    client.stop();
    Serial.println("Client disconnected");
  }
}