// include/Wifi_Control.h
#ifndef WIFI_CONTROL_H
#define WIFI_CONTROL_H
#include <WiFi.h>
#include <mbedtls/aes.h>
#include <mbedtls/md.h>
#include <esp_random.h>

// Configuration constants
#define KEY_SIZE 32
#define NONCE_SIZE 16
#define AUTH_TIMEOUT 5000
#define MAX_RETRIES 3
#define COMMAND_TIMEOUT 2000

// Status codes for error handling
enum class WifiStatus {
    SUCCESS,
    INIT_FAILED,
    AUTH_FAILED,
    CONNECTION_LOST,
    ENCRYPTION_ERROR,
    TIMEOUT,
    INVALID_COMMAND
};

// Command acknowledgment structure
struct CommandAck {
    bool success;
    unsigned long timestamp;
    WifiStatus status;
    String message;
};

extern const char *ssid;
extern const char *password;
extern WiFiServer wifiServer;
extern String receivedMessage;
extern bool done;

// Security variables
extern uint8_t encryptionKey[KEY_SIZE];
extern uint8_t currentNonce[NONCE_SIZE];

// Core functions
bool initWifiAccessPoint(); // Now returns bool for error checking
bool startWifiServer(); // Now returns bool for error checking
void remoteControl(void (*beginFlight)());

// Security functions
bool generateSecurityParameters();
bool authenticateClient(WiFiClient& client);
bool encryptData(const uint8_t* input, size_t input_len, uint8_t* output, size_t* output_len);
bool decryptData(const uint8_t* input, size_t input_len, uint8_t* output, size_t* output_len);
void generateHMAC(const uint8_t* data, size_t data_len, uint8_t* hmac);
bool verifyHMAC(const uint8_t* data, size_t data_len, const uint8_t* hmac);

// Error handling and acknowledgment functions
void sendAcknowledgment(WiFiClient& client, CommandAck ack);
bool handleError(WifiStatus error);
String getStatusMessage(WifiStatus status);
#endif // WIFI_CONTROL_H