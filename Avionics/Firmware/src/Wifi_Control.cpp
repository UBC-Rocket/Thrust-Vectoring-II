// Wifi_Control.cpp
#include "Wifi_Control.h"

const char *ssid = "TVR_SECURE";
const char *password = "UBCRocket_TVR_2024!";
WiFiServer wifiServer(80);
String receivedMessage = "";
bool done = false;

// Security variables
uint8_t encryptionKey[KEY_SIZE];
uint8_t currentNonce[NONCE_SIZE];
mbedtls_aes_context aes;

// Error handling helper function
WifiStatus handleError(WifiStatus error) {
    String errorMsg = getStatusMessage(error);
    Serial.println("Error: " + errorMsg);
    return error;
}

String getStatusMessage(WifiStatus status) {
    switch(status) {
        case WifiStatus::SUCCESS: return "Operation successful";
        case WifiStatus::INIT_FAILED: return "WiFi initialization failed";
        case WifiStatus::AUTH_FAILED: return "Authentication failed";
        case WifiStatus::CONNECTION_LOST: return "Connection lost";
        case WifiStatus::ENCRYPTION_ERROR: return "Encryption/Decryption error";
        case WifiStatus::TIMEOUT: return "Operation timed out";
        case WifiStatus::INVALID_COMMAND: return "Invalid command received";
        default: return "Unknown error";
    }
}

bool initWifiAccessPoint() {
    Serial.println("Setting up Wi-Fi Access Point...");
    
    // Generate random encryption key
    if(!generateSecurityParameters()) {
        return handleError(WifiStatus::INIT_FAILED);
    }
    
    // Initialize encryption
    mbedtls_aes_init(&aes);
    if(mbedtls_aes_setkey_enc(&aes, encryptionKey, 256) != 0) {
        return handleError(WifiStatus::INIT_FAILED);
    }
    
    // Start AP with WPA2
    if(!WiFi.softAP(ssid, password, 1, 1)) {  // Channel 1, Hidden SSID
        return handleError(WifiStatus::INIT_FAILED);
    }
    
    IPAddress IP = WiFi.softAPIP();
    Serial.print("Access Point IP: ");
    Serial.println(IP);
    return true;
}

bool generateSecurityParameters() {
    if(!ESP.getRandomBytes(encryptionKey, KEY_SIZE)) {
        return false;
    }
    return true;
}

bool authenticateClient(WiFiClient& client) {
    // Generate new challenge nonce
    if(!ESP.getRandomBytes(currentNonce, NONCE_SIZE)) {
        return handleError(WifiStatus::AUTH_FAILED);
    }
    
    // Send challenge
    if(client.write(currentNonce, NONCE_SIZE) != NONCE_SIZE) {
        return handleError(WifiStatus::AUTH_FAILED);
    }
    
    // Wait for response with timeout
    unsigned long startTime = millis();
    while(!client.available() && millis() - startTime < AUTH_TIMEOUT) {
        delay(10);
    }
    
    if(!client.available()) {
        return handleError(WifiStatus::TIMEOUT);
    }
    
    // Verify response
    uint8_t receivedHMAC[32];
    if(client.readBytes(receivedHMAC, 32) != 32) {
        return handleError(WifiStatus::AUTH_FAILED);
    }
    
    uint8_t expectedHMAC[32];
    generateHMAC(currentNonce, NONCE_SIZE, expectedHMAC);
    
    return memcmp(receivedHMAC, expectedHMAC, 32) == 0;
}

void sendAcknowledgment(WiFiClient& client, CommandAck ack) {
    // Create JSON acknowledgment
    String jsonAck = "{\"success\":" + String(ack.success ? "true" : "false") + ",";
    jsonAck += "\"timestamp\":" + String(ack.timestamp) + ",";
    jsonAck += "\"status\":\"" + getStatusMessage(ack.status) + "\",";
    jsonAck += "\"message\":\"" + ack.message + "\"}";
    
    // Encrypt acknowledgment
    uint8_t encryptedAck[512];
    size_t encryptedLen;
    
    if(encryptData((uint8_t*)jsonAck.c_str(), jsonAck.length(), encryptedAck, &encryptedLen)) {
        client.write(encryptedAck, encryptedLen);
    }
}

bool encryptData(const uint8_t* input, size_t input_len, uint8_t* output, size_t* output_len) {
    // Add PKCS7 padding
    size_t padded_len = (input_len + 15) & ~15;
    uint8_t padding = padded_len - input_len;
    
    // Copy input and add padding
    memcpy(output, input, input_len);
    memset(output + input_len, padding, padding);
    
    *output_len = padded_len;
    
    // Encrypt data
    for(size_t i = 0; i < padded_len; i += 16) {
        if(mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, output + i, output + i) != 0) {
            return false;
        }
    }
    
    return true;
}

bool decryptData(const uint8_t* input, size_t input_len, uint8_t* output, size_t* output_len) {
    if(input_len % 16 != 0) {
        return false;
    }
    
    // Decrypt data
    for(size_t i = 0; i < input_len; i += 16) {
        if(mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_DECRYPT, input + i, output + i) != 0) {
            return false;
        }
    }
    
    // Remove padding
    uint8_t padding = output[input_len - 1];
    if(padding > 16) {
        return false;
    }
    
    *output_len = input_len - padding;
    return true;
}

void remoteControl(void (*beginFlight)()) {
    WiFiClient client = wifiServer.available();
    
    if (client) {
        Serial.println("Client connected");
        
        // Authenticate client
        if (!authenticateClient(client)) {
            client.stop();
            return;
        }
        
        Serial.println("Client authenticated");
        CommandAck ack;
        
        while (client.connected()) {
            if (client.available()) {
                // Read encrypted data
                uint8_t encryptedData[1024];
                size_t dataLen = client.readBytes(encryptedData, 1024);
                
                // Decrypt data
                uint8_t decryptedData[1024];
                size_t decryptedLen;
                
                if (!decryptData(encryptedData, dataLen, decryptedData, &decryptedLen)) {
                    ack = {false, millis(), WifiStatus::ENCRYPTION_ERROR, "Decryption failed"};
                    sendAcknowledgment(client, ack);
                    continue;
                }
                
                // Process command
                if (decryptedLen > 0 && decryptedData[0] == 'A') {
                    beginFlight();
                    ack = {true, millis(), WifiStatus::SUCCESS, "Flight initiated"};
                } else {
                    ack = {false, millis(), WifiStatus::INVALID_COMMAND, "Invalid command"};
                }
                
                sendAcknowledgment(client, ack);
            }
            
            // Check connection health
            if (!client.connected()) {
                handleError(WifiStatus::CONNECTION_LOST);
                break;
            }
        }
        
        client.stop();
        Serial.println("Client disconnected");
    }
}