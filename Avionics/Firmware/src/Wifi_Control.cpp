// Firmware/src/Wifi_Control.cpp
#include "Wifi_Control.h"
#include "flightdata.h"

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
bool handleError(WifiStatus error) {
    String errorMsg = getStatusMessage(error);
    Serial.println("Error: " + errorMsg);
    return false;
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
        Serial.println("Failed to generate security parameters");
        handleError(WifiStatus::INIT_FAILED);
        return false;
    }
    Serial.println("Security parameters generated successfully");
    
    // Initialize encryption
    mbedtls_aes_init(&aes);
    if(mbedtls_aes_setkey_enc(&aes, encryptionKey, 256) != 0) {
        Serial.println("Failed to initialize encryption");
        handleError(WifiStatus::INIT_FAILED);
        return false;
    }
    Serial.println("Encryption initialized successfully");
    
    // Start AP with WPA2
    Serial.print("Creating AP with SSID: ");
    Serial.println(ssid);
    
    // Change the last parameter to 0 to make it visible (not hidden)
    if(!WiFi.softAP(ssid, password, 1, 0)) {
        Serial.println("Failed to create access point");
        handleError(WifiStatus::INIT_FAILED);
        return false;
    }
    
    IPAddress IP = WiFi.softAPIP();
    Serial.print("Access Point IP: ");
    Serial.println(IP);
    return true;
}


bool generateSecurityParameters() {
    // Use password to generate key instead of random bytes
    // Password is already defined as "UBCRocket_TVR_2024!"
    Serial.print("Generating encryption key from password...");

    // Initialize the hash context
    mbedtls_md_context_t ctx;
    mbedtls_md_init(&ctx);
    mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 0);

    mbedtls_md_starts(&ctx);
    // Update with password
    mbedtls_md_update(&ctx, (const unsigned char*)password, strlen(password));
    // Calculate the hash
    mbedtls_md_finish(&ctx, encryptionKey);
    mbedtls_md_free(&ctx);
    
    return true;
}


bool authenticateClient(WiFiClient& client) {
    // Generate new challenge nonce
    Serial.println("Generating nonce...");
    for (int i = 0; i < NONCE_SIZE; i++) {
        currentNonce[i] = (uint8_t)esp_random();
    }
    
    // Send challenge
    Serial.println("Sending nonce challenge to client...");
    int bytesSent = client.write(currentNonce, NONCE_SIZE);
    if(bytesSent != NONCE_SIZE) {
        Serial.println("Failed to send complete nonce");
        handleError(WifiStatus::AUTH_FAILED);
        return false;
    }
    Serial.println("Nonce sent successfully");

    Serial.println("Starting wait for HMAC response...");
    Serial.print("Current millis: ");
    Serial.println(millis());
    
    // Wait for response with timeout
    Serial.println("Waiting for HMAC response...");
    unsigned long startTime = millis();
    unsigned long currentTime;
    bool timedOut = false;

    while (!client.available()) {
        currentTime = millis();
        if (currentTime - startTime >= AUTH_TIMEOUT) {
            timedOut = true;
            break;
        }
        // Short yield to prevent watchdog triggering
        delay(5);
    }

    if (timedOut) {
        Serial.println("Timeout waiting for HMAC response");
        handleError(WifiStatus::TIMEOUT);
        return false;
    }

    Serial.print("Received HMAC response");
    
    // Verify response
    uint8_t receivedHMAC[32];
    Serial.println("Reading HMAC response...");
    if(client.readBytes(receivedHMAC, 32) != 32) {
        Serial.println("Failed to read complete HMAC");
        handleError(WifiStatus::AUTH_FAILED);
        return false;
    }
    
    uint8_t expectedHMAC[32];
    generateHMAC(currentNonce, NONCE_SIZE, expectedHMAC);
    
    bool result = (memcmp(receivedHMAC, expectedHMAC, 32) == 0);
    Serial.print("HMAC comparison result: ");
    Serial.println(result ? "MATCH" : "MISMATCH");
    Serial.println(result ? "HMAC authentication SUCCESS" : "HMAC authentication FAILED");
    if (!result) {
        for (int i = 0; i < 32; i++) {
            if (receivedHMAC[i] != expectedHMAC[i]) {
                Serial.print("First difference at byte ");
                Serial.print(i);
                Serial.print(": received 0x");
                Serial.print(receivedHMAC[i], HEX);
                Serial.print(", expected 0x");
                Serial.println(expectedHMAC[i], HEX);
                break;
            }
        }
    }
    
    return result;
}


void generateHMAC(const uint8_t* data, size_t data_len, uint8_t* hmac) {
    Serial.print("Generating HMAC for data of length: ");
    Serial.println(data_len);

    mbedtls_md_context_t ctx;
    mbedtls_md_init(&ctx);
    
    if(mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1) != 0) {
        Serial.println("HMAC setup failed");
        return;
    }
    
    if(mbedtls_md_hmac_starts(&ctx, encryptionKey, KEY_SIZE) != 0) {
        Serial.println("HMAC starts failed");
        mbedtls_md_free(&ctx);
        return;
    }
    
    if(mbedtls_md_hmac_update(&ctx, data, data_len) != 0) {
        Serial.println("HMAC update failed");
        mbedtls_md_free(&ctx);
        return;
    }
    
    if(mbedtls_md_hmac_finish(&ctx, hmac) != 0) {
        Serial.println("HMAC finish failed");
    }
    
    mbedtls_md_free(&ctx);
}


void sendAcknowledgment(WiFiClient& client, CommandAck ack) {
    // Create JSON acknowledgment
    String jsonAck = "{\"success\":" + String(ack.success ? "true" : "false") + ",";
    jsonAck += "\"timestamp\":" + String(ack.timestamp) + ",";
    jsonAck += "\"status\":\"" + getStatusMessage(ack.status) + "\",";
    jsonAck += "\"message\":\"" + ack.message + "\"}";
    
    Serial.print("JSON ACK: ");
    Serial.println(jsonAck);
    
    // Encrypt acknowledgment
    uint8_t encryptedAck[512];
    size_t encryptedLen;
    
    if(encryptData((uint8_t*)jsonAck.c_str(), jsonAck.length(), encryptedAck, &encryptedLen)) {
        Serial.print("Encrypted ACK length: ");
        Serial.println(encryptedLen);
        size_t bytesSent = client.write(encryptedAck, encryptedLen);
        Serial.print("Bytes sent: ");
        Serial.println(bytesSent);
    } else {
        Serial.println("Failed to encrypt acknowledgment");
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
            Serial.println("Encryption failed");
            return false;
        }
    }
    
    Serial.println("Encryption successful");
    return true;
}


bool decryptData(const uint8_t* input, size_t input_len, uint8_t* output, size_t* output_len) {  
    if(input_len % 16 != 0) {
        Serial.println("Invalid input length - not a multiple of 16");
        return false;
    }
    
    // Decrypt data
    for(size_t i = 0; i < input_len; i += 16) {
        if(mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_DECRYPT, input + i, output + i) != 0) {
            Serial.println("Decryption failed");
            return false;
        }
    }
    
    // Remove padding
    uint8_t padding = output[input_len - 1];
    
    if(padding > 16) {
        Serial.println("Invalid padding value");
        return false;
    }
    
    *output_len = input_len - padding;
    Serial.print("Decrypted length: ");
    Serial.println(*output_len);
    
    Serial.println("Decryption successful");
    return true;
}


bool verifyHMAC(const uint8_t* data, size_t data_len, const uint8_t* hmac) {
    uint8_t calculatedHMAC[32];
    generateHMAC(data, data_len, calculatedHMAC);
    return memcmp(calculatedHMAC, hmac, 32) == 0;
}


void remoteControl(void (*beginFlight)()) {
    WiFiClient client = wifiServer.available();
    
    if (client) {
        Serial.println("\n\n!!! CLIENT CONNECTED !!!");
        
        // Wait a bit for the connection to stabilize
        delay(100);
        
        // Check if it's an HTTP request by peeking at the first byte
        if(client.available() && (client.peek() == 'G' || client.peek() == 'P' || client.peek() == 'H')) {
            Serial.println("HTTP request detected");
            
            String currentLine = "";  // Buffer for incoming HTTP request line
            String request = "";      // Complete HTTP request
            
            while (client.connected()) {
                if (client.available()) {
                    char c = client.read();
                    request += c;
                    
                    // Process HTTP request character by character
                    if (c == '\n') {
                        if (currentLine.length() == 0) {
                            // End of HTTP headers, process the request
                            Serial.println("Processing HTTP request");
                            
                            if (request.indexOf("GET /download") >= 0) {
                                Serial.println("CSV download requested");
                                done = true;
                                
                                // Call serve_csv which handles the HTTP response internally
                                currentData.serve_csv(client);
                                
                                Serial.println("CSV data served successfully");
                                break;
                            } else {
                                // Send 404 response for any other HTTP request
                                client.println("HTTP/1.1 404 Not Found");
                                client.println("Content-Type: text/plain");
                                client.println("Connection: close");
                                client.println();
                                client.println("File not found");
                                Serial.println("404 response sent");
                                break;
                            }
                        } else {
                            // Reset current line for the next line in HTTP request
                            currentLine = "";
                        }
                    } else if (c != '\r') {
                        currentLine += c;
                    }
                }
            }
        } else {
            // Not an HTTP request - handle as encrypted command
            Serial.println("Starting authentication for encrypted command...");
            
            // Authenticate client first
            if (!authenticateClient(client)) {
                Serial.println("Authentication failed, disconnecting client");
                client.stop();
                return;
            }
            
            Serial.println("Client authenticated successfully");
            CommandAck ack;
            
            // Wait for encrypted command
            Serial.println("Waiting for encrypted command...");
            unsigned long cmdStartTime = millis();
            while(!client.available() && millis() - cmdStartTime < 5000) { // 5-second timeout
                delay(10);
            }
            
            if(!client.available()) {
                Serial.println("Timeout waiting for command");
                client.stop();
                return;
            }
            
            // Read encrypted data
            uint8_t encryptedData[1024];
            size_t dataLen = client.readBytes(encryptedData, 1024);
            Serial.print("Received ");
            Serial.print(dataLen);
            Serial.println(" bytes of encrypted data");
            
            // Decrypt data using your security implementation
            uint8_t decryptedData[1024];
            size_t decryptedLen;
            
            if (!decryptData(encryptedData, dataLen, decryptedData, &decryptedLen)) {
                Serial.println("Decryption failed");
                ack = {false, millis(), WifiStatus::ENCRYPTION_ERROR, "Decryption failed"};
                sendAcknowledgment(client, ack);
                client.stop();
                return;
            }
            
            Serial.print("Decrypted data: ");
            for (size_t i = 0; i < decryptedLen; i++) {
                Serial.print((char)decryptedData[i]);
            }
            Serial.println();
            
            // Process command ('A' for flight initiation)
            if (decryptedLen > 0 && decryptedData[0] == 'A') {
                Serial.println("Valid 'A' command received, initiating flight");
                beginFlight();
                ack = {true, millis(), WifiStatus::SUCCESS, "Flight initiated"};
            } else {
                Serial.print("Invalid command: ");
                for (size_t i = 0; i < decryptedLen; i++) {
                    Serial.print((char)decryptedData[i]);
                }
                Serial.println();
                ack = {false, millis(), WifiStatus::INVALID_COMMAND, "Invalid command"};
            }
            
            // Send encrypted acknowledgment
            Serial.println("Sending acknowledgment...");
            sendAcknowledgment(client, ack);
            Serial.println("Acknowledgment sent");
            
            // Clear any remaining data in the buffer
            while (client.available()) {
                client.read();
            }
        }
        
        client.stop();
        Serial.println("Client disconnected normally");
    } else {
        // This will help us see if the function is being called periodically
        static unsigned long lastCheck = 0;
        unsigned long now = millis();
        if (now - lastCheck > 5000) {  // Print every 5 seconds
            Serial.println("Waiting for client connection...");
            lastCheck = now;
        }
    }
}


bool startWifiServer() {
    wifiServer.begin();
    Serial.println("WiFi server started");
    return true;
}