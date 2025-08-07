# Firmware/src/test_firmware.py
import unittest
import socket
import time
from rocket_client import RocketClient

# --- Test Configuration ---
ESP32_IP = "192.168.4.1"
ESP32_PORT = 80
CORRECT_PASSWORD = "UBCRocket_TVR_2024!"
INCORRECT_PASSWORD = "ThisIsTheWrongPassword123"

class TestRocketFirmware(unittest.TestCase):
    
    def setUp(self):
        """Called before each test. Ensures no client is lingering."""
        self.client = None
        print(f"\n--- Running {self.id()} ---")
        # IMPORTANT: Manually reboot the ESP32 before starting the test suite.
        # This ensures the firmware starts in the correct IDLE state.
    
    def tearDown(self):
        """Called after each test. Ensures the connection is closed."""
        if self.client:
            self.client.disconnect()

    def test_TC01_successful_connection_and_authentication(self):
        """Validates TC-01."""
        self.client = RocketClient(ESP32_IP, ESP32_PORT, CORRECT_PASSWORD)
        self.client.connect()
        self.assertTrue(self.client.authenticate(), "Authentication should succeed with the correct password.")
            
    def test_TC02_authentication_failure_with_wrong_password(self):
        """Validates TC-02."""
        self.client = RocketClient(ESP32_IP, ESP32_PORT, INCORRECT_PASSWORD)
        self.client.connect()
        # The firmware will close the connection upon receiving an invalid HMAC. 
        # The client's subsequent recv will fail. We expect a TimeoutError or ConnectionError.
        with self.assertRaises((TimeoutError, ConnectionError), msg="Authentication should fail with an incorrect password."):
            self.client.authenticate()
        
    def test_TC03_authentication_failure_timeout(self):
        """Validates TC-03. Verifies the AUTH_TIMEOUT on the firmware."""
        # This test connects but does not send anything, forcing a server-side timeout.
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(8.0) # Timeout greater than the firmware's 5s AUTH_TIMEOUT
            sock.connect((ESP32_IP, ESP32_PORT))
            nonce = sock.recv(16) # Receive the nonce to prove connection was made
            self.assertEqual(len(nonce), 16)
            # Now we just wait. The server should time out and close the connection.
            # A subsequent recv should return 0 bytes.
            time.sleep(6) # Wait for firmware's 5s timeout to elapse
            data = sock.recv(1024)
            self.assertEqual(len(data), 0, "Server should have closed connection after AUTH_TIMEOUT.")

    def test_TC04_successful_flight_initiate_command(self):
        """Validates TC-04. Requires a rebooted ESP32 in IDLE state."""
        self.client = RocketClient(ESP32_IP, ESP32_PORT, CORRECT_PASSWORD)
        self.client.connect()
        self.client.authenticate()
        
        ack = self.client.send_command('A')
        
        self.assertTrue(ack.get('success'), "Ack 'success' field should be true.")
        self.assertEqual(ack.get('status'), "Operation successful", "Status message should be correct.")
        self.assertEqual(ack.get('message'), "Flight initiated", "Message should confirm initiation.")

    def test_TC05_invalid_command_rejection(self):
        """Validates TC-05. Requires a rebooted ESP32 in IDLE state."""
        self.client = RocketClient(ESP32_IP, ESP32_PORT, CORRECT_PASSWORD)
        self.client.connect()
        self.client.authenticate()
        
        ack = self.client.send_command('X') # 'X' is not a valid command.
        
        self.assertFalse(ack.get('success'), "Ack 'success' field should be false.")
        self.assertEqual(ack.get('status'), "Invalid command received", "Status should be 'Invalid command'.")

    def test_TC06_successful_csv_download(self):
        """Validates TC-06. Self-contained: initiates flight, then downloads."""
        # Step 1: Initiate flight to ensure data.csv has content.
        self.client = RocketClient(ESP32_IP, ESP32_PORT, CORRECT_PASSWORD)
        self.client.connect()
        self.client.authenticate()
        self.client.send_command('A')
        self.client.disconnect() # Disconnect the command client
        
        time.sleep(2) # Allow firmware to write a few rows of data.

        # Step 2: Download the file using a new client instance.
        dl_client = RocketClient(ESP32_IP, ESP32_PORT, CORRECT_PASSWORD)
        status_code, headers, body = dl_client.download_csv()

        self.assertEqual(status_code, 200)
        self.assertEqual(headers.get('content-type'), 'text/csv')
        expected_header = b"Time (ms),Input x,Output x,Input y,Output y,Accel x"
        self.assertTrue(body.startswith(expected_header), "CSV body should start with the correct header.")
        self.assertGreater(len(body.split(b'\n')), 2, "CSV should contain data rows.")

    def test_TC07_http_404_not_found(self):
        """Validates TC-07."""
        # Use the download method but with a custom, invalid path. This requires a small temporary modification or separate function.
        # For simplicity, we make a raw request.
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as http_sock:
            http_sock.settimeout(5.0)
            http_sock.connect((ESP32_IP, ESP32_PORT))
            request = f"GET /some/other/path HTTP/1.1\r\nHost: {ESP32_IP}\r\n\r\n"
            http_sock.sendall(request.encode('utf-8'))
            response_data = http_sock.recv(1024)
            self.assertTrue(response_data.startswith(b'HTTP/1.1 404 Not Found'))

if __name__ == '__main__':
    print("======================================================================")
    print("==           UBC ROCKET THRUST VECTORING FIRMWARE TEST SUITE          ==")
    print("======================================================================")
    print("IMPORTANT: Ensure the ESP32 has been manually power-cycled/rebooted")
    print("           before running this test suite to ensure a clean state.")
    print("----------------------------------------------------------------------")
    unittest.main()