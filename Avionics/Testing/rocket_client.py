# Firmware/src/rocket_client.py
import socket
import json
import hmac
import hashlib
import time
from Crypto.Cipher import AES
from Crypto.Util.Padding import pad, unpad

class RocketClient:
    """A non-GUI client for interacting with the rocket firmware, designed for test automation."""
    def __init__(self, ip, port, password):
        self.ip = ip
        self.port = port
        self.password = password
        self.sock = None
        # Key is derived once and stored. This matches the firmware's logic.
        self.encryption_key = hashlib.sha256(self.password.encode()).digest()

    def _encrypt(self, data_str):
        """Encrypts a string using AES/ECB with PKCS7 padding."""
        cipher = AES.new(self.encryption_key, AES.MODE_ECB)
        padded_data = pad(data_str.encode('utf-8'), AES.block_size)
        return cipher.encrypt(padded_data)

    def _decrypt(self, encrypted_data):
        """Decrypts data using AES/ECB and removes PKCS7 padding."""
        cipher = AES.new(self.encryption_key, AES.MODE_ECB)
        decrypted_padded = cipher.decrypt(encrypted_data)
        return unpad(decrypted_padded, AES.block_size).decode('utf-8')

    def _generate_hmac(self, data):
        """Generates an HMAC-SHA256 signature."""
        return hmac.new(self.encryption_key, data, hashlib.sha256).digest()

    def connect(self):
        """Establishes a TCP connection to the rocket."""
        if self.sock:
            self.disconnect()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(10.0) # Set a timeout for all socket operations
        try:
            self.sock.connect((self.ip, self.port))
            # Per firmware analysis, a brief delay allows the server to be ready.
            time.sleep(0.2)
        except (socket.timeout, ConnectionRefusedError) as e:
            self.sock = None
            raise ConnectionError(f"Failed to connect to {self.ip}:{self.port}") from e

    def disconnect(self):
        """Closes the TCP connection."""
        if self.sock:
            self.sock.close()
            self.sock = None

    def authenticate(self, timeout=5.0):
        """Performs the HMAC challenge-response authentication."""
        if not self.sock:
            raise ConnectionError("Not connected. Cannot authenticate.")
        try:
            # 1. Receive challenge nonce from server
            nonce = self.sock.recv(16)
            if len(nonce) != 16:
                raise ConnectionError("Failed to receive complete nonce from server.")

            # 2. Generate and send HMAC response
            hmac_response = self._generate_hmac(nonce)
            self.sock.sendall(hmac_response)
            return True
        except socket.timeout as e:
            raise TimeoutError("Authentication timed out while waiting for nonce or sending response.") from e

    def send_command(self, command_char):
        """Encrypts and sends a command, then waits for and decrypts the acknowledgment."""
        if not self.sock:
            raise ConnectionError("Not connected. Cannot send command.")
        try:
            # 1. Encrypt and send command
            encrypted_command = self._encrypt(command_char)
            self.sock.sendall(encrypted_command)

            # 2. Wait for and process acknowledgment
            encrypted_ack = self.sock.recv(1024)
            if not encrypted_ack:
                 raise ConnectionError("Did not receive acknowledgment from server.")
            
            decrypted_ack_str = self._decrypt(encrypted_ack)
            return json.loads(decrypted_ack_str)
        except socket.timeout as e:
            raise TimeoutError("Timed out waiting for command acknowledgment.") from e

    def download_csv(self):
        """Performs an HTTP GET request to download the data.csv file."""
        # HTTP is a separate transaction, so it uses its own socket.
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as http_sock:
            http_sock.settimeout(10.0)
            http_sock.connect((self.ip, self.port))
            
            request = f"GET /download HTTP/1.1\r\nHost: {self.ip}\r\nConnection: close\r\n\r\n"
            http_sock.sendall(request.encode('utf-8'))

            response_data = b''
            while True:
                chunk = http_sock.recv(4096)
                if not chunk:
                    break
                response_data += chunk
            
            parts = response_data.split(b'\r\n\r\n', 1)
            headers_raw = parts[0].decode('utf-8')
            body = parts[1] if len(parts) > 1 else b''

            header_lines = headers_raw.split('\r\n')
            status_line = header_lines[0]
            status_code = int(status_line.split(' ')[1])
            
            headers = {line.split(': ', 1)[0].lower(): line.split(': ', 1)[1] for line in header_lines[1:]}

            return status_code, headers, body