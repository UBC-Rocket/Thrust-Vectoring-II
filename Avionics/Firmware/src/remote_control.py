import tkinter as tk
from tkinter import messagebox
import socket
import threading
import time
import json
import hmac
import hashlib
from Crypto.Cipher import AES
from Crypto.Util.Padding import pad, unpad
import os

class ESP32GUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("ESP32 WiFi Control")
        self.root.geometry("400x350")  # Increased size for more status info
        
        # Network settings
        self.esp32_ip = "192.168.4.1"
        self.esp32_port = 80
        self.connected = False
        self.max_retries = 3
        self.auth_timeout = 5  # seconds
        
        # Security settings
        self.password = "UBCRocket_TVR_2024!"  # Must match ESP32
        self.encryption_key = None  # Will be derived from password
        
        # GUI Elements
        self.connection_label = tk.Label(self.root, text="WiFi Status: Disconnected", fg="red")
        self.connection_label.pack(pady=10)
        
        self.auth_label = tk.Label(self.root, text="Authentication: Not Authenticated", fg="red")
        self.auth_label.pack(pady=10)
        
        self.send_button = tk.Button(self.root, text="Send 'A' Command", 
                                   command=self.send_command, state='disabled')
        self.send_button.pack(pady=20)
        
        self.status_label = tk.Label(self.root, text="Status: Not Connected")
        self.status_label.pack(pady=20)
        
        self.response_label = tk.Label(self.root, text="Last Response: None", 
                                     wraplength=350)
        self.response_label.pack(pady=20)
        
        # Start connection monitoring
        self.monitor_thread = threading.Thread(target=self.monitor_connection, 
                                            daemon=True)
        self.monitor_thread.start()

    def encrypt_data(self, data):
        try:
            cipher = AES.new(self.encryption_key, AES.MODE_ECB)
            padded_data = pad(data.encode(), AES.block_size)
            return cipher.encrypt(padded_data)
        except Exception as e:
            print(f"Encryption error: {e}")
            return None

    def decrypt_data(self, encrypted_data):
        try:
            cipher = AES.new(self.encryption_key, AES.MODE_ECB)
            decrypted_data = cipher.decrypt(encrypted_data)
            return unpad(decrypted_data, AES.block_size)
        except Exception as e:
            print(f"Decryption error: {e}")
            return None

    def generate_hmac(self, data):
        return hmac.new(self.encryption_key, data, hashlib.sha256).digest()

    def authenticate_with_server(self, sock):
        try:
            # Receive challenge nonce
            nonce = sock.recv(16)
            if not nonce:
                return False
            
            # Generate HMAC response
            hmac_response = self.generate_hmac(nonce)
            
            # Send response
            sock.send(hmac_response)
            
            self.root.after(0, lambda: self.auth_label.config(
                text="Authentication: Authenticated", fg="green"))
            return True
            
        except Exception as e:
            print(f"Authentication error: {e}")
            self.root.after(0, lambda: self.auth_label.config(
                text="Authentication: Failed", fg="red"))
            return False

    def check_wifi_connection(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2)
            result = sock.connect_ex((self.esp32_ip, self.esp32_port))
            sock.close()
            return result == 0
        except:
            return False

    def monitor_connection(self):
        while True:
            connected = self.check_wifi_connection()
            if connected != self.connected:
                self.connected = connected
                self.root.after(0, self.update_connection_status)
            time.sleep(2)

    def update_connection_status(self):
        if self.connected:
            self.connection_label.config(text="WiFi Status: Connected", fg="green")
            self.send_button.config(state='normal')
            self.status_label.config(text="Status: Ready")
        else:
            self.connection_label.config(text="WiFi Status: Disconnected", fg="red")
            self.send_button.config(state='disabled')
            self.status_label.config(text="Status: Not Connected")
            self.auth_label.config(text="Authentication: Not Authenticated", fg="red")

    def process_acknowledgment(self, encrypted_ack):
        try:
            decrypted_ack = self.decrypt_data(encrypted_ack)
            if not decrypted_ack:
                return False
            
            ack_data = json.loads(decrypted_ack.decode())
            
            # Update status based on acknowledgment
            status_text = f"Status: {ack_data['status']}"
            response_text = f"Last Response: {ack_data['message']}"
            
            self.status_label.config(text=status_text, 
                fg="green" if ack_data['success'] else "red")
            self.response_label.config(text=response_text)
            
            return ack_data['success']
            
        except Exception as e:
            print(f"Error processing acknowledgment: {e}")
            return False

    def send_command(self):
        sock = None
        retry_count = 0
        
        while retry_count < self.max_retries:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5)
                sock.connect((self.esp32_ip, self.esp32_port))
                
                # Authenticate first
                if not self.authenticate_with_server(sock):
                    raise Exception("Authentication failed")
                
                # Encrypt and send command
                encrypted_command = self.encrypt_data('A')
                if not encrypted_command:
                    raise Exception("Encryption failed")
                
                sock.send(encrypted_command)
                
                # Wait for acknowledgment
                encrypted_ack = sock.recv(1024)
                if not encrypted_ack:
                    raise Exception("No acknowledgment received")
                
                if self.process_acknowledgment(encrypted_ack):
                    break
                
            except socket.timeout:
                retry_count += 1
                self.status_label.config(text=f"Status: Retry {retry_count}/{self.max_retries}")
                
            except Exception as e:
                self.status_label.config(text=f"Status: Error - {str(e)}")
                messagebox.showerror("Error", str(e))
                break
                
            finally:
                if sock:
                    sock.close()
                
        if retry_count == self.max_retries:
            messagebox.showerror("Error", "Connection timed out after maximum retries")

    def derive_key(self):
        # Derive 32-byte key from password using SHA-256
        self.encryption_key = hashlib.sha256(self.password.encode()).digest()

    def run(self):
        # Initialize encryption key before starting
        self.derive_key()
        self.root.mainloop()

if __name__ == "__main__":
    app = ESP32GUI()
    app.run()