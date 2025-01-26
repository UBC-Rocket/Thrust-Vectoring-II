import tkinter as tk
from tkinter import messagebox
import socket
import threading
import time

class ESP32GUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("ESP32 WiFi Control")
        self.root.geometry("300x250")
        
        self.esp32_ip = "192.168.4.1"
        self.esp32_port = 80
        self.connected = False
        self.max_retries = 3
        
        # auto wifi status check
        self.connection_label = tk.Label(self.root, text="WiFi Status: Disconnected", fg="red")
        self.connection_label.pack(pady=10)
        
        self.send_button = tk.Button(self.root, text="Send 'A'", command=self.send_a, state='disabled')
        self.send_button.pack(pady=20)
        
        self.status_label = tk.Label(self.root, text="Status: Not Connected")
        self.status_label.pack(pady=20)
        
        # Start connection monitoring
        self.monitor_thread = threading.Thread(target=self.monitor_connection, daemon=True)
        self.monitor_thread.start()

    # attempting TCP connection to ESP32
    def check_wifi_connection(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2)
            result = sock.connect_ex((self.esp32_ip, self.esp32_port))
            sock.close()
            return result == 0
        except:
            return False

    # Continuously monitors connection status in background thread, updates GUI
    def monitor_connection(self):
        while True:
            connected = self.check_wifi_connection()
            if connected != self.connected:
                self.connected = connected
                self.root.after(0, self.update_connection_status)
            time.sleep(2)  

    # Updates GUI elements based on connection status
    def update_connection_status(self):
        if self.connected:
            self.connection_label.config(text="WiFi Status: Connected", fg="green")
            self.send_button.config(state='normal')
            self.status_label.config(text="Status: Ready")
        else:
            self.connection_label.config(text="WiFi Status: Disconnected", fg="red")
            self.send_button.config(state='disabled')
            self.status_label.config(text="Status: Not Connected")

    # 3 retries × 5 seconds per timeout = 15 seconds maximum
    def send_a(self):
        sock = None
        retry_count = 0
        while retry_count < self.max_retries:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5)
                sock.connect((self.esp32_ip, self.esp32_port))
                sock.send('A'.encode())
                # Keep connection open briefly to ensure processing
                time.sleep(0.1)
                # Read response to clear buffer
                try:
                    sock.recv(1024)
                except socket.timeout:
                    pass
                self.status_label.config(text="Status: Command Sent")
                break
            except socket.timeout:
                retry_count += 1
                if retry_count == self.max_retries:
                    messagebox.showerror("Error", "Connection timed out")
            finally:
                if sock:
                    sock.close()

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    app = ESP32GUI()
    app.run()