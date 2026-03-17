import tkinter as tk
from tkinter import scrolledtext
import socket
import threading
import json
import os
import time

# --- CONFIGURATION ---
ESP32_IP = "192.168.2.192"
PORT = 23
CONFIG_FILE = "pid_settings.json"
WATCHDOG_TIMEOUT = 5.0  # Seconds to wait before assuming connection is "stale"

class FlightControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 PID Tuner - Initializing...")
        
        self.settings = self.load_settings()

        # --- Telemetry Display ---
        self.log_area = scrolledtext.ScrolledText(root, width=150, height=15, bg="#1e1e1e", fg="#00ff00")
        self.log_area.grid(row=0, column=0, columnspan=6, padx=10, pady=10)

        # --- PID Input Fields ---
        self.entries = {}
        fields = [
            ("P Gain (Rate)", "rp", 1, 0), ("P Gain (Angle)", "rpa", 1, 2), ("M1 Offset", "M1", 1, 4),
            ("I Gain (Rate)", "ri", 2, 0), ("I Gain (Angle)", "ria", 2, 2), ("M2 Offset", "M2", 2, 4),
            ("D Gain (Rate)", "rd", 3, 0), ("D Gain (Angle)", "rda", 3, 2), ("M3 Offset", "M3", 3, 4),
                                                                             ("M4 Offset", "M4", 4, 4),
            ("Yaw P Gain",    "yp", 5, 0),
            ("Yaw I Gain",    "yi", 5, 2),
            ("Yaw D Gain",    "yd", 6, 0),
            ("Vel P Gain",    "vp", 5, 4),  ("Alt P Gain",    "ap", 5, 6),              ("Vel I Gain",    "vi", 6, 4),  ("Alt I Gain",    "ai", 6, 6), 
            ("Vel D Gain",    "vd", 7, 4),  ("Alt D Gain",    "ad", 7, 6),  
            ("Pitch Offset",  "po", 7, 0),
            ("Roll Offset",   "ro", 7, 2),
        ]

        for label_text, key, r, c in fields:
            tk.Label(root, text=label_text).grid(row=r, column=c, sticky="e")
            entry = tk.Entry(root)
            entry.grid(row=r, column=c+1)
            entry.insert(0, self.settings.get(key, "0.0"))
            self.entries[key] = entry

        self.send_btn = tk.Button(root, text="Update & Save PIDs", command=self.manual_send, bg="#2c3e50", fg="white")
        self.send_btn.grid(row=8, column=1, pady=10)

        self.clear_btn = tk.Button(root, text="Clear Log", command=lambda: self.log_area.delete('1.0', tk.END))
        self.clear_btn.grid(row=8, column=0)

        # --- Networking State ---
        self.sock = None
        self.running = True
        self.is_connected = False
        self.last_data_time = 0 # Watchdog timestamp
        
        self.conn_thread = threading.Thread(target=self.connection_manager, daemon=True)
        self.conn_thread.start()

    def load_settings(self):
        default = {
                "rp":"0.15", "rpa":"5.0", "ri":"0.01", "ria":"0.00", "rd":"0.001", "rda":"0.00",
                "yp":"0.00", "yi":"0.00", "yd":"0.00",
                "vp":"0.00", "vi":"0.00", "vd":"0.00",
                "ap":"0.00", "ai":"0.00", "ad":"0.00",
                "M1":"0.00", "M2":"0.00", "M3":"0.00", "M4":"0.00",
                "po":"0.00", "ro":"0.00"   
        }
        if os.path.exists(CONFIG_FILE):
            try:
                with open(CONFIG_FILE, 'r') as f: return json.load(f)
            except: return default
        return default

    def save_settings(self):
        current_data = {key: entry.get() for key, entry in self.entries.items()}
        with open(CONFIG_FILE, 'w') as f: json.dump(current_data, f)

    def connection_manager(self):
        """Persistent loop that manages connection health and watchdogs."""
        while self.running:
            if not self.is_connected:
                self.root.after(0, self.update_status, "Connecting...")
                try:
                    if self.sock:
                        try: self.sock.close()
                        except: pass
                    
                    self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.sock.settimeout(3)
                    self.sock.connect((ESP32_IP, PORT))
                    self.sock.settimeout(None) 
                    
                    self.is_connected = True
                    self.last_data_time = time.time() # Reset watchdog on connect
                    
                    self.root.after(0, self.update_status, "CONNECTED")
                    self.root.after(0, self.update_log, "SYSTEM: Handshake Successful.\n")

                    rx_thread = threading.Thread(target=self.receive_loop, daemon=True)
                    rx_thread.start()

                    time.sleep(0.5)
                    self.send_pid_commands()

                    # --- WATCHDOG CHECK LOOP ---
                    while rx_thread.is_alive() and self.is_connected:
                        time.sleep(1)
                        # Check if data has stalled
                        time_since_last_packet = time.time() - self.last_data_time
                        if time_since_last_packet > WATCHDOG_TIMEOUT:
                            self.root.after(0, self.update_log, f"WATCHDOG: No data for {WATCHDOG_TIMEOUT}s. Forcing Reset.\n")
                            self.is_connected = False # This triggers a break and reconnect
                            break

                except (socket.error, Exception):
                    self.is_connected = False
                    self.root.after(0, self.update_status, "OFFLINE (Retrying...)")
                    time.sleep(3)
            
            time.sleep(0.5)

    def receive_loop(self):
        while True:
            try:
                resp = self.sock.recv(4096)
                if not resp:
                    break
                
                # UPDATE WATCHDOG TIMESTAMP
                self.last_data_time = time.time()
                
                text = resp.decode('utf-8', errors='ignore')
                self.root.after(0, self.update_log, text)
            except:
                break
        
        self.is_connected = False

    def update_status(self, msg):
        self.root.title(f"ESP32 PID Tuner - {msg}")

    def update_log(self, text):
        self.log_area.insert(tk.END, text)
        self.log_area.see(tk.END)

    def send_pid_commands(self):
        if not self.sock or not self.is_connected:
            return False
        
        commands = [
            f"RP {self.entries['rp'].get()}\n",  f"RPa {self.entries['rpa'].get()}\n",
            f"RI {self.entries['ri'].get()}\n",  f"RIa {self.entries['ria'].get()}\n",
            f"RD {self.entries['rd'].get()}\n",  f"RDa {self.entries['rda'].get()}\n",
            f"YP {self.entries['yp'].get()}\n",
            f"YI {self.entries['yi'].get()}\n",
            f"YD {self.entries['yd'].get()}\n",
            f"VP {self.entries['vp'].get()}\n",  # velXPID
            f"VI {self.entries['vi'].get()}\n",  # velXPID
            f"VD {self.entries['vd'].get()}\n",  # velXPID
            f"AP {self.entries['ap'].get()}\n",  # altPID
            f"AI {self.entries['ai'].get()}\n",  # altPID
            f"AD {self.entries['ad'].get()}\n",  # altPID
            f"M1 {self.entries['M1'].get()}\n",  f"M2 {self.entries['M2'].get()}\n",
            f"M3 {self.entries['M3'].get()}\n",  f"M4 {self.entries['M4'].get()}\n",
            f"PO {self.entries['po'].get()}\n",  
            f"RO {self.entries['ro'].get()}\n",  
        ]

        try:
            for cmd in commands:
                self.sock.sendall(cmd.encode())
                time.sleep(0.1)
            return True
        except:
            self.is_connected = False
            return False

    def manual_send(self):
        self.save_settings()
        if self.send_pid_commands():
            self.update_log("CLIENT: Manual Sync Successful.\n")
        else:
            self.update_log("CLIENT: Error - Not Connected.\n")

if __name__ == "__main__":
    root = tk.Tk()
    gui = FlightControllerGUI(root)
    def on_close():
        gui.running = False
        root.destroy()
    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()
