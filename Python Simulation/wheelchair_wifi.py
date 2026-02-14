import socket
import time

# --- CONFIGURATION ---
ROBOT_IP = "192.168.4.1"  # Default IP for ESP32 AP Mode
ROBOT_PORT = 80

print(f"Connecting to Robot at {ROBOT_IP}...")

try:
    # 1. Create a TCP Socket
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.settimeout(5) # 5 second timeout for connection
    
    # 2. Connect
    client.connect((ROBOT_IP, ROBOT_PORT))
    print("✅ Connected to Wheelchair via Wi-Fi!")
    print("LED on ESP32 should be ON now.")
    
    while True:
        cmd = input("Command (F/B/L/R/S or Q): ").upper()
        
        if cmd == 'Q':
            break
            
        if cmd in ['F', 'B', 'L', 'R', 'S', '0', '5', '9']:
            # Send data as bytes
            client.send(cmd.encode()) 
            print(f"Sent: {cmd}")
        else:
            print("Invalid Command")

except socket.timeout:
    print("❌ Connection Timed Out. Did you connect to 'Wheelchair_Robot' Wi-Fi?")
except ConnectionRefusedError:
    print("❌ Connection Refused. Is the ESP32 powered on?")
except Exception as e:
    print(f"❌ Error: {e}")
finally:
    client.close()
    print("Connection Closed.")