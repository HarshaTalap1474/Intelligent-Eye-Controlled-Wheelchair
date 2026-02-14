import serial
import time

# REPLACE THIS with your 'Outgoing' COM port number
COM_PORT = 'COM10' 
BAUD_RATE = 115200 # ESP32 is faster than Arduino

try:
    print(f"Connecting to {COM_PORT}...")
    # timeout=1 ensures we don't hang forever
    esp32 = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    time.sleep(2) # Give connection time to stabilize
    print("✅ Connected to Wheelchair!")

    while True:
        cmd = input("Command (F/B/L/R/S or 0-9): ").upper()
        if cmd == 'Q': break
        
        # Send command
        esp32.write(cmd.encode())
        print(f"Sent: {cmd}")

except serial.SerialException:
    print(f"❌ ERROR: Could not open {COM_PORT}. Check Bluetooth settings.")
    print("Tip: Remove device from Windows and Pair again if stuck.")
finally:
    if 'esp32' in locals() and esp32.is_open:
        esp32.close()