import asyncio
from bleak import BleakScanner, BleakClient

# --- CONFIGURATION ---
DEVICE_NAME = "Wheelchair_BLE"
# These must match your ESP32 Code
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"

async def run():
    print("Scanning for Wheelchair...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME)
    
    if device is None:
        print(f"❌ Could not find device named '{DEVICE_NAME}'")
        return

    print(f"✅ Found {DEVICE_NAME}! Connecting...")

    try:
        async with BleakClient(device) as client:
            print("✅ Connected! LED should be ON.")
            
            while True:
                # FIX 1: Run input() in a separate thread so it doesn't block Bluetooth
                cmd = await asyncio.to_thread(input, "Command (F/B/L/R/S/0-9 or Q): ")
                cmd = cmd.upper()
                
                if cmd == 'Q':
                    print("Disconnecting...")
                    break
                
                if cmd in ['F', 'B', 'L', 'R', 'S', '0', '5', '9']:
                    try:
                        # FIX 2: response=False (Fire and Forget) - Faster & prevents timeouts
                        await client.write_gatt_char(CHARACTERISTIC_UUID, cmd.encode(), response=False)
                        print(f"Sent: {cmd}")
                    except Exception as e:
                        print(f"❌ Send Failed: {e}")
                else:
                    print("Invalid Command")

    except Exception as e:
        print(f"❌ Connection Error: {e}")

if __name__ == "__main__":
    asyncio.run(run())