# ==============================================================================
#      PATCH: ADD THIS TO THE TOP OF THE OLD SCRIPT TO FIX BLUETOOTH
# ==============================================================================
import threading
import queue
import asyncio
from bleak import BleakScanner, BleakClient

# --- CONFIG ---
DEVICE_NAME = "Wheelchair_BLE"
CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"

# --- GLOBAL QUEUE (The bridge between Camera and Bluetooth) ---
ble_queue = queue.Queue()

# --- THE WORKER FUNCTION (Runs in background) ---
def bluetooth_background_worker():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    async def async_main():
        while True:
            print(f"[BLE] Scanning for {DEVICE_NAME}...")
            try:
                device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=5.0)
                if not device:
                    await asyncio.sleep(2.0)
                    continue
                
                print(f"[BLE] ✅ Connecting...")
                async with BleakClient(device) as client:
                    print(f"[BLE] ✅ CONNECTED!")
                    last_msg = ""
                    
                    while client.is_connected:
                        try:
                            # 1. Get command from Old Script
                            cmd = ble_queue.get(timeout=0.1)
                            
                            # 2. Convert string to Byte
                            protocol_map = {"FORWARD": b'F', "REVERSE": b'B', 
                                            "LEFT": b'L', "RIGHT": b'R', 
                                            "STOP": b'S', "IDLE": b'S'}
                            char_code = protocol_map.get(cmd, b'S')
                            
                            # 3. Send ONLY if changed (Prevents lag)
                            if cmd != last_msg:
                                print(f"[BLE] Sending: {cmd}")
                                await client.write_gatt_char(CHARACTERISTIC_UUID, char_code, response=False)
                                last_msg = cmd
                                
                        except queue.Empty:
                            continue
                        except Exception as e:
                            print(f"[BLE] Error: {e}")
                            break # Trigger reconnect
            except Exception as e:
                print(f"[BLE] Connection Failed: {e}")
                await asyncio.sleep(2.0)

    loop.run_until_complete(async_main())

# --- START THE THREAD (Put this before 'while True') ---
bt_thread = threading.Thread(target=bluetooth_background_worker, daemon=True)
bt_thread.start()

# ==============================================================================
#      INSTRUCTION: HOW TO USE IN OLD LOOP
# ==============================================================================
# Inside your main 'while True' loop, simply add:
#
# ble_queue.put(current_state)
#
# Example:
# if current_state != last_printed_state:
#     print(f"DEVICE_SIGNAL: {current_state}")
#     ble_queue.put(current_state)  <-- ADD THIS LINE
# ==============================================================================