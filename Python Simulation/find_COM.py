import serial.tools.list_ports

def list_ports():
    print("Searching for COM ports...")
    ports = serial.tools.list_ports.comports()
    for port in ports:
        print(f"Found: {port.device} - {port.description}")

if __name__ == "__main__":
    list_ports()