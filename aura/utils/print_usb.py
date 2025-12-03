import serial.tools.list_ports

def list_all_usb_ports():
    """
    List all USB serial ports on Raspberry Pi with detailed information.
    """
    ports = serial.tools.list_ports.comports()
    
    print("=" * 70)
    print("USB SERIAL PORTS ON RASPBERRY PI")
    print("=" * 70)
    
    if not ports:
        print("No USB serial ports found!")
        print("\nMake sure your device is connected.")
        return
    
    for i, port in enumerate(ports, 1):
        print(f"\n[Port {i}]")
        print(f"  Device:        {port.device}")
        print(f"  Name:          {port.name}")
        print(f"  Description:   {port.description}")
        print(f"  Hardware ID:   {port.hwid}")
        print(f"  Manufacturer:  {port.manufacturer}")
        print(f"  Product:       {port.product}")
        print(f"  Serial Number: {port.serial_number}")
        print(f"  Location:      {port.location}")
        print(f"  VID:PID:       {port.vid}:{port.pid}" if port.vid else "  VID:PID:       N/A")
        print("-" * 70)
    
    print(f"\nTotal ports found: {len(ports)}")
    print("=" * 70)

if __name__ == "__main__":
    list_all_usb_ports()
