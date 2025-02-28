import serial
import time

# Configuration
PORT = "COM5"  # Change this to match your system (e.g., "/dev/ttyUSB0" for Linux)
BAUDRATE = 115200
BIN_FILE_PATH = "firmware.bin"  # Path to the binary file

# UART Command (Modify if your bootloader expects a specific command to start transfer)
BOOTLOADER_START_CMD = b"START_UPDATE\n"

def send_bin_file():
    try:
        # Open the serial port
        with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
            print(f"Connected to {PORT} at {BAUDRATE} baud")

            # Wait for the bootloader to be ready (optional delay)
            time.sleep(1)

            # Notify bootloader to start firmware update (if required)
            # ser.write(BOOTLOADER_START_CMD)
            # time.sleep(0.5)  # Wait for bootloader acknowledgment (modify as needed)
            while(1):
                cmd = ser.read(1)
                print(cmd)
                if(cmd == b"F"):
                    cmd = ser.read(9)
                    if(cmd == b"ILE_INFO"):
                        ser.write(b"123s456s")
                        print("sent")
            

            # # Open the .bin file in binary mode
            # with open(BIN_FILE_PATH, "rb") as f:
            #     binary_data = f.read()
            
            # # Send the binary file over UART
            # print(f"Sending {len(binary_data)} bytes...")
            # ser.write(binary_data)

            # # Wait for bootloader response (if needed)
            # response = ser.read(100)  # Read up to 100 bytes (adjust as needed)
            # print("Bootloader Response:", response.decode(errors="ignore"))

            # print("File transfer completed.")
    
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    send_bin_file()
