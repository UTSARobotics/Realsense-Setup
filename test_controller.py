try:
    esp32 = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
except:
    print("ESP32 not found on ACM0, try USB0")
    esp32 = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
