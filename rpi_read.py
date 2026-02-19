from machine import Pin, UART
import time

uart = UART(2, baudrate=420000, bits=8, parity=None, stop=1, rx=27, tx=16)
print(uart)

def read_ghost_data():
    if uart.any():
        data = uart.read()
        print("Hello")
        print(data)
        
while True:
    read_ghost_data()
    time.sleep_ms(10)
    
