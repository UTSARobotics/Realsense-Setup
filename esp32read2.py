from machine import UART
import time

uart = UART(2, baudrate=420000, bits=8, parity=None, stop=1, rx=27, tx=16)

# CRC8 DVB-S2
def crc8_dvb_s2(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


buffer = bytearray()

def parse_ghst():
    global buffer

    if uart.any():
        buffer.extend(uart.read())

    while len(buffer) >= 4:  # minimum: addr + len + type + crc
        addr = buffer[0]
        length = buffer[1]

        full_frame_len = 2 + length

        if len(buffer) < full_frame_len:
            return  # wait for more bytes

        frame = buffer[:full_frame_len]
        buffer = buffer[full_frame_len:]

        # Structure:
        # [ADDR][LEN][TYPE][PAYLOAD...][CRC]

        frame_type = frame[2]
        received_crc = frame[-1]

        # CRC over TYPE + PAYLOAD
        calculated_crc = crc8_dvb_s2(frame[2:-1])

        if calculated_crc != received_crc:
            print("CRC FAIL")
            continue

        payload = frame[3:-1]

        print("Valid Frame")
        print("Type:", frame_type)
        print("Payload:", payload)

        # Decode first 5 channels (need at least 8 payload bytes)
        if len(payload) >= 8:
            ch1 = (payload[0] | (payload[1] << 8)) & 0x0FFF
            ch2 = ((payload[1] >> 4) | (payload[2] << 4)) & 0x0FFF
            ch3 = (payload[3] | (payload[4] << 8)) & 0x0FFF
            ch4 = ((payload[4] >> 4) | (payload[5] << 4)) & 0x0FFF
            ch5 = (payload[6] | (payload[7] << 8)) & 0x0FFF

            # 2-position switch logic
            # weight = 100 implied by full 0-4096 range
            ch5_switch = 1 if ch5 >= 2000 else 0

            print(
                "CH1:", ch1,
                "CH2:", ch2,
                "CH3:", ch3,
                "CH4:", ch4,
                "CH5:", ch5,
                "SW:", ch5_switch
            )


while True:
    parse_ghst()
    time.sleep_ms(2)
