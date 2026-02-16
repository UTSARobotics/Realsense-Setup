from pymavlink import mavutil
import time
import sys


def main():
    print("--- COYOTE ROVER BRAIN STARTING ---")

    # 1. Connect
    connection_string = 'udp:127.0.0.1:14550'
    print(f"Connecting to Rover on {connection_string}...")
    master = mavutil.mavlink_connection(connection_string)

    # 2. Wait for Heartbeat
    master.wait_heartbeat()

    # [FIXED LINE BELOW]
    print(f"Heartbeat from System {master.target_system}, Component {master.target_component}")
    print("Connection Successful!")

    # 3. Listen for data
    while True:
        try:
            msg = master.recv_match(type='ATTITUDE', blocking=True)
            if msg:
                print(f"Roll: {msg.roll:.2f} | Pitch: {msg.pitch:.2f} | Yaw: {msg.yaw:.2f}")
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error: {e}")


if __name__ == "__main__":
    main()
