import time
import serial

#ros libraries
import rclpy
from rclpy.node import Node

#ros msgs
from std_msgs.msg import Int32MultiArray

#struct
import struct

#rover node
class rover_node(Node):
    
    def __init__(self):
        super().__init__('rover_node')
        
        #serial object too comunicate with esp32
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        #node subscriber
        self.sub = self.create_subscription(
            #array size always = 2
            Int32MultiArray(),
            'rover_tweet',
            self.rover_callback,
            10)
        
    #subscriber callback
    def rover_callback(self,msg):
            
            #parsing and packing msg into motor_id and motor_pwm
            #packed_motor_id = struct.pack('<i',msg.data[0])
            #packed_motor_duty = struct.pack('<i',msg.data[1])
            motor_id = msg.data[0]
            motor_duty = msg.data[1]
            
            
            #sending msg too esp32
            self.ser.write(f"{motor_id}\n".encode('utf-8'))
            self.ser.write(f"{motor_duty}\n".encode('utf-8'))
            
            #self.ser.write(packed_motor_id)
            #self.ser.write(packed_motor_duty)
            


#main function
def main():
    
    #running the node
    rclpy.init()
    rnode = rover_node()
    rclpy.spin(rnode)
    
    #if spin fails shutdown
    rnode.destroy_node()
    rclpy.shutdown()
