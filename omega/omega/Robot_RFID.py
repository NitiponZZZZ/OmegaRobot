#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int16
import serial

RS485 = serial.Serial(
    port='/dev/ttl_rfid',
    baudrate = 9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0.1
)
class RFIDNode(Node):
    def __init__(self):
        super().__init__("RFID")
        self.rf_pub = self.create_publisher(
            Int16, "rfid_data", 20)
        self.read_timer = self.create_timer(0.05, self.read_timer_callback)
        self.station = 0
        self.Id_ = ["E0840000134216067C","E0840000134017267D","E08400001338494970","E08400001337661678","E08300000197886572","E08400001341097878","E0840000134083977A","F99900180420040377","E08400001342519373","E08400001342905170"] 
        self.get_logger().info('RFID Started')
        
    def read_timer_callback(self):
        
        FIDs = str(RS485.readline().decode())
        IDs = FIDs.strip('$').split('#')

        for i in range(0,len(self.Id_)):
            if str(IDs[0]) == str(self.Id_[i]):
                self.station = i+1

        self.stations = Int16()
        self.stations.data = self.station
        self.rf_pub.publish(self.stations)


def main():
    rclpy.init()
    RF = RFIDNode()
    rclpy.spin(RF)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
