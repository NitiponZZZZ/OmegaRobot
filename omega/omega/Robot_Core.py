#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Int16
from sensor_msgs.msg import Joy
import serial
from PIL import Image 

cmd_hex = [0x1b, 0xa1, 0x00, 0x40, 0xbf, 0x05]
Get_Navigation = bytes(cmd_hex) 

ser = serial.Serial("/dev/ttl_magnetic", baudrate=115200, timeout=0.005)

class Robot(Node):
    def __init__(self):
        super().__init__('robot_core')

        self.leftwheel_pub = self.create_publisher(
            Float32, 'wheel_command_left', 10)
        self.rightwheel_pub = self.create_publisher(
            Float32, 'wheel_command_right', 10)
        self.joy_sub = self.create_subscription(
            Joy, "joy", self.joy_callback, 20)
        self.rf_sub = self.create_subscription(
            Int16, "rfid_data", self.rfid_callback, 20)

        self.timer = self.create_timer(0.04, self.timer_callback)
        self.timer2 = self.create_timer(0.05, self.PID_timer_callback)

        self.cmd_l = Float32()
        self.cmd_r = Float32()

        # Joy
        self.cmd_l3 = Float32()
        self.cmd_r3 = Float32()
        self.LeftHat = 0.0
        self.RightHatX = 0.0
        self.RightHatY = 0.0
        self.speedFW = 1.0
        self.speedT = 0.35
        self.L1_wheel_speed = 0.0
        self.R1_wheel_speed = 0.0
        self.L2_wheel_speed = 0.0
        self.R2_wheel_speed = 0.0
        self.Enable = 0
        self.AnaL2 = 0.0
        self.AnaR2 = 0.0

        # PID constants
        self.P = 0;
        self.I = 0;
        self.D = 0;

        self.KP = 0.7  # Proportional gain
        self.KI = 0.0  # Integral gain
        self.KD = 0.0 # Derivative gain

        self.error = 0
        self.lastError = 0
        self.sensors = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.left_speed = 0.00
        self.right_speed = 0.00

        self.base_speed = 0.3
        self.max_speed = 1.0
        self.min_speed = -self.base_speed

        self.get_logger().info("robot core is running")

        self.num1 = Image.open("/home/robot5g/dev_ws/src/omega/omega/num1.jpg") 
        self.num2 = Image.open("/home/robot5g/dev_ws/src/omega/omega/num2.jpg") 
        self.num3 = Image.open("/home/robot5g/dev_ws/src/omega/omega/num3.jpg") 
        self.num4 = Image.open("/home/robot5g/dev_ws/src/omega/omega/num4.jpg") 
        self.num5 = Image.open("/home/robot5g/dev_ws/src/omega/omega/num5.jpg") 
        self.num6 = Image.open("/home/robot5g/dev_ws/src/omega/omega/num6.jpg") 
        self.num7 = Image.open("/home/robot5g/dev_ws/src/omega/omega/num7.jpg") 
        self.num8 = Image.open("/home/robot5g/dev_ws/src/omega/omega/num8.jpg") 
        self.num9 = Image.open("/home/robot5g/dev_ws/src/omega/omega/num9.jpg") 
        self.num10 = Image.open("/home/robot5g/dev_ws/src/omega/omega/num10.jpg") 



    def joy_callback(self, joy_in):
        self.LeftHat = joy_in.axes[1]
        self.RightHatX = joy_in.axes[3]
        self.RightHatY = joy_in.axes[4]
        self.Enable = joy_in.buttons[4]
        self.AnaL2 = joy_in.axes[2]
        self.AnaR2 = joy_in.axes[5]


    def rfid_callback(self, rf_in):
        rf_data = rf_in.data 
        # while rf_data == 1:
        #     self.num1.show()
        # while rf_data == 2:
        #     self.num2.show()
        # while rf_data == 3:
        #     self.num3.show()
        # while rf_data == 4:
        #     self.num4.show()
        # while rf_data == 5:
        #     self.num5.show()
        # while rf_data == 6:
        #     self.num6.show()
        # while rf_data == 7:
        #     self.num7.show()
        # while rf_data == 8:
        #     self.num8.show()
        # while rf_data == 9:
        #     self.num9.show()
        # while rf_data == 10:
        #     self.num10.show()
            

    def timer_callback(self):
        if int(self.Enable):
            self.cmd_r.data = 0.00
            self.cmd_l.data = 0.00
            if self.LeftHat > 0.01 or self.LeftHat < -0.01:  # Forward & Backward
                self.cmd_l3.data = self.LeftHat*self.speedFW
                self.cmd_r3.data = self.LeftHat*self.speedFW

            if self.AnaL2 < -0.2 or self.AnaR2 < -0.2:
                if self.AnaL2 < -0.2:
                    #self.get_logger().info("Rotate Left")
                    self.cmd_l3.data = -(-self.AnaL2*self.speedT)
                    self.cmd_r3.data = (-self.AnaL2*self.speedT)

                elif self.AnaR2 < -0.2:
                    #self.get_logger().info("Rotate Right")
                    self.cmd_l3.data = (-self.AnaR2*self.speedT)
                    self.cmd_r3.data = -(-self.AnaR2*self.speedT)

            if self.RightHatX > 0.02 or self.RightHatX < -0.02 and self.LeftHat > 0.02 or self.LeftHat < -0.02:  # curve
                if self.RightHatX > 0.02 and self.LeftHat > 0.02:
                    #self.get_logger().info("Curve forward Left")
                    self.cmd_l3.data = self.RightHatX*(self.speedT*0.5)
                    self.cmd_r3.data = self.RightHatX*self.speedT

                elif self.RightHatX < -0.02 and self.LeftHat > 0.02:
                    #self.get_logger().info("Curve forward Right")
                    self.cmd_l3.data = -1*self.RightHatX*self.speedT
                    self.cmd_r3.data = -1*self.RightHatX*(self.speedT*0.5)

                elif self.RightHatX > 0.02 and self.LeftHat < -0.02:
                    #self.get_logger().info("Curve backward Left")
                    self.cmd_l3.data = -(self.RightHatX*(self.speedT*0.5))
                    self.cmd_r3.data = -(self.RightHatX*self.speedT)

                elif self.RightHatX < -0.02 and self.LeftHat < -0.02:
                    #self.get_logger().info("Curve backward Right")
                    self.cmd_l3.data = -(-self.RightHatX*self.speedT)
                    self.cmd_r3.data = -(-self.RightHatX*(self.speedT*0.5))

            elif (self.LeftHat < 0.01 and self.LeftHat > -0.01 and self.RightHatX < 0.02 and self.RightHatX > -0.02 and self.LeftHat < 0.02 and self.LeftHat > -0.02 and self.AnaL2 > -0.2 and self.AnaR2 > -0.2):
                # self.get_logger().info("Stop")
                self.cmd_l3.data = 0.00
                self.cmd_r3.data = 0.00

            self.leftwheel_pub.publish(self.cmd_l3)
            self.rightwheel_pub.publish(self.cmd_r3)
            
        elif not int(self.Enable):
            self.leftwheel_pub.publish(self.cmd_l)
            self.rightwheel_pub.publish(self.cmd_r)


    def PID_timer_callback(self):

        ser.write(Get_Navigation)
        ser.flush()
        reading = ser.readline()
        val1 = reading[3]
        val2 = reading[4]
        con1=bin(val1)[2:].zfill(8)
        con2=bin(val2)[2:].zfill(8)
        
        for i in range(8):
            self.sensors[i] = int(con1[i])
        for i in range(8):
            self.sensors[i+8] = int(con2[i])
  
        if self.sensors[0] == 0 and self.sensors[1] == 0 and self.sensors[2] == 0 and self.sensors[3] == 0 and self.sensors[4] == 0 and self.sensors[5] == 0 and self.sensors[6] == 0 and self.sensors[7] == 0 and self.sensors[8] == 0 and self.sensors[9] == 0 and self.sensors[10] == 0 and self.sensors[11] == 0 and self.sensors[12] == 0 and self.sensors[13] == 0 and self.sensors[14] == 0 and self.sensors[15] == 0 :
            positions = 0.3
        else:        
            positions = (self.sensors[0] * 0.6 + self.sensors[1] * 0.5625 + self.sensors[2] * 0.525 + self.sensors[3] * 0.4875 + self.sensors[4] * 0.45 + self.sensors[5] * 0.4125 + self.sensors[6] * 0.375 + self.sensors[7] * 0.3375 + self.sensors[8] * 0.3 + self.sensors[9] * 0.2625 + self.sensors[10]* 0.225 + self.sensors[11]* 0.1875 + self.sensors[12] * 0.15 + self.sensors[13] * 0.1125 + self.sensors[14]* 0.075 + self.sensors[15] * 0.0375) / (self.sensors[15] + self.sensors[14]+ self.sensors[13]+ self.sensors[12]+ self.sensors[11]+ self.sensors[10]+ self.sensors[9]+ self.sensors[8]+ self.sensors[7]+ self.sensors[6]+ self.sensors[5]+ self.sensors[4]+ self.sensors[3]+ self.sensors[2]+ self.sensors[1]+ self.sensors[0])
        
        self.error = positions - 0.3
        self.P = self.error
        self.I = self.I + self.error
        self.D = self.error - self.lastError
        motor_speed = self.KP* self.P + self.KI * self.I +self.KD * self.D
        self.lastError = self.error
        
        self.right_speed = self.base_speed + motor_speed
        self.left_speed = self.base_speed - motor_speed
        
        if self.right_speed > self.max_speed:
            self.right_speed = self.max_speed
        if self.left_speed > self.max_speed:
            self.left_speed = self.max_speed
        
        if self.right_speed < self.min_speed:
            self.right_speed = self.min_speed
        if self.left_speed < self.min_speed: 
            self.left_speed = self.min_speed

        # if self.right_speed < 0:
        #     self.right_speed = 0
        # if self.left_speed < 0: 
        #     self.left_speed = 0

        self.cmd_r.data = float(self.right_speed)
        self.cmd_l.data = float(self.left_speed)

        self.get_logger().info(str(self.sensors))
        self.get_logger().info('out left rpm: %s' % self.left_speed)
        self.get_logger().info('out right rpm: %s' % self.right_speed)


def main():
    rclpy.init()

    rb = Robot()
    rclpy.spin(rb)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
