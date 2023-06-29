#!/usr/bin/env python3

import serial
import rclpy
from rclpy.node import Node
from math import sin, cos, pi
from nav_msgs.msg import Odometry
from serial.tools import list_ports
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist, Quaternion, TransformStamped

class odom_wheel_node(Node):
    def __init__(self, SerialPort, Baudrate):
        super().__init__("ODOMETRY_WHEEL")
        self.nodename = "ODOMETRY_WHEEL"
        self.get_logger().info(f"========={self.nodename}=========")

        # Initial Port
        self.serial_port = SerialPort
        self.baud_rate = Baudrate
        self._serial = self.serial_connect(self.serial_port, self.baud_rate)

        # Internal Data Variable
        self.x = 0.0 # position in xy plane
        self.y = 0.0
        self.th = 0.0
        self.dx = 0.0 # speeds in x/rotation
        self.dr = 0.0
        self.then = self.get_clock().now() # time for determining dx/dy
        self.enc_wheel_left = 0.0
        self.enc_wheel_right = 0.0
        self.enc_wheel_left_pv = 0
        self.enc_wheel_right_pv = 0  
        self.enc_wheel_left_mult = 0.0
        self.enc_wheel_right_mult = 0.0              
        self.prev_enc_left = 0
        self.prev_enc_right = 0
        self.ns_to_sec = 1.0e-9
# 33023.0, 34016.0
        # Parametersself.ticks_meter_R
        self.ticks_meter = float(self.declare_parameter('ticks_meter', 33519.5).value)
        # self.ticks_meter = float(self.declare_parameter('ticks_meter', 33023.0).value)
        # self.ticks_meter_R = float(self.declare_parameter('ticks_meter_R', 34016.0).value)
        
        self.base_width = float(self.declare_parameter('base_width', 0.28).value)
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.encoder_min = self.declare_parameter('encoder_min', -2147483648).value
        self.encoder_max = self.declare_parameter('encoder_max', 2147483647).value
        self.encoder_low_wrap = self.declare_parameter('wheel_low_wrap', (
            self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min).value
        self.encoder_high_wrap = self.declare_parameter('wheel_high_wrap', (
            self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min).value
        
        # Subscribe Topic cmd_vel
        self.sub_cmd_vel = self.create_subscription(Twist, "cmd_vel_accel_decel", self.callback_cmd_vel, 10) #cmd_vel_accel_decel

        # Publish Topic odom_wheel
        self.pub_odom_wheel = self.create_publisher(Odometry, "odometry/wheel", 10)

        # TF BroadCaster
        self.odom_broadcaster = TransformBroadcaster(self)

        # RATE TIMER HZ
        self.rate = self.declare_parameter("rate", 100.0).value
        self.create_timer(1.0/self.rate, self.update)

    def callback_cmd_vel(self, twist):
        speed = twist.linear.x
        angular = twist.angular.z
        print(speed, angular)
        wheel_left_set = ((speed + angular) * self.ticks_meter)
        wheel_right_set = ((speed - angular) * self.ticks_meter)
        str_send = str(wheel_left_set) + "," + str(wheel_right_set) + '\n'
        self.serial_write(str_send)

    def update(self):
        self.enc_wheel_left, self.enc_wheel_right = self.wheel_enc_ticks()

        # Calculate Time
        now = self.get_clock().now()
        elapsed = now - self.then
        self.then = now
        elapsed = elapsed.nanoseconds / self.ns_to_sec

        # Calculate Odometry
        distance_wheel_left = (self.enc_wheel_left - self.enc_wheel_left_pv) / self.ticks_meter
        distance_wheel_right = (self.enc_wheel_right - self.enc_wheel_right_pv) / self.ticks_meter
        
        # print(self.enc_wheel_left ,self.enc_wheel_right)
        self.get_logger().info(f"========={self.enc_wheel_left, self.enc_wheel_right}=========")
             

        # Distance traveled is the average of the two wheels 
        dist = (distance_wheel_left + distance_wheel_right) / 2

        # This approximation works (in radians) for small angles
        th = (distance_wheel_left - distance_wheel_right) / self.base_width

        self.dx = dist / elapsed
        self.dr = th / elapsed

        if dist != 0:
            # Calculate Distance Traveled in x and y
            x = cos(th) * dist
            y = -sin(th) * dist
            # Calculate the final position of the robot
            self.x = self.x + (cos(self.th) * x - sin(self.th) * y)
            self.y = self.y + (sin(self.th) * x + cos(self.th) * y)
        if th != 0:
            self.th = self.th + th   

        # Publish Info odom_wheel
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(self.th / 2)
        quaternion.w = cos(self.th / 2)

        transform_stamped_msg = TransformStamped()
        transform_stamped_msg.header.stamp = now.to_msg()
        transform_stamped_msg.header.frame_id = self.odom_frame
        transform_stamped_msg.child_frame_id = self.base_frame
        transform_stamped_msg.transform.translation.x = self.x
        transform_stamped_msg.transform.translation.y = self.y
        transform_stamped_msg.transform.translation.z = 0.0
        transform_stamped_msg.transform.rotation.w = quaternion.w
        transform_stamped_msg.transform.rotation.x = quaternion.x
        transform_stamped_msg.transform.rotation.y = quaternion.y
        transform_stamped_msg.transform.rotation.z = quaternion.z

        # self.odom_broadcaster.sendTransform(transform_stamped_msg)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion
        odom.twist.twist.linear.x = self.dx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.dr
        self.pub_odom_wheel.publish(odom)

        self.enc_wheel_left_pv = self.enc_wheel_left
        self.enc_wheel_right_pv = self.enc_wheel_right

    def wheel_enc_ticks(self):
        recv = self._readline().decode("utf-8")
        buf_lr = recv.split(",")
        enc_wheel_left_raw = int(buf_lr[0])
        enc_wheel_right_raw = int(buf_lr[1])
        
        if enc_wheel_left_raw < self.encoder_low_wrap and self.prev_enc_left > self.encoder_high_wrap:
            self.enc_wheel_left_mult = self.enc_wheel_left_mult + 1
            
        if enc_wheel_left_raw > self.encoder_high_wrap and self.prev_enc_left < self.encoder_low_wrap:
            self.enc_wheel_left_mult = self.enc_wheel_left_mult - 1
            
        enc_wheel_left = 1.0 * (enc_wheel_left_raw + self.enc_wheel_left_mult * (self.encoder_max - self.encoder_min))
        self.prev_enc_left = enc_wheel_left_raw
        
        if enc_wheel_right_raw < self.encoder_low_wrap and self.prev_enc_right > self.encoder_high_wrap:
            self.enc_wheel_right_mult = self.enc_wheel_right_mult + 1
            
        if enc_wheel_right_raw > self.encoder_high_wrap and self.prev_enc_right < self.encoder_low_wrap:
            self.enc_wheel_right_mult = self.enc_wheel_right_mult - 1
            
        enc_wheel_right = 1.0 * (enc_wheel_right_raw + self.enc_wheel_right_mult * (self.encoder_max - self.encoder_min))
        self.prev_enc_right = enc_wheel_right_raw
        
        return enc_wheel_left, enc_wheel_right
    
    def serial_connect(self, serial_port, baudrate):
        _serial_port = serial.Serial(serial_port, baudrate)
        return _serial_port
    
    def serial_read(self, _serial, range_buf=100):
        return _serial.read(range_buf)

    def serial_write(self, speed_vel):
        res = bytes(speed_vel, 'utf-8')
        self._serial.write(res)
        
    def _readline(self):
        eol = b'\r\n'
        leneol = len(eol)
        line = bytearray()
        while True:
            reader = self.serial_read(self._serial, 1)
            if reader:
                line += reader
                if line[-leneol:] == eol:
                    break
            else:
                break
        return bytes(line)

STR_USBPORT = "USB VID:PID=16C0:0483 SER=7442840 LOCATION=1-6.1:1.0"
_baudrate = 9600

def getControl_drivePort():
    for port in list(list_ports.comports()):
        print(port[2])
        if port[2] == STR_USBPORT:
            return port[0]

def main(args=None):
    rclpy.init(args=args)
    _serial_port = getControl_drivePort()
    _odom_wheel_node = odom_wheel_node(_serial_port, _baudrate)
    rclpy.spin(_odom_wheel_node)
    _odom_wheel_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()    
