import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class AccelDecel(Node):

    def __init__(self):
        super().__init__('Accel_Decel')
        self.nodename = "ACCEL_DECEL_NODE"
        self.get_logger().info(f"{self.nodename} STARTED")

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel_accel_decel', 1)

        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.callback_cmd_vel, 1)

        self.subscription  # prevent unused variable warning

        self.pv = Twist()
        self.sp = Twist()

        timer_period = 1/60 # seconds
        self.timer = self.create_timer(timer_period, self.update)

  
    def update(self):

        self.sp.linear.x = round(self.sp.linear.x, 2)
        # self.get_logger().info(f"{self.sp.linear.x} SP")

        if self.sp.linear.x == 0.0 and abs(self.sp.linear.x - self.pv.linear.x) < 0.01:
            self.pv.linear.x = 0.0

        if self.sp.linear.x > self.pv.linear.x:
            self.pv.linear.x = self.pv.linear.x + 0.01
            

        elif self.sp.linear.x < self.pv.linear.x:
            self.pv.linear.x = self.pv.linear.x - 0.01

        self.pv.linear.x = round(self.pv.linear.x, 2)

        # self.get_logger().info(f"{self.pv.linear.x} PV")
        
        self.pv.angular.z = self.sp.angular.z

        self.publisher_.publish(self.pv)

    def callback_cmd_vel(self,twist):
       
        self.sp = twist

def main(args=None):
    rclpy.init(args=args)

    accel_decel = AccelDecel()

    rclpy.spin(accel_decel)

    accel_decel.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()