import math

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class StaticFramePublisher(Node):
    def __init__(self):
        super().__init__('Tf_Static_Publisher')
        self._tf_publish_rate = StaticTransformBroadcaster(self)

        # Publish static transforms once at startup
        self._tf_publish_rate.sendTransform(self.make_transforms())

    def make_transforms(self):
        map_to_odom = TransformStamped()
        map_to_odom.header.frame_id = 'map'
        map_to_odom.header.stamp = self.get_clock().now().to_msg()
        map_to_odom.child_frame_id = 'odom'
        map_to_odom.transform.translation.x = 0.0
        map_to_odom.transform.translation.y = 0.0
        map_to_odom.transform.translation.z = 0.0
        map_to_odom.transform.rotation.w = 1.0
        map_to_odom.transform.rotation.x = 0.0
        map_to_odom.transform.rotation.y = 0.0
        map_to_odom.transform.rotation.z = 0.0

        base_footprint_to_base_link = TransformStamped()
        base_footprint_to_base_link.header.frame_id = 'base_footprint'
        base_footprint_to_base_link.header.stamp = self.get_clock().now().to_msg()
        base_footprint_to_base_link.child_frame_id = 'base_link'
        base_footprint_to_base_link.transform.translation.x = -0.2
        base_footprint_to_base_link.transform.translation.y = 0.0
        base_footprint_to_base_link.transform.translation.z = 0.15 #0.125
        base_footprint_to_base_link.transform.rotation.w = 1.0
        base_footprint_to_base_link.transform.rotation.x = 0.0
        base_footprint_to_base_link.transform.rotation.y = 0.0
        base_footprint_to_base_link.transform.rotation.z = 0.0

        base_link_to_bno055 = TransformStamped()
        base_link_to_bno055.header.frame_id = 'base_link'
        base_link_to_bno055.header.stamp = self.get_clock().now().to_msg()
        base_link_to_bno055.child_frame_id = 'bno055'
        base_link_to_bno055.transform.translation.x = 0.05 #-0.22
        base_link_to_bno055.transform.translation.y = 0.0
        base_link_to_bno055.transform.translation.z = 0.215
        base_link_to_bno055.transform.rotation.w = 1.0
        base_link_to_bno055.transform.rotation.x = 0.0
        base_link_to_bno055.transform.rotation.y = 0.0
        base_link_to_bno055.transform.rotation.z = 0.0

        base_link_to_laser = TransformStamped()
        base_link_to_laser.header.frame_id = 'base_link'
        base_link_to_laser.header.stamp = self.get_clock().now().to_msg()
        base_link_to_laser.child_frame_id = 'laser'
        base_link_to_laser.transform.translation.x = 0.0
        base_link_to_laser.transform.translation.y = 0.0
        base_link_to_laser.transform.translation.z = 0.3
        base_link_to_laser.transform.rotation.w = 1.0
        base_link_to_laser.transform.rotation.x = 0.0
        base_link_to_laser.transform.rotation.y = 0.0
        base_link_to_laser.transform.rotation.z = 0.0          

        return(
            map_to_odom,
            base_footprint_to_base_link,
            base_link_to_bno055,
            base_link_to_laser,
        )

def main():
    rclpy.init()
    node = StaticFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
