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

        # base_footprint_to_base_link = TransformStamped()
        # base_footprint_to_base_link.header.frame_id = 'base_footprint'
        # base_footprint_to_base_link.header.stamp = self.get_clock().now().to_msg()
        # base_footprint_to_base_link.child_frame_id = 'base_link'
        # base_footprint_to_base_link.transform.translation.x = 0.0
        # base_footprint_to_base_link.transform.translation.y = 0.0
        # base_footprint_to_base_link.transform.translation.z = 0.15 #0.125
        # base_footprint_to_base_link.transform.rotation.w = 1.0
        # base_footprint_to_base_link.transform.rotation.x = 0.0
        # base_footprint_to_base_link.transform.rotation.y = 0.0
        # base_footprint_to_base_link.transform.rotation.z = 0.0

        base_link_to_bno055 = TransformStamped()
        base_link_to_bno055.header.frame_id = 'base_link'
        base_link_to_bno055.header.stamp = self.get_clock().now().to_msg()
        base_link_to_bno055.child_frame_id = 'bno055'
        base_link_to_bno055.transform.translation.x = 0.0
        base_link_to_bno055.transform.translation.y = 0.12
        base_link_to_bno055.transform.translation.z = 0.175
        base_link_to_bno055.transform.rotation.w = 1.0
        base_link_to_bno055.transform.rotation.x = 0.0
        base_link_to_bno055.transform.rotation.y = 0.0
        base_link_to_bno055.transform.rotation.z = 0.0

        base_link_to_omron_front = TransformStamped()
        base_link_to_omron_front.header.frame_id = 'base_link'
        base_link_to_omron_front.header.stamp = self.get_clock().now().to_msg()
        base_link_to_omron_front.child_frame_id = 'omron_front'
        base_link_to_omron_front.transform.translation.x = 0.185
        base_link_to_omron_front.transform.translation.y = 0.0
        base_link_to_omron_front.transform.translation.z = 0.185
        base_link_to_omron_front.transform.rotation.w = 1.0
        base_link_to_omron_front.transform.rotation.x = 0.0
        base_link_to_omron_front.transform.rotation.y = 0.0
        base_link_to_omron_front.transform.rotation.z = 0.0

        base_link_to_omron_back = TransformStamped()
        base_link_to_omron_back.header.frame_id = 'base_link'
        base_link_to_omron_back.header.stamp = self.get_clock().now().to_msg()
        base_link_to_omron_back.child_frame_id = 'omron_back'
        base_link_to_omron_back.transform.translation.x = -0.185
        base_link_to_omron_back.transform.translation.y = 0.0
        base_link_to_omron_back.transform.translation.z = 0.185
        base_link_to_omron_back.transform.rotation.w = 0.0
        base_link_to_omron_back.transform.rotation.x = 0.0
        base_link_to_omron_back.transform.rotation.y = 0.0
        base_link_to_omron_back.transform.rotation.z = 1.0



        # base_link_to_rslidar = TransformStamped()
        # base_link_to_rslidar.header.frame_id = 'base_link'
        # base_link_to_rslidar.header.stamp = self.get_clock().now().to_msg()
        # base_link_to_rslidar.child_frame_id = 'rslidar' #rslidar
        # base_link_to_rslidar.transform.translation.x = 0.0
        # base_link_to_rslidar.transform.translation.y = 0.0
        # base_link_to_rslidar.transform.translation.z = 0.25
        # base_link_to_rslidar.transform.rotation.w = 1.0
        # base_link_to_rslidar.transform.rotation.x = 0.0
        # base_link_to_rslidar.transform.rotation.y = 0.0
        # base_link_to_rslidar.transform.rotation.z = 0.0     

        # base_link_to_omron = TransformStamped()
        # base_link_to_omron.header.frame_id = 'base_link'
        # base_link_to_omron.header.stamp = self.get_clock().now().to_msg()
        # base_link_to_omron.child_frame_id = 'omron'
        # base_link_to_omron.transform.translation.x = 0.3
        # base_link_to_omron.transform.translation.y = 0.0
        # base_link_to_omron.transform.translation.z = 0.215 #0.185
        # base_link_to_omron.transform.rotation.w = 0.976
        # base_link_to_omron.transform.rotation.x = 0.0
        # base_link_to_omron.transform.rotation.y = -0.22
        # base_link_to_omron.transform.rotation.z = 0.0 

        # base_link_to_laser = TransformStamped()
        # base_link_to_laser.header.frame_id = 'base_link'
        # base_link_to_laser.header.stamp = self.get_clock().now().to_msg()
        # base_link_to_laser.child_frame_id = 'laser'
        # base_link_to_laser.transform.translation.x = 0.05
        # base_link_to_laser.transform.translation.y = 0.0
        # base_link_to_laser.transform.translation.z = 0.35
        # base_link_to_laser.transform.rotation.w = 0.0
        # base_link_to_laser.transform.rotation.x = 1.0
        # base_link_to_laser.transform.rotation.y = 0.0
        # base_link_to_laser.transform.rotation.z = 0.0          

        return(
            map_to_odom,
            # base_footprint_to_base_link,
            base_link_to_bno055,
            base_link_to_omron_front,
            base_link_to_omron_back,
            # base_link_to_rslidar,
            # base_link_to_omron,
            # base_link_to_laser,
            
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
