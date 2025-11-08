#####################################
# Imports
#####################################
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Float32, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Pose, PoseWithCovariance, Twist, TwistWithCovariance
from rover2_control_interface.msg import DriveCommandMessage
import math

#####################################
# Global Variables
#####################################
NODE_NAME = "odometry_node"

DEFAULT_GPS_TOPIC = "autonomous/simple_position"
DEFAULT_IMU_HEADING_TOPIC = "imu/data/heading"
DEFAULT_DRIVE_TOPIC = "command_control/iris_drive"
DEFAULT_ODOM_TOPIC = "rover_odom"


DEFAULT_HERTZ = 10

#####################################
# Odometry Node
#####################################
class OdometryNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        # Parameters
        self.gps_topic = self.declare_parameter('gps_topic', DEFAULT_GPS_TOPIC).value
        self.imu_heading_topic = self.declare_parameter('imu_heading_topic', DEFAULT_IMU_HEADING_TOPIC).value
        self.odom_topic = self.declare_parameter('odom_topic', DEFAULT_ODOM_TOPIC).value
        self.drive_topic = self.declare_parameter('drive_topic', DEFAULT_DRIVE_TOPIC).value
        self.hz = self.declare_parameter('rate', DEFAULT_HERTZ).value

        # State
        self.latest_lat = None
        self.latest_lon = None
        self.latest_heading = None
        self.latest_twist = None
        self.origin_set = False
        self.origin_lat = None
        self.origin_lon = None

        self.create_subscription(String, self.gps_topic, self.gps_callback, 10)
        self.create_subscription(Float32, self.imu_heading_topic, self.imu_callback, 10)
        self.create_subscription(DriveCommandMessage, self.drive_topic, self.drive_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        self.timer = self.create_timer(1.0 / self.hz, self.publish_odometry)
        self.get_logger().info(f"{NODE_NAME} started. Subscribed to {self.gps_topic} and {self.imu_heading_topic}")

    def gps_callback(self, msg):
        coordStr = msg.data.split(";")
        self.latest_lat = float(coordStr[0])
        self.latest_lon = float(coordStr[1])
        if not self.origin_set:
            self.origin_lat = float(coordStr[0])
            self.origin_lon = float(coordStr[1])
            self.origin_set = True
    def imu_callback(self, msg: Float32):
        self.latest_heading = msg.data

    def drive_callback(self, msg: DriveCommandMessage):
        self.latest_twist = msg.drive_twist

    def latlon_to_local_xy(self, lat, lon):
        R = 6378137.0  # Earth radius (m)
        d_lat = math.radians(lat - self.origin_lat)
        d_lon = math.radians(lon - self.origin_lon)
        x = d_lon * R * math.cos(math.radians((lat + self.origin_lat) / 2.0))
        y = d_lat * R
        return x, y

    def publish_odometry(self):
        if None in [self.latest_lat, self.latest_lon, self.latest_heading] or not self.origin_set:
            return

        x, y = self.latlon_to_local_xy(self.latest_lat, self.latest_lon)
        yaw = math.radians(self.latest_heading)

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose = Pose(
            position=Point(x=x, y=y, z=0.0),
            orientation=orientation
        )

        odom.twist.twist = self.latest_twist

        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

