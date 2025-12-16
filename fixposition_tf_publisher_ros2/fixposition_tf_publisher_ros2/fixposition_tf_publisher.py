import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TransformStamped
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import NavSatFix
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_ros import TransformBroadcaster
from fixposition_driver_ros2.msg import ODOMENU, LLH
from nav_msgs.msg import Odometry
from robot_localization.srv import ToLL, FromLL
from rclpy.logging import LoggingSeverity
from rcl_interfaces.msg import SetParametersResult
import pyproj


def proj_lla_to_ecef(lat, lon, alt):
    """
    Convert geographic coordinates (latitude, longitude, altitude) 
    to Earth-Centered, Earth-Fixed (ECEF) coordinates.

    Args:
        lat (float): Latitude in degrees.
        lon (float): Longitude in degrees.
        alt (float): Altitude in meters.

    Returns:
        tuple: ECEF coordinates (x, y, z) in meters.
    """
    # Define the transformer from WGS84 geographic (lat, lon, alt) to ECEF
    transformer = pyproj.Transformer.from_crs(
        "EPSG:4326",  # Geographic CRS (WGS84)
        "EPSG:4978",  # ECEF CRS
        always_xy=True  # Ensure lon, lat order for consistency
    )

    # Perform the transformation
    x, y, z = transformer.transform(lon, lat, alt)
    return x, y, z
   

def proj_ecef_to_lla(x, y, z):
    """
    Convert Earth-Centered, Earth-Fixed (ECEF) coordinates to geographic (latitude, longitude, altitude).

    Args:
        x (float): X-coordinate in meters.
        y (float): Y-coordinate in meters.
        z (float): Z-coordinate in meters.

    Returns:
        tuple: Geographic coordinates (latitude, longitude, altitude) in degrees and meters.
    """
    # Define the transformer from ECEF to WGS84 geographic (lat, lon, alt)
    transformer = pyproj.Transformer.from_crs(
        "EPSG:4978",  # ECEF CRS
        "EPSG:4326",  # Geographic CRS (WGS84)
        always_xy=True  # Ensure order (x, y, z)
    )

    # Perform the transformation
    lon, lat, alt = transformer.transform(x, y, z, radians=False)
    return lat, lon, alt

class FixPositionTransformer(Node):

    def __init__(self):
        super().__init__('fixposition_tf_publisher')
        self.declare_parameter('log_level', 'INFO')  # Declare the log level parameter with a default value
        # Set the initial logging level
        initial_level = self.get_parameter('log_level').value.upper()
        self.set_log_level(initial_level)

        # Initialize a tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # Define a custom QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  
            durability=DurabilityPolicy.VOLATILE,      
            depth=10                                   
        )
        self.subscription = self.create_subscription(
            LLH,
            '/fixposition/fpa/llh',
            self.gps_callback,
            qos_profile)

        self.odom_sub = self.create_subscription(
            ODOMENU,
            '/fixposition/fpa/odomenu',
            self.odom_callback,
            qos_profile)

        self.odom_pub = self.create_publisher(Odometry, "odom", 1)
        self.gps_pub = self.create_publisher(NavSatFix, "gps/filtered", 1)
        self.tf_broadcaster = TransformBroadcaster(self)
        # Create "toLL" service
        self.to_ll_srv = self.create_service(
            ToLL,
            'toLL',
            self.to_ll_callback
        )
        self.get_logger().info('toLL service is ready.')

        # Create "fromLL" service
        self.from_ll_srv = self.create_service(
            FromLL,
            'fromLL',
            self.from_ll_callback
        )
        self.get_logger().info('fromLL service is ready.')
        self.add_on_set_parameters_callback(self.param_callback)

        self.timer = self.create_timer(0.02, self.republish_transforms)  # 50 Hz

    def republish_transforms(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "FP_ECEF",  # Target frame
                "FP_ENU0",  # Source frame
                rclpy.time.Time(),  # Latest available transform
                timeout=rclpy.duration.Duration(seconds=1.0)  # Timeout
            )

            transform.header.frame_id = "world"
            transform.child_frame_id = "map"
            transform.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(transform)
            self.get_logger().debug("broad casted from world to map")

            # Map to odom ,currently empty tranform
            transform_map_to_odom = TransformStamped()
            transform_map_to_odom.header.stamp = self.get_clock().now().to_msg()
            transform_map_to_odom.header.frame_id = 'map'  # New parent frame
            transform_map_to_odom.child_frame_id = 'odom'  # New child frame
            self.tf_broadcaster.sendTransform(transform_map_to_odom)
            self.get_logger().debug("broad casted from map to odom")

            # Now odom to base_link
            transform = self.tf_buffer.lookup_transform(
                "FP_ENU0",  # Target frame
                "FP_POI",  # Source frame
                rclpy.time.Time(),  # Latest available transform
                timeout=rclpy.duration.Duration(seconds=1.0)  # Timeout
            )
            transform.header.frame_id = "odom"
            transform.child_frame_id = "base_footprint"
            transform.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(transform)
            self.get_logger().debug("broad casted from odom to base_link")

        except Exception as e:
            self.get_logger().warn(f"Could not republish transform: {str(e)}")

    def set_log_level(self, level_str):
        """Sets the log level based on the given string."""
        level_str = level_str.upper()
        try:
            severity = getattr(LoggingSeverity, level_str)
            self.get_logger().set_level(severity)
            self.get_logger().info(f"Logging level set to: {level_str}")
        except AttributeError:
            self.get_logger().error(f"Invalid log level: {level_str}")

    def param_callback(self, params):
        """Callback to handle dynamic log level changes."""
        for param in params:
            if param.name == 'log_level':
                self.set_log_level(param.value)
        return SetParametersResult(successful=True)
    
    def to_ll_callback(self, request, response):
        """Callback for the toLL service."""
        self.get_logger().debug(
            f"Received toLL request: x={request.map_point.x}, y={request.map_point.y}, z={request.map_point.z}"
        )
        input_point = PointStamped()
        input_point.header.frame_id = "FP_ENU0"
        input_point.header.stamp = self.get_clock().now().to_msg()
        input_point.point.x, input_point.point.y, input_point.point.z = request.map_point.x, request.map_point.y, request.map_point.z
        try:   
            transform = self.tf_buffer.lookup_transform(
                    "FP_ECEF",  # Target frame
                    "FP_ENU0",  # Source frame
                    rclpy.time.Time(),  # Latest available transform
                    timeout=rclpy.duration.Duration(seconds=1.0)  # Timeout
                )
                 # Transform the point
            transformed_point = tf2_geometry_msgs.do_transform_point(input_point, transform)
        except Exception as e:
            self.get_logger().warn(f"Failed to transform point: {str(e)}")
            return response
        response.ll_point.latitude, response.ll_point.longitude, response.ll_point.altitude =  proj_ecef_to_lla(transformed_point.point.x, 
                                                                        transformed_point.point.y, transformed_point.point.z)
        self.get_logger().debug(
            f"Coords in GPS: lat={response.ll_point.latitude}, longitude={response.ll_point.longitude}, latitude={response.ll_point.altitude}"
        )
        return response

    def from_ll_callback(self, request, response):
        """Callback for the fromLL service."""

        self.get_logger().debug(
            f"Received fromLL request: latitude={request.ll_point.latitude}, "
            f"longitude={request.ll_point.longitude}, altitude={request.ll_point.altitude}"
        )

        input_point = PointStamped()
        input_point.header.frame_id = "FP_ECEF"
        input_point.header.stamp = self.get_clock().now().to_msg()
        input_point.point.x, input_point.point.y, input_point.point.z = proj_lla_to_ecef(request.ll_point.latitude, 
                                                                                request.ll_point.longitude, request.ll_point.altitude)
        try:   
            transform = self.tf_buffer.lookup_transform(
                    "FP_ENU0",  # Target frame
                    "FP_ECEF",  # Source frame
                    rclpy.time.Time(),  # Latest available transform
                    timeout=rclpy.duration.Duration(seconds=1.0)  # Timeout
                )
            # Transform the point
            transformed_point = tf2_geometry_msgs.do_transform_point(input_point, transform)
        except Exception as e:
                self.get_logger().warn(f"Failed to transform point: {str(e)}")
                return response

        # Log the transformed point
        self.get_logger().debug(
            f"ECEF Point: ({input_point.point.x}, {input_point.point.y}, {input_point.point.z}) -> "
            f"ENU Point: ({transformed_point.point.x}, {transformed_point.point.y}, {transformed_point.point.z})"
        )
        response.map_point.x = transformed_point.point.x
        response.map_point.y = transformed_point.point.y
        response.map_point.z = transformed_point.point.z
        return response

        
    def odom_callback(self, data):
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.child_frame_id = "base_footprint"
        odom.pose = data.pose
        odom.twist = data.velocity
        self.odom_pub.publish(odom)

    def gps_callback(self, data):
        gps_data = NavSatFix()
        gps_data.header.frame_id = "base_link"
        gps_data.header.stamp = self.get_clock().now().to_msg()
        gps_data.latitude = data.position.x
        gps_data.longitude =  data.position.y
        gps_data.altitude = data.position.z
        self.gps_pub.publish(gps_data)
        return 0
        
def main(args=None):
    rclpy.init(args=args)
    node = FixPositionTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down FixPositionTransformer node gracefully...")
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
    finally:
        # Safely destroy the node before shutting down the context
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
if __name__ == "__main__":
    main()
