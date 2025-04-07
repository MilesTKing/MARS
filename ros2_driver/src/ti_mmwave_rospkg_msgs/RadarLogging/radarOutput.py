#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
# import time # Removed - No longer needed without summary logic

# Import the point cloud reader. Adapt the import based on your ROS 2 version.
try:
    # ROS 2 Humble+ recommended import
    import sensor_msgs_py.point_cloud2 as pc2
except ImportError:
    # Fallback for older distros or if the above fails
    print("Could not import sensor_msgs_py.point_cloud2, trying legacy point_cloud2")
    try:
        import point_cloud2 as pc2
    except ImportError:
        print("Failed to import point_cloud2 library.")
        print("Please ensure ROS 2 Python point cloud utilities are installed.")
        print("Try: sudo apt install ros-$ROS_DISTRO-sensor-msgs-py")
        exit(1)

POINTCLOUD_TOPIC = '/ti_mmwave/radar_scan_pcl' #ros2 topic name

FIELD_NAMES = ['x', 'y', 'z', 'intensity', 'velocity'] #ros2 topic names for radar output values

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('Radar_Output') # Renamed node slightly
        self.subscription = self.create_subscription(
            PointCloud2,
            POINTCLOUD_TOPIC,
            self.listener_callback,
            10) # QoS profile depth
        self.get_logger().info(f"Subscribing to {POINTCLOUD_TOPIC}")

        #Retrieve data fields
        try:
            self.idx_x = FIELD_NAMES.index('x')
            self.idx_y = FIELD_NAMES.index('y')
            self.idx_z = FIELD_NAMES.index('z')
            self.idx_intensity = FIELD_NAMES.index('intensity')
            self.idx_velocity = FIELD_NAMES.index('velocity')
        except ValueError as e:
             self.get_logger().fatal(f"Programming error: Field name mismatch during index setup: {e}.")
             rclpy.shutdown() # Shutdown if setup fails
             raise SystemExit(f"Failed to find assumed field during setup: {e}")

        # --- Removed summary timing attributes ---
        # self.last_print_time = 0
        # self.print_interval = 2.0 # Print summary every 2 seconds

    def listener_callback(self, msg: PointCloud2):
        # Use the read_points generator with the ASSUMED field names.
        # If any assumed field is missing in the actual message, this will raise a ValueError.
        try:
             point_generator = pc2.read_points(msg, field_names=FIELD_NAMES, skip_nans=True)
        except ValueError as e:
            # Handle case where read_points fails because a field is actually missing
            self.get_logger().error(f"Error reading points!")

        point_count = 0

        for point in point_generator:
            # point is a structure/tuple containing the requested fields in the order specified
            # Access data using pre-calculated indices for clarity and safety
            x = point[self.idx_x]
            y = point[self.idx_y]
            z = point[self.idx_z]
            intensity = point[self.idx_intensity]
            velocity = point[self.idx_velocity]

            #Print the first 5 points of each message
            if point_count < 5:
                self.get_logger().info(
                    f"Point {point_count}: X={x:.3f}, Y={y:.3f}, Z={z:.3f}, "
                    f"Intensity={intensity:.1f}, Velocity={velocity:.3f}"
                )
            point_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()