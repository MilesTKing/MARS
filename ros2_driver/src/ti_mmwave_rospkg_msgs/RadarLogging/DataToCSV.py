#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
# import time # Removed previously
import csv
import os
import sys
import traceback

# Import the point cloud reader (with fallback)
try:
    import sensor_msgs_py.point_cloud2 as pc2
except ImportError:
    print("Could not import sensor_msgs_py.point_cloud2, trying legacy point_cloud2")
    try:
        import point_cloud2 as pc2
    except ImportError:
        print("Failed to import point_cloud2 library.")
        sys.exit(1)

POINTCLOUD_TOPIC = '/ti_mmwave/radar_scan_pcl'
# The data names used by point cloud library.
REQUIRED_FIELD_NAMES = ['x', 'y', 'z', 'intensity', 'velocity'] # Data field names from PointCloud2 library.
OUTPUT_CSV_FILENAME = 'logged_radar_data.csv'
CSV_WRITE_PATH = ('./'+OUTPUT_CSV_FILENAME)

class PointCloudLoggerToCsv(Node):
    def __init__(self):
        super().__init__('RadarDataToCsv_Silent') 

        self.get_logger().info(f"Expecting fields from PointCloud2: {REQUIRED_FIELD_NAMES}")
        try:
            self.idx_x = REQUIRED_FIELD_NAMES.index('x')
            self.idx_y = REQUIRED_FIELD_NAMES.index('y')
            self.idx_z = REQUIRED_FIELD_NAMES.index('z')
            self.idx_intensity = REQUIRED_FIELD_NAMES.index('intensity')
            self.idx_vel = REQUIRED_FIELD_NAMES.index('velocity') 
        except ValueError as e:
             raise SystemExit(f"Failed setup.")

        # -- Open CSV file and write header.
        self.csv_file = None
        self.csv_writer = None
        self.header = ['timestamp_sec', 'point_index', 'x', 'y', 'z', 'intensity', 'velocity'] #CSV header
        
        #Using CSV library to open file and write header.
        try:
            self.csv_file = open(CSV_WRITE_PATH, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(self.header)
            self.get_logger().info(f"Opened CSV file '{CSV_WRITE_PATH}' and wrote header: {self.header}")
        except IOError as e:
            self.get_logger().fatal(f"Could not open CSV file '{CSV_WRITE_PATH}' for writing: {e}")
            raise SystemExit(f"Failed to open CSV file: {e}")

        # -- Create ROS2 Subscription --
        self.subscription = self.create_subscription(
            PointCloud2,        
            POINTCLOUD_TOPIC,   #Subscribing to ros2 ti_mmwave/radar_scan_pcl topic
            self.listener_callback,10) 

        self.get_logger().info(f"Subscribing to {POINTCLOUD_TOPIC}")
        
    def listener_callback(self, msg: PointCloud2):
        timestamp_sec = msg.header.stamp.sec

        try:
             # Using point cloud library to read radar output
             point_generator = pc2.read_points(msg, field_names=REQUIRED_FIELD_NAMES, skip_nans=True)
        except ValueError as e:
             # Log error if fields don't match expectation
             self.get_logger().error(f"Error reading points!")

        point_index = 0
        for point in point_generator:
            try:
                # Access data directly using indices calculated in __init__
                x = point[self.idx_x]
                y = point[self.idx_y]
                z = point[self.idx_z]
                intensity = point[self.idx_intensity]
                velocity = point[self.idx_vel]

                row = [timestamp_sec, point_index, x, y, z, intensity, velocity] # Define format and write data to CSV
                self.csv_writer.writerow(row)
                
                point_index += 1
            except IndexError:
                 self.get_logger().error(f"Index error accessing point data for point {point_index}. Skipping point.")# Log error if point data structure is invalid
                 continue 

    def destroy_node(self):
        """Ensure file is closed when node is shutdown."""
        if hasattr(self, 'csv_file') and self.csv_file and not self.csv_file.closed:
            self.csv_file.close()
            self.get_logger().info(f"CSV file '{CSV_WRITE_PATH}' closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PointCloudLoggerToCsv()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
             node.get_logger().info('Keyboard interrupt, shutting down.')
    except SystemExit as e:
         if node:
              node.get_logger().info(f"Node startup failed or exited: {e}")
         else:
              # If node wasn't even created successfully
              print(f"Node startup failed or exited: {e}")
    except Exception as e:
         if node:
              node.get_logger().error(f"Unhandled exception: {e}")
         else:
              print(f"Unhandled exception before node initialized?: {e}")
         traceback.print_exc() # Print detailed traceback
    finally:
        # Ensure cleanup happens regardless of shutdown reason
        if node:
            node.destroy_node()
        if rclpy.ok(): # Check if context is still valid before shutdown
             rclpy.shutdown()

if __name__ == '__main__':
    main()