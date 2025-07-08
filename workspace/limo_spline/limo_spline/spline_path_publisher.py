import rclpy
from rclpy.node import Node
from spline.spline_runner import SplineRunner
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
import math

class SplinePathPublisher(Node):
    def __init__(self):
        self.ROBOT_ELEVATION = 0.3
        super().__init__("spline_path_publisher")

        self.declare_parameter("map_file", "")
        self.declare_parameter("technique", "")

        self.path = self.get_path()

        self.spline_path_publisher = self.create_publisher(Path, "/path", 10)

        self.timer = self.create_timer(0.5, self.publish_path)

    def get_path(self):
        map_file = self.get_parameter("map_file").get_parameter_value().string_value
        technique = self.get_parameter("technique").get_parameter_value().string_value

        self.get_logger().info("Started Spline Path Publisher!")

        spline_runner_obj = SplineRunner(map_file, technique, visualization=False)

        return spline_runner_obj.map_path.curves
    
    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = "world"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg_list = []

        for segment in self.path:
            segment_pose_msg_list = self.get_segment_pose_list(segment)
            pose_msg_list.extend(segment_pose_msg_list)
        
        path_msg.poses = pose_msg_list
        self.spline_path_publisher.publish(path_msg)

    def get_segment_pose_list(self, segment):
        segment_pose_msg_list = []
        for i in range(len(segment)):
            pose_msg = PoseStamped()
            x_current, y_current = segment[i]

            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "world"

            pose_msg.pose.position.x = float(x_current)
            pose_msg.pose.position.y = float(y_current)
            pose_msg.pose.position.z = self.ROBOT_ELEVATION

            if (i == len(segment) - 1):
                yaw = prev_yaw
            else:
                x_next, y_next = segment[i + 1]
                yaw = math.atan2(y_next - y_current, x_next - x_current)

            pose_msg.pose.orientation = self.compute_orientation(yaw)
        
            segment_pose_msg_list.append(pose_msg)
            prev_yaw = yaw
        
        return segment_pose_msg_list
    
    def compute_orientation(self, yaw):
        orientation = Quaternion()
        orientation.x = 0.0
        orientation.y = 0.0
        orientation.z = math.sin(yaw/2.0)
        orientation.w = math.cos(yaw/2.0)
        return orientation


def main():
    rclpy.init()
    node = SplinePathPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()