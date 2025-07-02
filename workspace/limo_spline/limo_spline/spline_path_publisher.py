import rclpy
from rclpy.node import Node
from spline.spline_runner import SplineRunner
from geometry_msgs.msg import PoseStamped

class SplinePathPublisher(Node):
    def __init__(self):
        super().__init__("spline_path_publisher")

        self.declare_parameter(
            "map_file", 
            rclpy.Parameter.Type.STRING
        )

        self.declare_parameter(
            "technique", 
            rclpy.Parameter.Type.STRING
        )

        self.start_spline_runner()
    
    def start_spline_runner(self):
        map_file = self.get_parameter("map_file").get_parameter_value().string_value
        technique = self.get_parameter("technique").get_parameter_value().string_value

        self.spline_runner_obj = SplineRunner(map_file, technique)

    def publish_path(self):
        pass

def main():
    rclpy.init()
    node = SplinePathPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()