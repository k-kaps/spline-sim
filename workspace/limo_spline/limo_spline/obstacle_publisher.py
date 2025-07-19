import rclpy
from rclpy.node import Node
from spline.spline_runner import SplineRunner
from visualization_msgs.msg import Marker, MarkerArray

class ObstaclePublisher(Node):
    def __init__(self):
        super().__init__("obstacle_publisher")

        self.declare_parameter("map_file", "")
        self.declare_parameter("technique", "")

        self.map_obj = self.get_obj()

        self.obstacle_publisher = self.create_publisher(MarkerArray, "/obstacles", 10)
        
        self.create_timer(1.0, self.publish_obstacles)
    
    def get_obj(self):
        map_file = self.get_parameter("map_file").get_parameter_value().string_value
        technique = self.get_parameter("technique").get_parameter_value().string_value

        spline_runner_obj = SplineRunner(map_file, technique, visualization=False)

        return spline_runner_obj.map_obj
    
    def publish_obstacles(self):
        marker_array = MarkerArray()
        for obstacle_dict in self.map_obj.map_obstacles:
            x = float(obstacle_dict["b"][0])
            y = float(obstacle_dict["b"][2])
            obstacle_length_x = float(obstacle_dict["b"][1])
            obstacle_length_y = float(obstacle_dict["b"][3])
            obstacle_x = x + obstacle_length_x/2
            obstacle_y = y + obstacle_length_y/2
        
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "obstacles"
            marker.id = obstacle_dict['id']
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = obstacle_x
            marker.pose.position.y = obstacle_y
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 0.0
            marker.scale.x = obstacle_length_x
            marker.scale.y = obstacle_length_y
            marker.scale.z = 1.0
            marker.color.r = 0.8
            marker.color.g = 0.1
            marker.color.b = 0.1
            marker.color.a = 0.7
            marker_array.markers.append(marker)

        self.obstacle_publisher.publish(marker_array)

def main():
    rclpy.init()
    node = ObstaclePublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()