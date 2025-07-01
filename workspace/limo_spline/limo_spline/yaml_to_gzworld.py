#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory
import argparse
import yaml
import os

def parse_cli_args():
    parser = argparse.ArgumentParser(description="to construct a curve connecting all the waypoints, while ensuring obstacle avoidance")
    parser.add_argument("--map_path", type=str, help="path to the YAML map file", required=True)
    parser.add_argument("--world_path", type=str, help="path to save the output Gazebo world file to", required=True)
    args = parser.parse_args()
    
    return args

class WorldConverter():

    SCALE_FACTOR = 0.1

    OBSTACLE_HEIGHT = 1
    OBSTACLE_COLOR = "brown"

    BOUNDARY_THICKNESS = 0.5
    BOUNDARY_HEIGHT = 0.5
    BOUNDARY_COLOR = "white"

    package_dir = get_package_share_directory('limo_spline')
    TEMPLATE_FILES_DIR = os.path.join(package_dir,'template_files')
    
    def __init__(self, map_path, world_path):
        with open(map_path, 'r') as map:
            self.map_file = yaml.safe_load(map)
        
        self.world_file = open(world_path, 'w')
        self.world_name = os.path.splitext(os.path.basename(world_path))[0]
        self.convert_to_gz_world()
        
        self.world_file.close()
    
    def convert_to_gz_world(self):
        self.write_header()
        
        for obstacle in self.map_file.get("obstacles"):
            obstacle_id = obstacle.get("id")
            obstacle_name = f"obstacle{obstacle_id}"

            x = self.SCALE_FACTOR * obstacle.get("location")[0]
            y = self.SCALE_FACTOR * obstacle.get("location")[1]
            obstacle_length_x = self.SCALE_FACTOR * obstacle.get("width")
            obstacle_length_y = self.SCALE_FACTOR * obstacle.get("height")

            obstacle_x = x + obstacle_length_x/2
            obstacle_y = y + obstacle_length_y/2

            self.write_obstacle(obstacle_name, obstacle_x, obstacle_y, obstacle_length_x, obstacle_length_y, self.OBSTACLE_HEIGHT, self.OBSTACLE_COLOR)
        
        world_area = self.map_file.get("area")
        x_range = world_area.get("x_range")
        y_range = world_area.get("y_range")
        self.write_boundary(x_range, y_range)

        self.write_footer()

    def write_header(self):
        with open(f'{self.TEMPLATE_FILES_DIR}/world_header.sdf', 'r') as h:
            header_template = h.read()
        header = header_template.format(world_name=self.world_name)
        self.world_file.write(header)

    def write_obstacle(self, obstacle_name, x, y, length_x, length_y, height, color):
        with open(f'{self.TEMPLATE_FILES_DIR}/world_obstacle.sdf', 'r') as o:
            obstacle_template = o.read()
        
        with open(f'{self.TEMPLATE_FILES_DIR}/{color}.txt', 'r') as c:
            color_txt = c.read()

        obstacle = obstacle_template.format(obstacle_name = obstacle_name,
                                            x = x,
                                            y = y,
                                            z = height/2,
                                            length_x = length_x,
                                            length_y = length_y,
                                            obstacle_height = height,
                                            color = color_txt)
        self.world_file.write(obstacle)

    def write_footer(self):
        with open(f'{self.TEMPLATE_FILES_DIR}/world_footer.sdf', 'r') as f:
            footer = f.read()
        self.world_file.write(footer)

    def write_boundary(self, x_range, y_range):

        horiz_boundary_x = x_range/2
        horiz_boundary_length = x_range + (2 * self.BOUNDARY_THICKNESS)
        horiz_boundary_width = self.BOUNDARY_THICKNESS
        horiz_boundary_y1 = -self.BOUNDARY_THICKNESS/2
        horiz_boundary_y2 = y_range + self.BOUNDARY_THICKNESS/2

        self.write_obstacle("bottom_wall", horiz_boundary_x, horiz_boundary_y1, horiz_boundary_length, horiz_boundary_width, self.BOUNDARY_HEIGHT, self.BOUNDARY_COLOR)
        self.write_obstacle("top_wall", horiz_boundary_x, horiz_boundary_y2, horiz_boundary_length, horiz_boundary_width, self.BOUNDARY_HEIGHT, self.BOUNDARY_COLOR)

        vert_boundary_y = y_range/2
        vert_boundary_length = self.BOUNDARY_THICKNESS
        vert_boundary_width = y_range + (2 * self.BOUNDARY_THICKNESS)
        vert_boundary_x1 = -self.BOUNDARY_THICKNESS/2
        vert_boundary_x2 = x_range + self.BOUNDARY_THICKNESS/2

        self.write_obstacle("left_wall", vert_boundary_x1, vert_boundary_y, vert_boundary_length, vert_boundary_width, self.BOUNDARY_HEIGHT, self.BOUNDARY_COLOR)
        self.write_obstacle("right_wall", vert_boundary_x2, vert_boundary_y, vert_boundary_length, vert_boundary_width, self.BOUNDARY_HEIGHT, self.BOUNDARY_COLOR)

def main():
    args = parse_cli_args()
    WorldConverter(args.map_path, args.world_path)

if __name__ == '__main__':
    main()