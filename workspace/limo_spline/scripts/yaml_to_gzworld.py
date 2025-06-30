import argparse
import yaml
import os

def parse_cli_args():
    parser = argparse.ArgumentParser(description="to construct a curve connecting all the waypoints, while ensuring obstacle avoidance")
    parser.add_argument("--map_path", type=str, help="path to the YAML map file", required=True)
    parser.add_argument("--out_path", type=str, help="path to save the output Gazebo world file to", required=True)
    args = parser.parse_args()
    
    return args

def write_header(map_filename, world_file):
    with open('template_files/world_header.sdf', 'r') as h:
        header_template = h.read()
    header = header_template.format(world_name=map_filename)
    world_file.write(header)

def write_obstacles(obstacle_id, x, y, length_x, length_y, height, color, world_file):
    with open('template_files/world_obstacle.sdf', 'r') as o:
        obstacle_template = o.read()
    
    with open(f'template_files/{color}.txt', 'r') as c:
        color_txt = c.read()

    obstacle = obstacle_template.format(obstacle_id = obstacle_id,
                                        x = x,
                                        y = y,
                                        z = height/2,
                                        length_x = length_x,
                                        length_y = length_y,
                                        obstacle_height = height,
                                        color = color_txt)
    world_file.write(obstacle)

def write_footer(world_file):
    with open('template_files/world_footer.sdf', 'r') as f:
        footer = f.read()
    world_file.write(footer)

def write_boundary(x_range, y_range, world_file):
    BOUNDARY_THICKNESS = 0.5
    BOUNDARY_HEIGHT = 0.5
    BOUNDARY_COLOR = "white"

    horiz_boundary_x = x_range/2
    horiz_boundary_length = x_range + (2 * BOUNDARY_THICKNESS)
    horiz_boundary_width = BOUNDARY_THICKNESS
    horiz_boundary_y1 = -BOUNDARY_THICKNESS/2
    horiz_boundary_y2 = y_range + BOUNDARY_THICKNESS/2

    write_obstacles("bottom_wall", horiz_boundary_x, horiz_boundary_y1, horiz_boundary_length, horiz_boundary_width, BOUNDARY_HEIGHT, BOUNDARY_COLOR, world_file)
    write_obstacles("top_wall", horiz_boundary_x, horiz_boundary_y2, horiz_boundary_length, horiz_boundary_width, BOUNDARY_HEIGHT, BOUNDARY_COLOR, world_file)

    vert_boundary_y = y_range/2
    vert_boundary_length = BOUNDARY_THICKNESS
    vert_boundary_width = y_range + (2 * BOUNDARY_THICKNESS)
    vert_boundary_x1 = -BOUNDARY_THICKNESS/2
    vert_boundary_x2 = x_range + BOUNDARY_THICKNESS/2

    write_obstacles("left_wall", vert_boundary_x1, vert_boundary_y, vert_boundary_length, vert_boundary_width, BOUNDARY_HEIGHT, BOUNDARY_COLOR, world_file)
    write_obstacles("right_wall", vert_boundary_x2, vert_boundary_y, vert_boundary_length, vert_boundary_width, BOUNDARY_HEIGHT, BOUNDARY_COLOR, world_file)
    

def main():
    args = parse_cli_args()
    map_path = args.map_path
    out_path = args.out_path
    
    with open(map_path, 'r') as map:
        map_file = yaml.safe_load(map)

    with open(out_path, 'w') as world:
        world_name = os.path.splitext(os.path.basename(out_path))[0]
        world_area = map_file.get("area")
        write_header(world_name, world)

        OBSTACLE_HEIGHT = 1
        OBSTACLE_COLOR = "brown"
        for obstacle in map_file.get("obstacles"):
            obstacle_name = "obstacle" + str(obstacle.get("id"))
            x = 0.1 * obstacle.get("location")[0]
            y = 0.1 * obstacle.get("location")[1]
            obstacle_length_x = 0.1 * obstacle.get("width")
            obstacle_length_y = 0.1 * obstacle.get("height")

            obstacle_x = x + obstacle_length_x/2
            obstacle_y = y + obstacle_length_y/2

            write_obstacles(obstacle_name, obstacle_x, obstacle_y, obstacle_length_x, obstacle_length_y, OBSTACLE_HEIGHT, OBSTACLE_COLOR, world)
        
        x_range = 0.1 * world_area.get("x_range")
        y_range = 0.1 * world_area.get("y_range")
        
        write_boundary(x_range, y_range, world)

        write_footer(world)

if __name__ == '__main__':
    main()