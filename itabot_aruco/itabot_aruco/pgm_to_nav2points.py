import numpy as np
from PIL import Image
import yaml
import os


def load_map_data(yaml_file):
    with open(yaml_file, "r") as file:
        map_data = yaml.safe_load(file)
    return map_data


def calculate_goal_points(map_file, resolution, init_pose):
    img = Image.open(map_file)
    img_array = np.array(img)
    available_points = np.where(img_array == 205)
    goal_points = []
    for y, x in zip(*available_points):
        real_x = x * resolution + init_pose[0]
        real_y = y * resolution + init_pose[1]
        goal_points.append((real_x, real_y))

    return goal_points


home_dir = os.environ["HOME"]
yaml_file = os.path.join(
    home_dir, "ros2_ws/src/italianobot/itabot_aruco/itabot_aruco/map/map.yaml"
)
map_file = os.path.join(
    home_dir, "ros2_ws/src/italianobot/itabot_aruco/itabot_aruco/map/map.pgm"
)
map_data = load_map_data(yaml_file)
resolution = map_data["resolution"]
init_pose = map_data["origin"][:2]
goal_points = calculate_goal_points(map_file, resolution, init_pose)
