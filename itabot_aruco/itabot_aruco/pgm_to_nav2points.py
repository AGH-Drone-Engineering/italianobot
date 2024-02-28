import numpy as np
from PIL import Image
import yaml
import os

MARGIN = 10


def load_map_data(yaml_file):
    with open(yaml_file, "r") as file:
        map_data = yaml.safe_load(file)
    return map_data


def calculate_goal_points(map_file, resolution, init_pose):
    img = Image.open(map_file)
    img_array = np.array(img)
    goal_points = []
    height, width = img_array.shape
    for y in range(1, height - 1):
        for x in range(1, width - 1):
            if np.all(img_array[y - 1 : y + 2, x - 1 : x + 2] == 205):
                # Sprawdź, czy poprzedni punkt celu jest co najmniej MARGIN kratki dalej
                if (
                    len(goal_points) == 0
                    or np.abs(goal_points[-1][0] - x * resolution - init_pose[0])
                    > MARGIN * resolution
                    or np.abs(goal_points[-1][1] - y * resolution - init_pose[1])
                    > MARGIN * resolution
                ):
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
print(len(goal_points))
