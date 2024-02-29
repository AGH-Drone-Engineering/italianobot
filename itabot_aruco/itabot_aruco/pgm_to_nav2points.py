import numpy as np
from PIL import Image
import yaml
import os

MARGIN = 10  # minimal spacing between points
WALL_DET = 5  # minimal spacing between rosbot and wall


def load_map_data(yaml_file):
    with open(yaml_file, "r") as file:
        map_data = yaml.safe_load(file)
    return map_data


def calculate_goal_points(map_file, resolution, init_pose):
    img = Image.open(map_file)
    img_array = np.array(img)
    out_put_img_array = (
        img_array.copy()
    )  # matrix to prepare visualization of the goal points
    goal_points = []
    height, width = img_array.shape
    for y in range(1, height - 1):
        for x in range(1, width - 1):
            if np.all(
                img_array[y - WALL_DET : y + WALL_DET, x - WALL_DET : x + WALL_DET]
                == 254
            ):
                # free space between points:
                if len(goal_points) == 0 or all(
                    np.abs(gp[0] - x * resolution - init_pose[0]) > MARGIN * resolution
                    or np.abs(gp[1] - y * resolution - init_pose[1])
                    > MARGIN * resolution
                    for gp in goal_points[-width // MARGIN :]
                ):

                    real_x = x * resolution + init_pose[0]
                    real_y = y * resolution + init_pose[1]
                    goal_points.append((real_x, real_y))

                    # Saving map with goal points
                    # Only for testing the algorithm
                    out_put_img_array[y][x] = 50
                    im = Image.fromarray(out_put_img_array)
                    directory = os.path.dirname(map_file)
                    points_file = os.path.join(directory, "points.pgm")
                    im.save(points_file)

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
