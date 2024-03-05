import numpy as np
from PIL import Image
import yaml
import os


def load_map_data(yaml_file):
    with open(yaml_file, "r") as file:
        map_data = yaml.safe_load(file)
    return map_data


def calculate_goal_points(map_file, resolution, init_pose, MARGIN, WALL_DET):
    img = Image.open(map_file)
    img_array = np.array(img)

    # matrix to prepare visualization of the goal points
    out_put_img_array = img_array.copy()

    goal_points = []
    height, width = img_array.shape
    for y in range(WALL_DET, height - 2):
        for x in range(WALL_DET, width - 2):
            if np.all(
                img_array[
                    y - WALL_DET : y + WALL_DET + 1, x - WALL_DET : x + WALL_DET + 1
                ]
                == 254
            ):
                # free space between points:
                if len(goal_points) == 0 or all(
                    np.abs(gp["px"] - x * resolution) > MARGIN * resolution
                    or np.abs(gp["py"] - y * resolution) > MARGIN * resolution
                    for gp in goal_points[-4 * width // MARGIN :]
                ):

                    real_x = x * resolution
                    real_y = y * resolution
                    rotation = [
                        {"ow": 1.0, "ox": 0.0, "oy": 0.0, "oz": 0.0},
                        {"ow": 0.7, "ox": 0.0, "oy": 0.0, "oz": -0.7},
                        {"ow": 0.0, "ox": 0.0, "oy": 0.0, "oz": 1.0},
                        {"ow": 0.7, "ox": 0.0, "oy": 0.0, "oz": 0.7},
                    ]

                    for i in range(len(rotation)):
                        goal_points.append(
                            {
                                "px": real_x,
                                "py": real_y,
                                "pz": 0.0,
                                "ow": rotation[i]["ow"],
                                "ox": rotation[i]["ox"],
                                "oy": rotation[i]["oy"],
                                "oz": rotation[i]["oz"],
                            }
                        )
                    # print(real_x, real_y)
                    # Saving map with goal points
                    # Only for testing the algorithm
                    out_put_img_array[y][x] = 150
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
MARGIN = 20  # minimal spacing between points
WALL_DET = 5  # minimal spacing between rosbot and wall
goal_points = calculate_goal_points(map_file, resolution, init_pose, MARGIN, WALL_DET)
