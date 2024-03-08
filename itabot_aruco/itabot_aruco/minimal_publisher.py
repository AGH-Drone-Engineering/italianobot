# ros2 run nav2_map_server map_saver_cli -f my_map
# Do wywołania po 5 minutach po explorer

# potem po (20s?) zapisywania
# minimal publisher

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import sys
import os
import re
import time
import json

home_dir = os.environ["HOME"]
pgm_file = os.path.join(home_dir, "ros2_ws/src/italianobot/itabot_aruco/itabot_aruco")
sys.path.insert(0, pgm_file)
import pgm_to_nav2points as nav2p


class GoalPublisher(Node):

    def __init__(self):

        # 15 minutes timer begin:
        self.start = time.time()

        # list of codes to find:
        self.arucos_to_find = ["55"]

        # stop exploration:
        try:
            os.system("pkill -f explore_node")
        except Exception as e:
            self.get_logger().info(f"{e}")

        # load map to create the goal points:
        home_dir = os.environ["HOME"]
        yaml_file = os.path.join(
            home_dir, "ros2_ws/src/italianobot/itabot_aruco/itabot_aruco/map/map.yaml"
        )
        map_file = os.path.join(
            home_dir, "ros2_ws/src/italianobot/itabot_aruco/itabot_aruco/map/map.pgm"
        )
        map_data = nav2p.load_map_data(yaml_file)
        resolution = map_data["resolution"]
        self.init_pose = map_data["origin"][:2]

        # set the parameters of algorithm (how far from wall the goal point can be placed and how far from another goal point):
        MARGIN = 37  # minimal spacing between points
        WALL_DET = 5  # minimal spacing between rosbot and wall

        # get list of goal points:
        self.points = nav2p.calculate_goal_points(
            map_file, resolution, self.init_pose, MARGIN, WALL_DET
        )

        # points publisher:
        super().__init__("goal_publisher")
        self.publisher_ = self.create_publisher(PoseStamped, "goal_pose", 10)
        timer_period = 0.5  # sekundy
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # pose with covariance subscriber (unnecessary but just for safety if tf fails)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, "pose", self.position_sub, 10
        )
        self.subscription  # prevent unused variable warning
        self.actual_position_pose = {
            "px": 0,
            "py": 0,
            "pz": 0,
            "ow": 0,
            "ox": 0,
            "oy": 0,
            "oz": 0,
        }

        # dict to find if rosbot has reached position
        self.actual_position_tf_base_link = dict()

        # actual goal point counter (= len(self.points) means that he will go to the base (0,0,0) because of IndexError)
        self.i = 0

        # parameter changing when real aruco positions saved
        self.raports_dir = os.path.join(
            home_dir, "ros2_ws/src/italianobot/itabot_aruco/itabot_aruco/raports"
        )
        # TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def different(self, a: float, b: float, margin: float) -> bool:
        a = float(a)
        b = float(b)
        return True if abs(a - b) <= margin else False

    def get_found_arucos(self):
        info_tf = self.tf_buffer.all_frames_as_string()
        pattern = r"\baruco\w*"
        info_tf_aruco = re.findall(pattern, info_tf)
        return [aruco[13:] for aruco in info_tf_aruco]

    def position_sub(self, msg):

        self.actual_position_pose["px"] = msg.pose.pose.position.x
        self.actual_position_pose["py"] = msg.pose.pose.position.y
        self.actual_position_pose["pz"] = msg.pose.pose.position.z
        self.actual_position_pose["ox"] = msg.pose.pose.orientation.x
        self.actual_position_pose["oy"] = msg.pose.pose.orientation.y
        self.actual_position_pose["oz"] = msg.pose.pose.orientation.z
        self.actual_position_pose["ow"] = msg.pose.pose.orientation.w

    def return_time(self):
        stop = time.time()
        return stop - self.start  # it return time is seconds

    def timer_callback(self):

        # Go to the base if time passed:
        # assumption: Exploration time equals 6 minutes
        # 9 minutes in seconds (minus 10 sec cuz of saving delay) for minimal publisher to find aruco codes
        EXPLORE_MINUTES = 6  # if this parameter is changed, please change it also in itabot_nav/bringup.launch.py
        SAVING_TIME_SECONDS = 10  # if this parameter is changed, please change it also in itabot_nav/bringup.launch.py
        time_passed = (15 - EXPLORE_MINUTES) * 60 - SAVING_TIME_SECONDS
        if self.return_time() >= time_passed:
            self.i = len(self.points)
            for i in range(10):
                self.get_logger().info(f"TIME PASSED")
        else:
            # Go to the base if all codes have been found:
            try:
                arucos_found = self.get_found_arucos()
                for aruco in arucos_found:
                    if aruco in self.arucos_to_find:
                        self.get_logger().info(f"Aruco found: {aruco}\n" * 10)
                        self.arucos_to_find.remove(aruco)
            except Exception as e:
                self.get_logger().info(f"Aruco_to_find remover error {e}\n" * 10)

            if len(self.arucos_to_find) == 0:
                self.i = len(self.points)

            # IF TIME DIDN'T EXPIRE AND ARUCO CODES NOT FOUND:
            elif self.i >= len(self.points):
                for i in range(10):
                    self.get_logger().info("Time didn't pass, aruco codes not found")
                self.i = 0

        # Main logic of goal points setter:
        try:
            msg = PoseStamped()
            msg.header.stamp.sec = 0
            msg.header.frame_id = "map"
            msg.pose.position.x = self.points[self.i]["px"]
            msg.pose.position.y = self.points[self.i]["py"]
            msg.pose.position.z = self.points[self.i]["pz"]
            msg.pose.orientation.x = self.points[self.i]["ox"]
            msg.pose.orientation.y = self.points[self.i]["oy"]
            msg.pose.orientation.z = self.points[self.i]["oz"]
            msg.pose.orientation.w = self.points[self.i]["ow"]

            self.publisher_.publish(msg)
            self.get_logger().info(
                f"\nGoal:\nnr:{self.i+1}/{len(self.points)}\nx: {msg.pose.position.x}\ny: {msg.pose.position.y}\nz: {msg.pose.position.z}\nw: {msg.pose.orientation.w}\n"
            )

        except IndexError:
            self.get_logger().info("All goals reached")
            msg = PoseStamped()
            msg.header.stamp.sec = 0
            msg.header.frame_id = "map"
            msg.pose.position.x = 0.0
            msg.pose.position.y = 0.0
            msg.pose.position.z = 0.0
            msg.pose.orientation.x = self.points[0]["ox"]
            msg.pose.orientation.y = self.points[0]["oy"]
            msg.pose.orientation.z = self.points[0]["oz"]
            msg.pose.orientation.w = self.points[0]["ow"]

            self.publisher_.publish(msg)
            self.get_logger().info("Returning to the base")
            try:
                self.get_logger().info("Trying to convert aruco axes")
                all_data = list()
                for filename in os.listdir(self.raports_dir):
                    if filename.endswith("json"):
                        json_path = os.path.join(self.raports_dir, filename)

                        with open(json_path, "r") as f:
                            data = json.load(f)
                            if "mean" in data["child_frame_id"]:
                                data["transform"]["translation"]["x"] -= self.init_pose[
                                    0
                                ]
                                data["transform"]["translation"]["y"] -= self.init_pose[
                                    1
                                ]
                                all_data.append(data)

                codes_real_position_json = os.path.join(
                    self.raports_dir, "codes_real_position.json"
                )
                if not os.path.exists(codes_real_position_json):
                    with open(codes_real_position_json, "w") as f:
                        json.dump(all_data, f)

                self.get_logger().info("JSON SAVED AS codes_real_position.json\n" * 5)
            except:
                self.get_logger().info("Fail to save real aruco pose")
        except Exception as e:
            self.get_logger().info(f"{e}")

        # read transform
        try:
            trans = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
            self.actual_position_tf_base_link["px"] = trans.transform.translation.x
            self.actual_position_tf_base_link["py"] = trans.transform.translation.y
            self.actual_position_tf_base_link["pz"] = trans.transform.translation.z
            self.actual_position_tf_base_link["ox"] = trans.transform.rotation.x
            self.actual_position_tf_base_link["oy"] = trans.transform.rotation.y
            self.actual_position_tf_base_link["oz"] = trans.transform.rotation.z
            self.actual_position_tf_base_link["ow"] = trans.transform.rotation.w
            self.get_logger().info(
                f"\nCURRENT BASE_LINK POSITION: \n{self.actual_position_tf_base_link}\n"
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info("Could not transform map to base_link: %s" % str(e))

        # changing i to i+ if goal reached
        margin = 0.3
        try:
            if (
                self.different(
                    self.actual_position_tf_base_link["px"],
                    self.points[self.i]["px"],
                    margin,
                )
                and self.different(
                    self.actual_position_tf_base_link["py"],
                    self.points[self.i]["py"],
                    margin,
                )
                and self.different(
                    self.actual_position_tf_base_link["pz"],
                    self.points[self.i]["pz"],
                    margin,
                )
                and self.different(
                    self.actual_position_tf_base_link["ow"],
                    self.points[self.i]["ow"],
                    margin,
                )
                and self.different(
                    self.actual_position_tf_base_link["ox"],
                    self.points[self.i]["ox"],
                    margin,
                )
                and self.different(
                    self.actual_position_tf_base_link["oy"],
                    self.points[self.i]["oy"],
                    margin,
                )
                and self.different(
                    self.actual_position_tf_base_link["oz"],
                    self.points[self.i]["oz"],
                    margin,
                )
            ):
                self.i += 1
        except:
            # if tf goes down posewithcovariencestamped of robot will take a lead
            try:
                if (
                    self.different(
                        self.actual_position_pose["px"],
                        self.points[self.i]["px"],
                        margin,
                    )
                    and self.different(
                        self.actual_position_pose["py"],
                        self.points[self.i]["py"],
                        margin,
                    )
                    and self.different(
                        self.actual_position_pose["pz"],
                        self.points[self.i]["pz"],
                        margin,
                    )
                    and self.different(
                        self.actual_position_pose["ow"],
                        self.points[self.i]["ow"],
                        margin,
                    )
                    and self.different(
                        self.actual_position_pose["ox"],
                        self.points[self.i]["ox"],
                        margin,
                    )
                    and self.different(
                        self.actual_position_pose["oy"],
                        self.points[self.i]["oy"],
                        margin,
                    )
                    and self.different(
                        self.actual_position_pose["oz"],
                        self.points[self.i]["oz"],
                        margin,
                    )
                ):
                    self.i += 1
            except IndexError:
                self.get_logger().info("All goals reached")
            except Exception as e:
                self.get_logger().info(f"Position diff error {e}")


def main(args=None):
    rclpy.init(args=args)

    goal_publisher = GoalPublisher()

    rclpy.spin(goal_publisher)

    goal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

# goal from terminal:
# ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 0.2, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
