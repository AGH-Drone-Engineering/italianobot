import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import sys
import os

home_dir = os.environ["HOME"]
pgm_file = os.path.join(home_dir, "ros2_ws/src/italianobot/itabot_aruco/itabot_aruco")
sys.path.insert(0, pgm_file)
import pgm_to_nav2points as nav2p


class GoalPublisher(Node):

    def __init__(self):
        home_dir = os.environ["HOME"]
        yaml_file = os.path.join(
            home_dir, "ros2_ws/src/italianobot/itabot_aruco/itabot_aruco/map/map.yaml"
        )
        map_file = os.path.join(
            home_dir, "ros2_ws/src/italianobot/itabot_aruco/itabot_aruco/map/map.pgm"
        )
        map_data = nav2p.load_map_data(yaml_file)
        resolution = map_data["resolution"]
        init_pose = map_data["origin"][:2]
        MARGIN = 50  # minimal spacing between points
        WALL_DET = 5  # minimal spacing between rosbot and wall
        self.points = nav2p.calculate_goal_points(
            map_file, resolution, init_pose, MARGIN, WALL_DET
        )

        super().__init__("goal_publisher")
        self.publisher_ = self.create_publisher(PoseStamped, "goal_pose", 10)
        timer_period = 3  # sekundy
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, "pose", self.position_sub, 10
        )
        self.subscription  # prevent unused variable warning
        self.actual_position_pose = {
            "px": -100,
            "py": -100,
            "pz": -100,
            "ow": -100,
            "ox": -100,
            "oy": -100,
            "oz": -100,
        }
        self.actual_position_tf_base_link = dict()

        self.i = 0

        # Dodajemy TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def different(self, a: float, b: float, margin: float) -> bool:
        a = float(a)
        b = float(b)
        return True if abs(a - b) <= margin else False

    def position_sub(self, msg):

        self.actual_position_pose["px"] = msg.pose.pose.position.x
        self.actual_position_pose["py"] = msg.pose.pose.position.y
        self.actual_position_pose["pz"] = msg.pose.pose.position.z
        self.actual_position_pose["ox"] = msg.pose.pose.orientation.x
        self.actual_position_pose["oy"] = msg.pose.pose.orientation.y
        self.actual_position_pose["oz"] = msg.pose.pose.orientation.z
        self.actual_position_pose["ow"] = msg.pose.pose.orientation.w

    def timer_callback(self):
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
            msg.pose.position.x = self.points[0]["px"]
            msg.pose.position.y = self.points[0]["py"]
            msg.pose.position.z = self.points[0]["pz"]
            msg.pose.orientation.x = self.points[0]["ox"]
            msg.pose.orientation.y = self.points[0]["oy"]
            msg.pose.orientation.z = self.points[0]["oz"]
            msg.pose.orientation.w = self.points[0]["ow"]

            self.publisher_.publish(msg)
            self.get_logger().info("Returning to the base")

        except Exception as e:
            self.get_logger().info(f"{e}")

        # current PoseWithCovarianceStamped position
        self.get_logger().info(
            f"\nCURRENT POSITION POSE_WITH_COVARIANCE_STAMPED: \n{self.actual_position_pose}\n"
        )
        # read transform
        try:
            trans = self.tf_buffer.lookup_transform(
                "odom", "base_link", rclpy.time.Time()
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
            self.get_logger().info("Could not transform odom to base_link: %s" % str(e))

        # changing i to i+ if goal reached
        margin = 0.5
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
