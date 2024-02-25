import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class GoalPublisher(Node):

    def __init__(self):
        super().__init__("goal_publisher")
        self.publisher_ = self.create_publisher(PoseStamped, "goal_pose", 10)
        timer_period = 3  # sekundy
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, "pose", self.position_sub, 10
        )
        self.subscription  # prevent unused variable warning
        self.actual_position_pose = {"x": -100, "y": -100, "z": -100, "w": -100}
        self.actual_position_tf_base_link = dict()

        self.points = [
            {"x": 0.2, "y": 0.0, "z": 0.0, "w": 1.0},
            {"x": 4.5, "y": 0.0, "z": 0.0, "w": 1.0},
            {"x": 4.5, "y": -1.0, "z": 0.0, "w": 1.0},
            {"x": 4.5, "y": -3.0, "z": 0.0, "w": 1.0},
            {"x": 4.5, "y": -5.0, "z": 0.0, "w": 1.0},
            {"x": 4.5, "y": -7.0, "z": 0.0, "w": 1.0},
        ]
        self.i = 0

        # Dodajemy TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def different(self, a: float, b: float, margin: float) -> bool:
        a = float(a)
        b = float(b)
        return True if abs(a - b) <= margin else False

    def position_sub(self, msg):

        self.actual_position_pose["x"] = msg.pose.pose.position.x
        self.actual_position_pose["y"] = msg.pose.pose.position.y
        self.actual_position_pose["z"] = msg.pose.pose.position.z
        self.actual_position_pose["w"] = msg.pose.pose.orientation.w

    def timer_callback(self):
        try:
            msg = PoseStamped()
            msg.header.stamp.sec = 0
            msg.header.frame_id = "map"
            msg.pose.position.x = self.points[self.i]["x"]
            msg.pose.position.y = self.points[self.i]["y"]
            msg.pose.position.z = self.points[self.i]["z"]
            msg.pose.orientation.w = self.points[self.i]["w"]
            self.publisher_.publish(msg)
            self.get_logger().info(
                f"\nGoal:\nnr:{self.i+1}/{len(self.points)}\nx: {msg.pose.position.x}\ny: {msg.pose.position.y}\nz: {msg.pose.position.z}\nw: {msg.pose.orientation.w}\n"
            )

        except IndexError:
            self.get_logger().info("All goals reached")
            msg = PoseStamped()
            msg.header.stamp.sec = 0
            msg.header.frame_id = "map"
            msg.pose.position.x = self.points[0]["x"]
            msg.pose.position.y = self.points[0]["y"]
            msg.pose.position.z = self.points[0]["z"]
            msg.pose.orientation.w = self.points[0]["w"]
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
            self.actual_position_tf_base_link["x"] = trans.transform.translation.x
            self.actual_position_tf_base_link["y"] = trans.transform.translation.y
            self.actual_position_tf_base_link["z"] = trans.transform.translation.z
            self.actual_position_tf_base_link["w"] = trans.transform.rotation.w
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
                    self.actual_position_tf_base_link["x"],
                    self.points[self.i]["x"],
                    margin,
                )
                and self.different(
                    self.actual_position_tf_base_link["y"],
                    self.points[self.i]["y"],
                    margin,
                )
                and self.different(
                    self.actual_position_tf_base_link["z"],
                    self.points[self.i]["z"],
                    margin,
                )
                and self.different(
                    self.actual_position_tf_base_link["w"],
                    self.points[self.i]["w"],
                    margin,
                )
            ):
                self.i += 1
        except:
            # if tf goes down posewithcovariencestamped of robot will take a lead
            try:
                if (
                    self.different(
                        self.actual_position_pose["x"],
                        self.points[self.i]["x"],
                        margin,
                    )
                    and self.different(
                        self.actual_position_pose["y"],
                        self.points[self.i]["y"],
                        margin,
                    )
                    and self.different(
                        self.actual_position_pose["z"],
                        self.points[self.i]["z"],
                        margin,
                    )
                    and self.different(
                        self.actual_position_pose["w"],
                        self.points[self.i]["w"],
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
