#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
import tf2_ros


class CartesianHorizontalTest(Node):

    # Izaberi: LEFT, MIDDLE ili RIGHT
    TEST_SURFACE = "LEFT"

    LEFT_SURFACE_WORLD = (-0.370, -1.125, 1.026)
    MIDDLE_SURFACE_WORLD = (-0.200, -1.125, 1.026)
    RIGHT_SURFACE_WORLD = (-0.030, -1.125, 1.026)

    ROBOT_BASE_X = -0.2
    ROBOT_BASE_Y = -0.5
    ROBOT_BASE_Z = 1.021
    ROBOT_YAW = math.radians(90.0)

    HORIZONTAL_DISTANCE_LIMIT = 0.45

    def __init__(self):
        super().__init__("cartesian_horizontal_test")

        self.latest_joint_state = None

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        self.cartesian_client = self.create_client(
            GetCartesianPath,
            "/compute_cartesian_path"
        )

        self.execute_client = ActionClient(
            self,
            ExecuteTrajectory,
            "/execute_trajectory"
        )

    def joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def wait_for_interfaces(self):
        self.get_logger().info("Cekam /compute_cartesian_path...")
        if not self.cartesian_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/compute_cartesian_path nije dostupan.")
            return False
        self.get_logger().info("/compute_cartesian_path je dostupan.")

        self.get_logger().info("Cekam /execute_trajectory...")
        if not self.execute_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("/execute_trajectory nije dostupan.")
            return False
        self.get_logger().info("/execute_trajectory je dostupan.")
        return True

    def wait_for_joint_state(self):
        self.get_logger().info("Cekam stvarni /joint_states...")
        for _ in range(150):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_joint_state is not None and self.latest_joint_state.name:
                self.get_logger().info("Stvarni /joint_states je dostupan.")
                return True
        self.get_logger().error("Nisam dobio /joint_states.")
        return False

    def get_current_tcp_pose(self):
        self.get_logger().info("Cekam link_base -> link_tcp TF...")
        for attempt in range(200):
            try:
                if self.tf_buffer.can_transform(
                    "link_base",
                    "link_tcp",
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05)
                ):
                    transform = self.tf_buffer.lookup_transform(
                        "link_base",
                        "link_tcp",
                        rclpy.time.Time()
                    )
                    pose = Pose()
                    pose.position.x = transform.transform.translation.x
                    pose.position.y = transform.transform.translation.y
                    pose.position.z = transform.transform.translation.z
                    pose.orientation.x = transform.transform.rotation.x
                    pose.orientation.y = transform.transform.rotation.y
                    pose.orientation.z = transform.transform.rotation.z
                    pose.orientation.w = transform.transform.rotation.w

                    self.get_logger().info(
                        f"TF dostupan nakon ~{attempt * 0.1:.1f} s."
                    )
                    return pose
            except Exception:
                pass

            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().error(
            "Nisam uspeo da pročitam link_base -> link_tcp TF."
        )
        return None

    def build_start_robot_state(self):
        if self.latest_joint_state is None:
            return None

        robot_state = RobotState()
        robot_state.joint_state = self.latest_joint_state
        robot_state.is_diff = False

        self.get_logger().info("STVARNA JOINT KONFIGURACIJA:")
        values = dict(
            zip(
                self.latest_joint_state.name,
                self.latest_joint_state.position
            )
        )

        for name in [
            "joint1", "joint2", "joint3", "joint4",
            "joint5", "joint6", "joint7"
        ]:
            if name in values:
                self.get_logger().info(
                    f"  {name} = {values[name]:.6f}"
                )
        return robot_state

    @staticmethod
    def copy_pose(pose):
        p = Pose()
        p.position.x = pose.position.x
        p.position.y = pose.position.y
        p.position.z = pose.position.z
        p.orientation.x = pose.orientation.x
        p.orientation.y = pose.orientation.y
        p.orientation.z = pose.orientation.z
        p.orientation.w = pose.orientation.w
        return p

    def world_to_base(self, x, y, z):

        dx = x - self.ROBOT_BASE_X
        dy = y - self.ROBOT_BASE_Y

        cos_yaw = math.cos(self.ROBOT_YAW)
        sin_yaw = math.sin(self.ROBOT_YAW)

        base_x = cos_yaw * dx + sin_yaw * dy
        base_y = -sin_yaw * dx + cos_yaw * dy
        base_z = z - self.ROBOT_BASE_Z

        return base_x, base_y, base_z

    def get_surface_world(self):

        name = self.TEST_SURFACE.strip().upper()

        if name == "LEFT":
            return self.LEFT_SURFACE_WORLD

        if name == "MIDDLE":
            return self.MIDDLE_SURFACE_WORLD

        if name == "RIGHT":
            return self.RIGHT_SURFACE_WORLD

        raise ValueError(
            f"Nepoznata povrsina: {self.TEST_SURFACE}"
        )

    def run_test(self):
        self.get_logger().info("========================================")
        self.get_logger().info(" CARTESIAN TRANSPORT TARGET TEST")
        self.get_logger().info("========================================")

        if not self.wait_for_interfaces():
            return False
        if not self.wait_for_joint_state():
            return False

        start_pose = self.get_current_tcp_pose()
        if start_pose is None:
            return False

        start_state = self.build_start_robot_state()
        if start_state is None:
            return False

        # Safety check. The test starts with a 5 cm upward move.
        if start_pose.position.z < 0.10:
            self.get_logger().error(
                f"TCP je prenizak za test: Z={start_pose.position.z:.4f} m."
            )
            self.get_logger().error(
                "Pomerite robota u visu, bezbedniju poziciju."
            )
            return False

        try:
            surface_world = self.get_surface_world()
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return False

        target_x, target_y, target_surface_z = self.world_to_base(
            surface_world[0],
            surface_world[1],
            surface_world[2],
        )

        if start_pose.position.z < 0.10:
            self.get_logger().error(
                f"TCP je prenizak za test: "
                f"Z={start_pose.position.z:.4f} m."
            )
            self.get_logger().error(
                "Pomerite robota u visu, bezbedniju poziciju."
            )
            return False

        try:
            surface_world = self.get_surface_world()
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return False

        target_x, target_y, target_surface_z = self.world_to_base(
            surface_world[0],
            surface_world[1],
            surface_world[2],
        )

        if start_pose.position.z < 0.10:
            self.get_logger().error(
                f"TCP je prenizak za test: "
                f"Z={start_pose.position.z:.4f} m."
            )
            self.get_logger().error(
                "Pomerite robota u visu, bezbedniju poziciju."
            )
            return False

        dx = target_x - start_pose.position.x
        dy = target_y - start_pose.position.y
        horizontal_distance = (dx * dx + dy * dy) ** 0.5

        if horizontal_distance > self.HORIZONTAL_DISTANCE_LIMIT:
            self.get_logger().error(
                f"Horizontalni pomak je prevelik: "
                f"{horizontal_distance:.4f} m."
            )
            self.get_logger().error(
                f"Test je ograničen na "
                f"{self.HORIZONTAL_DISTANCE_LIMIT:.2f} m."
            )
            return False

        # Ključni test:
        # Z ostaje TAČNO isti. Nema prethodnog podizanja.
        target_pose = self.copy_pose(start_pose)
        target_pose.position.x = target_x
        target_pose.position.y = target_y
        target_pose.position.z = start_pose.position.z

        self.get_logger().info("----------------------------------------")
        self.get_logger().info(
            f"TEST POVRSINA: {self.TEST_SURFACE.upper()}"
        )
        self.get_logger().info(
            "DUGI HORIZONTALNI CARTESIAN TEST"
        )
        self.get_logger().info(
            f"START: "
            f"X={start_pose.position.x:.4f}, "
            f"Y={start_pose.position.y:.4f}, "
            f"Z={start_pose.position.z:.4f}"
        )
        self.get_logger().info(
            f"CILJ:  "
            f"X={target_x:.4f}, "
            f"Y={target_y:.4f}, "
            f"Z={start_pose.position.z:.4f}"
        )
        self.get_logger().info(
            f"AUTOMATSKI SMER: "
            f"dX={dx:+.4f} m, dY={dy:+.4f} m"
        )
        self.get_logger().info(
            f"HORIZONTALNA UDALJENOST (TEST): "
            f"{horizontal_distance:.4f} m"
        )
        self.get_logger().info(
            "Z NE SME DA SE MENJA."
        )

        waypoints = [
            start_pose,
            target_pose,
        ]

        request = GetCartesianPath.Request()
        request.header.frame_id = "link_base"
        request.start_state = start_state
        request.group_name = "xarm7"
        request.link_name = "link_tcp"
        request.waypoints = waypoints
        request.max_step = 0.005
        request.jump_threshold = 0.0
        request.prismatic_jump_threshold = 0.0
        request.revolute_jump_threshold = 0.0
        request.avoid_collisions = True
        request.max_cartesian_speed = 0.05
        request.cartesian_speed_limited_link = "link_tcp"

        self.get_logger().info("----------------------------------------")
        self.get_logger().info("Pozivam /compute_cartesian_path...")

        future = self.cartesian_client.call_async(request)
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        if not future.done():
            self.get_logger().error(
                "Cartesian service nije vratio rezultat."
            )
            return False

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(
                f"Greška pri Cartesian planiranju: {exc}"
            )
            return False

        self.get_logger().info("----------------------------------------")
        self.get_logger().info(
            f"Cartesian fraction = {response.fraction:.3f}"
        )
        self.get_logger().info(
            f"Cartesian error_code = {response.error_code.val}"
        )

        if response.fraction < 0.99 or response.error_code.val != 1:
            self.get_logger().error(
                "Cartesian transport putanja nije validna."
            )
            self.get_logger().error(
                "NE izvršavam trajektoriju."
            )
            return False

        if not response.solution.joint_trajectory.points:
            self.get_logger().error(
                "Nema joint trajectory tačaka."
            )
            return False

        self.get_logger().info(
            f"Cartesian transport putanja je validna. "
            f"Broj tačaka = "
            f"{len(response.solution.joint_trajectory.points)}"
        )

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = response.solution
        goal.controller_names = []

        self.get_logger().info(
            "Šaljem trajektoriju na /execute_trajectory..."
        )

        send_future = self.execute_client.send_goal_async(goal)
        while rclpy.ok() and not send_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        if not send_future.done():
            self.get_logger().error(
                "ExecuteTrajectory nije odgovorio."
            )
            return False

        try:
            goal_handle = send_future.result()
        except Exception as exc:
            self.get_logger().error(
                f"Greška pri slanju trajektorije: {exc}"
            )
            return False

        if not goal_handle.accepted:
            self.get_logger().error(
                "ExecuteTrajectory je odbio trajektoriju."
            )
            return False

        self.get_logger().info(
            "ExecuteTrajectory cilj prihvaćen."
        )
        self.get_logger().info(
            "Robot sada treba da ide SAMO HORIZONTALNO do izračunatog X/Y cilja."
        )

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        if not result_future.done():
            self.get_logger().error(
                "ExecuteTrajectory rezultat nije vraćen."
            )
            return False

        try:
            result = result_future.result().result
        except Exception as exc:
            self.get_logger().error(
                f"Greška pri čitanju rezultata: {exc}"
            )
            return False

        self.get_logger().info("========================================")
        self.get_logger().info(
            f"ExecuteTrajectory error_code = {result.error_code.val}"
        )
        self.get_logger().info("========================================")

        if result.error_code.val == 1:
            self.get_logger().info("TEST USPESAN.")
            self.get_logger().info(
                "Cartesian transport do izabrane povrsine je uspesno izvrsen."
            )
            return True

        self.get_logger().error("TEST NIJE USPEO.")
        return False


def main(args=None):
    rclpy.init(args=args)
    node = CartesianHorizontalTest()

    try:
        success = node.run_test()
        return 0 if success else 1
    except KeyboardInterrupt:
        node.get_logger().info("Test prekinut.")
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
