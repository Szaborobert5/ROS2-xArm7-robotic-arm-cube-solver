import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class GripperTest(Node):

    def __init__(self):
        super().__init__('gripper_test')

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            '/xarm_gripper_traj_controller/follow_joint_trajectory'
        )

        self.get_logger().info(
            '========================================'
        )
        self.get_logger().info(
            '       GRIPPER TEST'
        )
        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'Cekam gripper controller...'
        )

        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'Gripper controller nije dostupan!'
            )
            rclpy.shutdown()
            return

        self.get_logger().info(
            'Gripper controller je dostupan.'
        )

        # Prvo OPEN
        self.send_gripper_goal(0.0)

    def send_gripper_goal(self, position):

        goal = FollowJointTrajectory.Goal()

        # -------------------------------------------------
        # Joint kojim upravlja gripper
        # -------------------------------------------------

        goal.trajectory.joint_names = [
            'drive_joint'
        ]

        # -------------------------------------------------
        # Ciljna pozicija
        # -------------------------------------------------

        point = JointTrajectoryPoint()

        point.positions = [
            position
        ]

        # 1 sekunda za pokret
        point.time_from_start.sec = 1

        goal.trajectory.points = [
            point
        ]

        if position == 0.0:
            self.get_logger().info(
                'SALJEM: GRIPPER OPEN (0.0)'
            )
        else:
            self.get_logger().info(
                'SALJEM: GRIPPER CLOSE (0.85)'
            )

        future = self.client.send_goal_async(
            goal
        )

        future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.get_logger().error(
                'Gripper je odbio cilj!'
            )

            rclpy.shutdown()
            return

        self.get_logger().info(
            'Gripper cilj prihvacen.'
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.result_callback
        )

    def result_callback(self, future):

        result = future.result().result

        self.get_logger().info(
            f'Gripper zavrsio. error_code = '
            f'{result.error_code}'
        )

        # -------------------------------------------------
        # Ako je upravo zavrsio OPEN
        # -------------------------------------------------

        if result.error_code == 0:

            # Ako smo poslali OPEN
            # sada posalji CLOSE

            # Proveravamo trenutnu komandu
            #
            # Jednostavan test:
            # ako je trenutno bio OPEN,
            # saljemo CLOSE.

            self.get_logger().info(
                'Cekam 2 sekunde pre CLOSE...'
            )

            self.timer = self.create_timer(
                2.0,
                self.send_close_once
            )

        else:

            self.get_logger().error(
                'Gripper nije uspesno zavrsio!'
            )

            rclpy.shutdown()

    def send_close_once(self):

        self.timer.cancel()

        self.send_gripper_goal(0.85)


def main(args=None):

    rclpy.init(args=args)

    node = GripperTest()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()