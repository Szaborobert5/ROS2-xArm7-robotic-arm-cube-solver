import rclpy
from rclpy.node import Node

from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
)


class MoveTest(Node):

    def __init__(self):
        super().__init__('move_test')

        self.client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

        self.get_logger().info(
            'Cekam MoveIt /move_action...'
        )

        self.client.wait_for_server()

        self.get_logger().info(
            'MoveIt je dostupan.'
        )

        self.send_goal()

    def send_goal(self):

        # Trenutne vrednosti koje smo dobili
        # sa /joint_states

        joints = [
            -1.0015,   # joint1
            -1.4764,   # joint2
            -0.3310,   # joint3
             1.3497,   # joint4
            -0.8261,   # joint5
             2.6862,   # joint6
            -1.8082    # joint7
        ]

        # Mali test pomeraj:
        # samo joint1 menjamo za ~5.7 stepeni

        joints[0] = -1.0015

        goal = MoveGroup.Goal()

        # MoveIt planning group
        goal.request.group_name = 'xarm7'

        # Koristimo trenutno stanje kao start
        goal.request.start_state.is_diff = True

        # -------------------------------------------------
        # Joint constraints
        # -------------------------------------------------

        constraints = Constraints()

        joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6',
            'joint7'
        ]

        for name, position in zip(
            joint_names,
            joints
        ):

            constraint = JointConstraint()

            constraint.joint_name = name
            constraint.position = position

            # Tolerancija oko ciljne pozicije
            constraint.tolerance_above = 0.01
            constraint.tolerance_below = 0.01

            # Težina
            constraint.weight = 1.0

            constraints.joint_constraints.append(
                constraint
            )

        goal.request.goal_constraints.append(
            constraints
        )

        # -------------------------------------------------
        # Planning
        # -------------------------------------------------

        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0

        # -------------------------------------------------
        # Execute
        # -------------------------------------------------

        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True

        self.get_logger().info(
            'Saljem MoveIt cilj...'
        )

        future = self.client.send_goal_async(
            goal
        )

        future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(
        self,
        future
    ):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.get_logger().error(
                'MoveIt je odbio cilj!'
            )

            rclpy.shutdown()
            return

        self.get_logger().info(
            'Cilj prihvacen. Robot se pomera...'
        )

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.result_callback
        )

    def result_callback(
        self,
        future
    ):

        result = future.result().result

        self.get_logger().info(
            f'MoveIt je zavrsio. '
            f'error_code = '
            f'{result.error_code.val}'
        )

        rclpy.shutdown()


def main(args=None):

    rclpy.init(args=args)

    node = MoveTest()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()