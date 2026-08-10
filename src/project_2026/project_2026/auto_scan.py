import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from std_msgs.msg import Int32

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint


class AutoScan(Node):

    def __init__(self):
        super().__init__('auto_scan')

        # =====================================================
        # MOVEIT
        # =====================================================

        self.move_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

        # Trenutni MoveIt goal
        self.current_goal_handle = None

        # =====================================================
        # CUBE DETECTOR
        # =====================================================

        self.cube_count = 0

        self.cube_count_sub = self.create_subscription(
            Int32,
            '/cube_count',
            self.cube_count_callback,
            10
        )

        # =====================================================
        # JOINT STATES
        # =====================================================

        self.current_joints = {}

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # =====================================================
        # SCAN STATE
        # =====================================================

        self.scan_positions = []

        self.scan_index = 0

        self.waiting_for_result = False

        self.waiting_for_camera = False

        self.scan_finished = False

        # =====================================================
        # TIMER
        # =====================================================

        self.timer = self.create_timer(
            0.2,
            self.scan_loop
        )

        self.camera_wait_timer = None

        # Timer za odlozeni STOP
        self.delayed_stop_timer = None

        # =====================================================
        # START
        # =====================================================

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            '          AUTO SCAN START'
        )

        self.get_logger().info(
            '========================================'
        )

    # =========================================================
    # JOINT STATES
    # =========================================================

    def joint_state_callback(self, msg):

        for name, position in zip(
            msg.name,
            msg.position
        ):
            self.current_joints[name] = position

    # =========================================================
    # CUBE COUNT
    # =========================================================

    def cube_count_callback(self, msg):

        self.cube_count = msg.data

        # Ako je scan vec zavrsen,
        # ne radimo nista.
        if self.scan_finished:
            return

        # Dok cekamo stabilizaciju kamere,
        # ne reagujemo.
        if self.waiting_for_camera:
            return

        # =====================================================
        # 5/5 DETEKTOVANO
        # =====================================================

        if self.cube_count >= 5:

            # Ako je odlozeni STOP vec pokrenut,
            # ne pokrecemo ga ponovo.
            if self.delayed_stop_timer is not None:
                return

            self.get_logger().info(
                '========================================'
            )

            self.get_logger().info(
                '5/5 CUBES DETECTED!'
            )

            self.get_logger().info(
                'Robot nastavlja jos 0.4 sekunde...'
            )

            self.get_logger().info(
                '========================================'
            )

            # Robot nastavlja jos 0.4 s
            self.delayed_stop_timer = self.create_timer(
                0.4,
                self.delayed_stop_callback
            )

        # =====================================================
        # MANJE OD 5
        # =====================================================

        else:

            self.get_logger().info(
                f'Cubes detected: '
                f'{self.cube_count}/5'
            )

    # =========================================================
    # MAIN SCAN LOOP
    # =========================================================

    def scan_loop(self):

        if self.scan_finished:
            return

        # Cekamo joint states
        if not self.current_joints:
            return

        # Cekamo MoveIt
        if not self.move_client.server_is_ready():
            return

        # Ako je robot vec u pokretu
        if self.waiting_for_result:
            return

        # Ako cekamo kameru
        if self.waiting_for_camera:
            return

        # =====================================================
        # SCAN POZICIJA
        # =====================================================

        if not self.scan_positions:

            # -------------------------------------------------
            # PROVERENA POZICIJA
            # -------------------------------------------------

            scan1 = [
                -1.0015103,
                -1.4763914,
                -0.3309921,
                 1.3497441,
                -0.8261122,
                 2.6861801,
                -1.8082478
            ]

            self.scan_positions = [
                scan1
            ]

            self.get_logger().info(
                'Koristim proverenu scan poziciju.'
            )

            self.get_logger().info(
                '----------------------------------------'
            )

            self.get_logger().info(
                f'joint1 = {scan1[0]:.4f}'
            )

            self.get_logger().info(
                f'joint2 = {scan1[1]:.4f}'
            )

            self.get_logger().info(
                f'joint3 = {scan1[2]:.4f}'
            )

            self.get_logger().info(
                f'joint4 = {scan1[3]:.4f}'
            )

            self.get_logger().info(
                f'joint5 = {scan1[4]:.4f}'
            )

            self.get_logger().info(
                f'joint6 = {scan1[5]:.4f}'
            )

            self.get_logger().info(
                f'joint7 = {scan1[6]:.4f}'
            )

            self.get_logger().info(
                '----------------------------------------'
            )

        # =====================================================
        # SVE POZICIJE ZAVRSENE
        # =====================================================

        if self.scan_index >= len(
            self.scan_positions
        ):

            self.finish_scan(
                success=False
            )

            return

        # =====================================================
        # POSALJI CILJ
        # =====================================================

        target = self.scan_positions[
            self.scan_index
        ]

        self.send_move_goal(
            target
        )

    # =========================================================
    # SEND MOVEIT GOAL
    # =========================================================

    def send_move_goal(self, joints):

        self.waiting_for_result = True

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            f'SCAN {self.scan_index + 1}/'
            f'{len(self.scan_positions)}'
        )

        self.get_logger().info(
            f'Target joint1 = {joints[0]:.4f}'
        )

        self.get_logger().info(
            'Brzina robota: 40%'
        )

        # -----------------------------------------------------
        # Joint names
        # -----------------------------------------------------

        joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6',
            'joint7'
        ]

        # -----------------------------------------------------
        # Constraints
        # -----------------------------------------------------

        constraints = Constraints()

        for name, position in zip(
            joint_names,
            joints
        ):

            constraint = JointConstraint()

            constraint.joint_name = name

            constraint.position = position

            constraint.tolerance_above = 0.01

            constraint.tolerance_below = 0.01

            constraint.weight = 1.0

            constraints.joint_constraints.append(
                constraint
            )

        # -----------------------------------------------------
        # MoveGroup goal
        # -----------------------------------------------------

        goal = MoveGroup.Goal()

        goal.request.group_name = 'xarm7'

        goal.request.goal_constraints.append(
            constraints
        )

        goal.request.num_planning_attempts = 5

        goal.request.allowed_planning_time = 5.0

        # =====================================================
        # SMANJENJE BRZINE I UBRZANJA
        # =====================================================

        goal.request.max_velocity_scaling_factor = 0.4

        goal.request.max_acceleration_scaling_factor = 0.4

        # -----------------------------------------------------
        # Planning options
        # -----------------------------------------------------

        goal.planning_options.plan_only = False

        goal.planning_options.look_around = False

        goal.planning_options.replan = True

        # -----------------------------------------------------
        # Send
        # -----------------------------------------------------

        self.get_logger().info(
            'Saljem cilj MoveIt-u...'
        )

        future = self.move_client.send_goal_async(
            goal
        )

        future.add_done_callback(
            self.goal_response_callback
        )

    # =========================================================
    # MOVEIT GOAL RESPONSE
    # =========================================================

    def goal_response_callback(self, future):

        try:

            goal_handle = future.result()

        except Exception as e:

            self.get_logger().error(
                f'Greska pri slanju cilja: {e}'
            )

            self.waiting_for_result = False

            return

        # -----------------------------------------------------
        # Goal rejected
        # -----------------------------------------------------

        if not goal_handle.accepted:

            self.get_logger().error(
                'MoveIt je odbio cilj.'
            )

            self.waiting_for_result = False

            return

        # -----------------------------------------------------
        # Sacuvaj goal handle
        # -----------------------------------------------------

        self.current_goal_handle = goal_handle

        self.get_logger().info(
            'Cilj prihvacen.'
        )

        self.get_logger().info(
            'Robot se pomera ka scan poziciji...'
        )

        # -----------------------------------------------------
        # Result
        # -----------------------------------------------------

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.move_result_callback
        )

    # =========================================================
    # MOVEIT RESULT
    # =========================================================

    def move_result_callback(self, future):

        # -----------------------------------------------------
        # Ako smo vec zavrsili scan zbog 5/5,
        # ne radimo nista vise.
        # -----------------------------------------------------

        if self.scan_finished:

            self.waiting_for_result = False

            self.current_goal_handle = None

            return

        try:

            result = future.result().result

            error_code = result.error_code.val

            self.get_logger().info(
                f'MoveIt zavrsio. '
                f'error_code = {error_code}'
            )

        except Exception as e:

            self.get_logger().error(
                f'Greska pri rezultatu MoveIt-a: {e}'
            )

        self.waiting_for_result = False

        self.current_goal_handle = None

        # -----------------------------------------------------
        # Ako je 5/5 vec detektovano
        # -----------------------------------------------------

        if self.cube_count >= 5:

            self.finish_scan(
                success=True
            )

            return

        # -----------------------------------------------------
        # Robot je stigao.
        # -----------------------------------------------------

        self.waiting_for_camera = True

        self.get_logger().info(
            'Robot je stigao na scan poziciju.'
        )

        self.get_logger().info(
            'Cekam 1.5 sekundi da se kamera stabilizuje...'
        )

        self.camera_wait_timer = self.create_timer(
            1.5,
            self.camera_ready_callback
        )

    # =========================================================
    # CAMERA READY
    # =========================================================

    def camera_ready_callback(self):

        if self.camera_wait_timer is not None:

            self.camera_wait_timer.cancel()

            self.destroy_timer(
                self.camera_wait_timer
            )

            self.camera_wait_timer = None

        self.waiting_for_camera = False

        self.get_logger().info(
            'Kamera spremna.'
        )

        self.get_logger().info(
            'Cekam detekciju kockica...'
        )

        self.scan_index += 1

    # =========================================================
    # DELAYED STOP
    # =========================================================

    def delayed_stop_callback(self):

        # -----------------------------------------------------
        # Zaustavi timer
        # -----------------------------------------------------

        if self.delayed_stop_timer is not None:

            self.delayed_stop_timer.cancel()

            self.destroy_timer(
                self.delayed_stop_timer
            )

            self.delayed_stop_timer = None

        self.get_logger().info(
            '0.4 sekunde proslo.'
        )

        self.get_logger().info(
            'Zaustavljam robota...'
        )

        # -----------------------------------------------------
        # Zavrsi scan.
        #
        # finish_scan() ce otkazati aktivni MoveIt cilj.
        # -----------------------------------------------------

        self.finish_scan(
            success=True
        )

    # =========================================================
    # FINISH SCAN
    # =========================================================

    def finish_scan(self, success):

        if self.scan_finished:
            return

        self.scan_finished = True

        # =====================================================
        # SUCCESS
        # =====================================================

        if success:

            self.get_logger().info(
                '========================================'
            )

            self.get_logger().info(
                '        5/5 CUBES DETECTED!'
            )

            # -------------------------------------------------
            # OTKAŽI AKTIVNI MOVEIT GOAL
            # -------------------------------------------------

            if self.current_goal_handle is not None:

                self.get_logger().info(
                    'Saljem CANCEL MoveIt cilju...'
                )

                try:

                    cancel_future = (
                        self.current_goal_handle
                        .cancel_goal_async()
                    )

                    cancel_future.add_done_callback(
                        self.cancel_done_callback
                    )

                except Exception as e:

                    self.get_logger().warn(
                        f'Greska pri cancelovanju: {e}'
                    )

            else:

                self.get_logger().info(
                    'Robot vise nema aktivan MoveIt cilj.'
                )

            self.get_logger().info(
                'SCAN ZAVRSEN'
            )

            self.get_logger().info(
                'ROBOT STOP'
            )

            self.get_logger().info(
                '========================================'
            )

        # =====================================================
        # FAILURE
        # =====================================================

        else:

            self.get_logger().warn(
                '========================================'
            )

            self.get_logger().warn(
                'SCAN ZAVRSEN'
            )

            self.get_logger().warn(
                'Nisu detektovane sve kockice.'
            )

            self.get_logger().warn(
                f'Poslednja detekcija: '
                f'{self.cube_count}/5'
            )

            self.get_logger().warn(
                '========================================'
            )

    # =========================================================
    # CANCEL CALLBACK
    # =========================================================

    def cancel_done_callback(self, future):

        try:

            future.result()

            self.get_logger().info(
                'MoveIt cilj je otkazan.'
            )

        except Exception as e:

            self.get_logger().warn(
                f'Greska pri otkazivanju MoveIt cilja: {e}'
            )


# =============================================================
# MAIN
# =============================================================

def main(args=None):

    rclpy.init(args=args)

    node = AutoScan()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info(
            'AUTO SCAN prekinut.'
        )

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':
    main()