import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Empty
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

from cube_interfaces.msg import CubeDetectionArray
from cube_interfaces.srv import DetectSurface
from cube_interfaces.srv import GetClosestCube

from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    JointConstraint,
    RobotState
)

from shape_msgs.msg import SolidPrimitive

import tf2_ros


class MoveToCube(Node):

    # =============================================================
    # ROBOT WORLD POZICIJA
    # =============================================================

    ROBOT_BASE_X = -0.2
    ROBOT_BASE_Y = -0.5
    ROBOT_BASE_Z = 1.021

    # Robot je u WORLD-u rotiran +90 stepeni
    ROBOT_YAW = math.radians(90.0)

    # =============================================================
    # KRETANJE DO KOCKICE
    # =============================================================

    # Prvo idemo 15 cm iznad kockice
    ABOVE_CUBE = 0.15

    # Prilaz kockici
    APPROACH_OFFSET = 0.005

    # FAZA 2 koristi bezbednu međuvisinu pre završnog prilaza.
    APPROACH_SAFE_HEIGHT = 0.10

    # Posle attach-a podižemo kockicu 20 cm
    LIFT_HEIGHT = 0.20

    # =============================================================
    # POVRŠINE
    # =============================================================

    LEFT_SURFACE_WORLD = (
        -0.370,
        -1.125,
        1.026
    )

    MIDDLE_SURFACE_WORLD = (
        -0.200,
        -1.125,
        1.026
    )

    RIGHT_SURFACE_WORLD = (
        -0.030,
        -1.125,
        1.026
    )

    # TCP je 15 cm iznad površine dok je skeniramo.
    SURFACE_HEIGHT_OFFSET = 0.15

    # Spuštanje TCP-a pre detach-a.
    DROP_TCP_OFFSET = 0.07

    # =============================================================
    # BEZBEDAN DROP - DODATNA ZAŠTITA
    # =============================================================
    DROP_MIN_TCP_CLEARANCE = 0.10
    DROP_PREPARE_CLEARANCE = 0.12
    DROP_TIME_SCALE = 2.5
    DROP_SETTLE_TIME = 0.5
    DROP_MAX_TCP_SPEED = 0.025

    DROP_ORIENTATION_TOLERANCE = 0.50
    DROP_PLANNING_TIME = 8.0

    # =============================================================
    # SLOTOVI ZA VIŠE KOCKICA NA ISTOJ POVRŠINI
    # =============================================================
    SURFACE_SLOT_OFFSETS = [
        (-0.05, -0.05),
        (0.05, 0.05),
        (0.05, -0.05),
        (0.05, 0.05),
        (0.00, -0.12),
    ]

    # =============================================================
    # AUTO-SCAN POZICIJE (PRIMARNA I SEKUNDARNA)
    # =============================================================
    AUTO_SCAN_POSITION = [
        -1.0015103,
        -1.4763914,
        -0.3309921,
         1.3497441,
        -0.8261122,
         2.6861801,
        -1.8082478
    ]

    # Sekundarna scan pozicija sa blagim ugaonom pomerajem radi šireg FOV
    SECONDARY_SCAN_POSITION = [
        -0.8015103,
        -1.4763914,
        -0.3309921,
         1.3497441,
        -0.8261122,
         2.6861801,
        -1.8082478
    ]

    # =============================================================
    # KONSTRUKTOR
    # =============================================================

    def __init__(self):

        super().__init__('move_to_cube')

        # =========================================================
        # CUBE DETECTION
        # =========================================================

        self.cube_subscription = self.create_subscription(
            CubeDetectionArray,
            '/detected_cubes',
            self.cubes_callback,
            10
        )

        # =========================================================
        # MOVEIT
        # =========================================================

        self.move_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

        # Cartesian DROP API
        self.cartesian_client = self.create_client(
            GetCartesianPath,
            '/compute_cartesian_path'
        )

        self.execute_trajectory_client = ActionClient(
            self,
            ExecuteTrajectory,
            '/execute_trajectory'
        )

        # =========================================================
        # CLOSEST CUBE SERVICE
        # =========================================================

        self.closest_client = self.create_client(
            GetClosestCube,
            '/get_closest_cube'
        )

        # =========================================================
        # SURFACE DETECTOR SERVICE
        # =========================================================

        self.surface_client = self.create_client(
            DetectSurface,
            '/detect_surface'
        )

        # =========================================================
        # TF2
        # =========================================================

        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # =========================================================
        # DEBUG LOGGER - JOINT STATE
        # =========================================================

        self.latest_joint_positions = {}

        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # =========================================================
        # ATTACH / DETACH PUBLISHERS
        # =========================================================

        self.attach_publishers = {}
        self.detach_publishers = {}

        for i in range(5):

            cube_name = f'cube_{i}'

            attach_topic = f'/gz/{cube_name}/attach'
            detach_topic = f'/gz/{cube_name}/detach'

            self.attach_publishers[cube_name] = (
                self.create_publisher(
                    Empty,
                    attach_topic,
                    10
                )
            )

            self.detach_publishers[cube_name] = (
                self.create_publisher(
                    Empty,
                    detach_topic,
                    10
                )
            )

        # =========================================================
        # FAZE I RETRY LOGIKA
        # =========================================================

        self.phase = 0
        self.approach_retry_count = 0
        self.approach_step = 0
        self.drop_retry_count = 0
        self.drop_step = None
        self.goal_sent = False

        self.move_retry_count = 0
        self.last_move_target = None

        # =========================================================
        # PODACI O KOCKICI
        # =========================================================

        self.detected_id = None
        self.detected_color = None

        self.cube_x = None
        self.cube_y = None
        self.cube_z = None

        self.real_cube_id = None
        self.expected_gazebo_id = None

        # =========================================================
        # SERVICE PROVERA KOCKICE
        # =========================================================

        self.service_check_count = 0
        self.service_check_results = []

        self.candidate_selection_in_progress = False
        self.selection_candidates = []
        self.selection_index = 0

        # =========================================================
        # SCAN STANJE (PROBLEM 2 REŠENJE)
        # =========================================================

        self.scan_state = 'PRIMARY'  # 'PRIMARY' ili 'SECONDARY'
        self.empty_scan_count = 0
        self.MAX_EMPTY_SCAN_FRAMES = 15

        # =========================================================
        # SURFACE STANJE
        # =========================================================

        self.current_surface_name = None
        self.current_surface_world = None
        self.detected_surface_color = None
        self.surface_service_pending = False

        # =========================================================
        # MULTI-CUBE STANJE
        # =========================================================

        self.completed_gazebo_ids = set()
        self.placed_counts = {
            'RED': 0,
            'GREEN': 0,
            'BLUE': 0,
        }

        self.current_drop_offset = (0.0, 0.0)
        self.current_drop_world = None
        self.auto_scan_retry_count = 0

        self.target_cube_count = None
        self.count_candidate = None
        self.count_stable_frames = 0
        self.COUNT_STABLE_REQUIRED = 5

        # =========================================================
        # LOG
        # =========================================================

        self.get_logger().info('========================================')
        self.get_logger().info('       MOVE TO CUBE - INTEGRATED FIXES')
        self.get_logger().info('========================================')
        self.get_logger().info('Cube detector: /detected_cubes')
        self.get_logger().info('Surface detector: /detect_surface')
        self.get_logger().info('Surface order: LEFT -> MIDDLE -> RIGHT')
        self.get_logger().info(
            f'Robot base WORLD: ({self.ROBOT_BASE_X}, {self.ROBOT_BASE_Y}, {self.ROBOT_BASE_Z})'
        )
        self.get_logger().info('Robot WORLD yaw: +90 deg')
        self.get_logger().info('Cekam /detected_cubes...')

    # =============================================================
    # DEBUG - JOINT STANJE
    # =============================================================

    def joint_state_callback(self, msg):
        self.latest_joint_positions = {
            name: position
            for name, position in zip(msg.name, msg.position)
        }

    def log_current_joints(self, title):
        self.get_logger().info('----------------------------------------')
        self.get_logger().info(title)

        joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7'
        ]

        if not self.latest_joint_positions:
            self.get_logger().warn('/joint_states nema dostupne podatke.')
            self.get_logger().info('----------------------------------------')
            return

        for name in joint_names:
            if name in self.latest_joint_positions:
                self.get_logger().info(
                    f'{name} = {self.latest_joint_positions[name]:.6f}'
                )
            else:
                self.get_logger().warn(f'{name} nije pronađen u /joint_states.')

        self.get_logger().info('----------------------------------------')

    # =============================================================
    # LINK_BASE -> WORLD
    # =============================================================

    def base_to_world(self, x, y, z):
        cos_yaw = math.cos(self.ROBOT_YAW)
        sin_yaw = math.sin(self.ROBOT_YAW)

        world_x = self.ROBOT_BASE_X + cos_yaw * x - sin_yaw * y
        world_y = self.ROBOT_BASE_Y + sin_yaw * x + cos_yaw * y
        world_z = self.ROBOT_BASE_Z + z

        return world_x, world_y, world_z

    # =============================================================
    # WORLD -> LINK_BASE
    # =============================================================

    def world_to_base(self, x, y, z):
        dx = x - self.ROBOT_BASE_X
        dy = y - self.ROBOT_BASE_Y

        cos_yaw = math.cos(self.ROBOT_YAW)
        sin_yaw = math.sin(self.ROBOT_YAW)

        base_x = cos_yaw * dx + sin_yaw * dy
        base_y = -sin_yaw * dx + cos_yaw * dy
        base_z = z - self.ROBOT_BASE_Z

        return base_x, base_y, base_z

    # =============================================================
    # CUBE DETECTION
    # =============================================================

    def cubes_callback(self, msg):
        if self.goal_sent:
            return

        if self.target_cube_count is None:
            current_count = len(msg.cubes)

            if current_count == 0:
                self.count_candidate = None
                self.count_stable_frames = 0
                return

            if current_count == self.count_candidate:
                self.count_stable_frames += 1
            else:
                self.count_candidate = current_count
                self.count_stable_frames = 1

            self.get_logger().info(
                f'Broj detektovanih kockica: {current_count} '
                f'({self.count_stable_frames}/{self.COUNT_STABLE_REQUIRED})'
            )

            if self.count_stable_frames < self.COUNT_STABLE_REQUIRED:
                return

            self.target_cube_count = current_count

            self.get_logger().info('========================================')
            self.get_logger().info(f'UKUPAN BROJ KOCKICA ZAKLJUČAN: {self.target_cube_count}')
            self.get_logger().info(f'Move_to_cube će izvršiti najviše {self.target_cube_count} iteracija.')
            self.get_logger().info('Ovaj broj se više ne menja tokom sortiranja.')
            self.get_logger().info('========================================')

        if (
            self.target_cube_count is not None
            and len(self.completed_gazebo_ids) >= self.target_cube_count
        ):
            self.get_logger().info('========================================')
            self.get_logger().info(
                f'SVE KOCKICE SU SORTIRANE: {len(self.completed_gazebo_ids)}/{self.target_cube_count}'
            )
            self.get_logger().info('SORTIRANJE JE ZAVRŠENO.')
            self.get_logger().info('========================================')
            self.phase = 20
            return

        if self.candidate_selection_in_progress:
            return

        # REŠENJE ZA PROBLEM 2: Rukovanje slučajem kada se kockica ne vidi sa scan pozicije
        if len(msg.cubes) == 0:
            if (self.phase in (0, 16) or not self.goal_sent) and self.target_cube_count is not None:
                if len(self.completed_gazebo_ids) < self.target_cube_count:
                    self.empty_scan_count += 1
                    if self.empty_scan_count >= self.MAX_EMPTY_SCAN_FRAMES:
                        self.empty_scan_count = 0
                        if self.scan_state == 'PRIMARY':
                            self.get_logger().warn('========================================')
                            self.get_logger().warn('KOCKICA NIJE UOČENA SA PRIMARNE SCAN POZICIJE!')
                            self.get_logger().info('Pomeram robot na SEKUNDARNU scan poziciju...')
                            self.get_logger().info('========================================')
                            self.scan_state = 'SECONDARY'
                            self.move_to_secondary_scan()
                        elif self.scan_state == 'SECONDARY':
                            self.get_logger().error('========================================')
                            self.get_logger().error('Kockica nije pronađena ni sa SEKUNDARNE scan pozicije.')
                            self.get_logger().error('Sigurno završavam ciklus sortiranja bez rušenja programa.')
                            self.get_logger().error('========================================')
                            self.phase = 20
            return

        self.empty_scan_count = 0  # Pronađena kockica, resetujemo brojač praznih frame-ova

        cubes = []
        for cube in msg.cubes:
            x = cube.x
            y = cube.y
            z = cube.z
            distance = math.sqrt(x * x + y * y + z * z)

            cubes.append({
                'id': cube.id,
                'color': cube.color,
                'x': x,
                'y': y,
                'z': z,
                'distance': distance
            })

        cubes.sort(key=lambda c: c['distance'])

        self.selection_candidates = cubes
        self.selection_index = 0
        self.candidate_selection_in_progress = True

        self.get_logger().info('========================================')
        self.get_logger().info(
            f'PROVERA KANDIDATA ZA SLEDEĆU KOCKICU ({len(cubes)} kandidata)'
        )
        self.get_logger().info(
            f'Već završene Gazebo kockice: {sorted(self.completed_gazebo_ids)}'
        )
        self.get_logger().info('========================================')

        self.probe_next_candidate()

    # =============================================================
    # PROVERA KANDIDATA PRE MOVEIT-A
    # =============================================================

    def probe_next_candidate(self):
        if self.selection_index >= len(self.selection_candidates):
            self.candidate_selection_in_progress = False
            self.get_logger().error(
                'Nijedan detektovani kandidat nije mogao da se poveže sa nezavršenom Gazebo kockicom.'
            )
            if self.scan_state == 'PRIMARY':
                self.get_logger().warn('Pokušavam prelazak na sekundarnu scan poziciju...')
                self.scan_state = 'SECONDARY'
                self.move_to_secondary_scan()
            else:
                self.get_logger().error('Ne šaljem MoveIt prema neproverenoj kockici.')
                self.phase = 20
            return

        candidate = self.selection_candidates[self.selection_index]

        self.get_logger().info('----------------------------------------')
        self.get_logger().info(
            f'PROVERA KANDIDATA {self.selection_index + 1}/{len(self.selection_candidates)}'
        )
        self.get_logger().info(f'Detector ID: {candidate["id"]}')
        self.get_logger().info(f'Boja: {candidate["color"]}')
        self.get_logger().info(
            f'LINK_BASE XYZ: ({candidate["x"]:.4f}, {candidate["y"]:.4f}, {candidate["z"]:.4f})'
        )
        self.get_logger().info(f'Udaljenost: {candidate["distance"]:.4f} m')

        if not self.closest_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('/get_closest_cube nije dostupan tokom izbora kandidata.')
            self.selection_index += 1
            self.probe_next_candidate()
            return

        request = GetClosestCube.Request()
        request.target_pose = Pose()
        request.target_pose.position.x = candidate['x']
        request.target_pose.position.y = candidate['y']
        request.target_pose.position.z = candidate['z']
        request.target_pose.orientation.w = 1.0

        future = self.closest_client.call_async(request)
        future.add_done_callback(self.candidate_probe_result_callback)

    def candidate_probe_result_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'Greška pri proveri kandidata: {e}')
            self.selection_index += 1
            self.probe_next_candidate()
            return

        candidate = self.selection_candidates[self.selection_index]

        if not response.found:
            self.get_logger().warn('Kandidat nije potvrđen preko /get_closest_cube.')

            if self.target_cube_count is not None:
                remaining_ids = []
                for i in range(self.target_cube_count):
                    gazebo_id_candidate = f'cube_{i}'
                    if gazebo_id_candidate not in self.completed_gazebo_ids:
                        remaining_ids.append(gazebo_id_candidate)

                if len(remaining_ids) == 1 and len(self.selection_candidates) == 1:
                    gazebo_id = remaining_ids[0]

                    self.get_logger().warn('----------------------------------------')
                    self.get_logger().warn('FALLBACK ZA POSLEDNJU KOCKICU')
                    self.get_logger().warn(
                        '/get_closest_cube nije potvrdio kockicu, ali je ostala samo jedna nezavršena Gazebo kockica.'
                    )
                    self.get_logger().warn(f'Jedina preostala Gazebo kockica: {gazebo_id}')
                    self.get_logger().warn('Koristim ovaj ID za nastavak sortiranja.')
                    self.get_logger().warn('----------------------------------------')

                    self.candidate_selection_in_progress = False
                    self.goal_sent = True

                    self.detected_id = candidate['id']
                    self.detected_color = candidate['color']
                    self.cube_x = candidate['x']
                    self.cube_y = candidate['y']
                    self.cube_z = candidate['z']
                    self.real_cube_id = None
                    self.expected_gazebo_id = gazebo_id

                    self.get_logger().info('========================================')
                    self.get_logger().info('IZABRANA NOVA KOCKICA')
                    self.get_logger().info(f'Detector ID (trenutni frame): {self.detected_id}')
                    self.get_logger().info(f'Gazebo ID (fallback): {gazebo_id}')
                    self.get_logger().info(f'Boja: {self.detected_color}')
                    self.get_logger().info(
                        f'LINK_BASE XYZ: ({self.cube_x:.4f}, {self.cube_y:.4f}, {self.cube_z:.4f})'
                    )
                    self.get_logger().info('MoveIt sada prilazi poslednjoj preostaloj kockici.')
                    self.get_logger().info('========================================')

                    current_tcp = self.get_current_tcp_pose()
                    if current_tcp is None:
                        self.get_logger().error('Ne mogu da odredim trenutni TCP za FAZU 1A.')
                        self.goal_sent = False
                        self.candidate_selection_in_progress = False
                        return

                    current_z = current_tcp.position.z
                    horizontal_z = max(current_z, self.cube_z + self.ABOVE_CUBE)

                    self.phase = 1
                    self.move_retry_count = 0
                    self.approach_retry_count = 0
                    self.approach_step = 0

                    self.get_logger().info('FAZA 1A - HORIZONTALNI XY PRILAZ')
                    self.get_logger().info(f'START TCP Z: {current_z:.4f}')
                    self.get_logger().info(
                        f'Target LINK_BASE: ({self.cube_x:.4f}, {self.cube_y:.4f}, {horizontal_z:.4f})'
                    )
                    self.get_logger().info('X/Y se menjaju prema kockici; Z ostaje na bezbednoj visini.')

                    self.send_move_goal(self.cube_x, self.cube_y, horizontal_z)
                    return

            self.selection_index += 1
            self.probe_next_candidate()
            return

        gazebo_id = response.cube_id
        self.get_logger().info(f'Kandidat odgovara Gazebo kockici: {gazebo_id}')

        if gazebo_id in self.completed_gazebo_ids:
            self.get_logger().warn(f'{gazebo_id} je VEĆ ZAVRŠENA. Preskačem ovaj detector kandidat.')
            self.selection_index += 1
            self.probe_next_candidate()
            return

        self.candidate_selection_in_progress = False
        self.goal_sent = True

        self.detected_id = candidate['id']
        self.detected_color = candidate['color']
        self.cube_x = candidate['x']
        self.cube_y = candidate['y']
        self.cube_z = candidate['z']
        self.real_cube_id = None
        self.expected_gazebo_id = gazebo_id

        self.get_logger().info('========================================')
        self.get_logger().info('IZABRANA NOVA KOCKICA')
        self.get_logger().info(f'Detector ID (trenutni frame): {self.detected_id}')
        self.get_logger().info(f'Gazebo ID (trajni identitet): {gazebo_id}')
        self.get_logger().info(f'Boja: {self.detected_color}')
        self.get_logger().info(
            f'LINK_BASE XYZ: ({self.cube_x:.4f}, {self.cube_y:.4f}, {self.cube_z:.4f})'
        )
        self.get_logger().info('MoveIt sada prilazi provereno izabranoj kockici.')
        self.get_logger().info('========================================')

        current_tcp = self.get_current_tcp_pose()
        if current_tcp is None:
            self.get_logger().error('Ne mogu da odredim trenutni TCP za FAZU 1A.')
            self.goal_sent = False
            self.candidate_selection_in_progress = False
            return

        current_z = current_tcp.position.z
        horizontal_z = max(current_z, self.cube_z + self.ABOVE_CUBE)

        self.phase = 1
        self.move_retry_count = 0
        self.approach_retry_count = 0
        self.approach_step = 0

        self.get_logger().info('FAZA 1A - HORIZONTALNI XY PRILAZ')
        self.get_logger().info(f'START TCP Z: {current_z:.4f}')
        self.get_logger().info(
            f'Target LINK_BASE: ({self.cube_x:.4f}, {self.cube_y:.4f}, {horizontal_z:.4f})'
        )
        self.get_logger().info('X/Y se menjaju prema kockici; Z ostaje na bezbednoj visini.')

        self.send_move_goal(self.cube_x, self.cube_y, horizontal_z)

    # =============================================================
    # MOVEIT CILJ
    # =============================================================

    def send_move_goal(self, x, y, z):
        self.last_move_target = (x, y, z)

        self.get_logger().info('Cekam MoveIt /move_action...')

        if not self.move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveIt /move_action nije dostupan!')
            return

        goal = MoveGroup.Goal()
        goal.request.group_name = 'xarm7'
        goal.request.max_velocity_scaling_factor = 0.25
        goal.request.max_acceleration_scaling_factor = 0.25
        goal.request.allowed_planning_time = 5.0

        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        position = PositionConstraint()
        position.header.frame_id = 'link_base'
        position.link_name = 'link_tcp'

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.015]

        position.constraint_region.primitives.append(primitive)

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        position.constraint_region.primitive_poses.append(pose)
        position.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header.frame_id = 'link_base'
        orientation.link_name = 'link_tcp'
        orientation.orientation.x = 1.0
        orientation.orientation.y = 0.0
        orientation.orientation.z = 0.0
        orientation.orientation.w = 0.0

        orientation.absolute_x_axis_tolerance = 0.15
        orientation.absolute_y_axis_tolerance = 0.15
        orientation.absolute_z_axis_tolerance = 0.15
        orientation.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position)
        constraints.orientation_constraints.append(orientation)
        goal.request.goal_constraints.append(constraints)

        self.get_logger().info(f'Saljem cilj: ({x:.4f}, {y:.4f}, {z:.4f})')

        future = self.move_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    # =============================================================
    # MOVEIT ACCEPT
    # =============================================================

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Greska pri prihvatanju MoveIt cilja: {e}')
            return

        if not goal_handle.accepted:
            self.get_logger().error('MoveIt JE ODBIO CILJ!')
            return

        self.get_logger().info('Cilj prihvacen.')
        self.get_logger().info('Robot se pomera...')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    # =============================================================
    # MOVEIT RESULT
    # =============================================================

    def result_callback(self, future):
        try:
            result = future.result().result
            error_code = result.error_code.val
        except Exception as e:
            self.get_logger().error(f'Greska pri citanju MoveIt rezultata: {e}')
            return

        self.get_logger().info('========================================')
        self.get_logger().info(f'MoveIt error_code = {error_code}')

        if error_code == -13:
            self.get_logger().warn('MoveIt: GOAL_VIOLATES_PATH_CONSTRAINTS')
        elif error_code == -12:
            self.get_logger().warn('MoveIt: GOAL_IN_COLLISION')
        elif error_code == -4:
            self.get_logger().warn('MoveIt: CONTROL_FAILED')

        self.get_logger().info('========================================')

        if error_code != 1:
            if self.phase == 2 and self.approach_retry_count < 1:
                self.approach_retry_count += 1
                if self.approach_step == 0:
                    retry_z = self.cube_z + self.APPROACH_SAFE_HEIGHT + 0.005
                    retry_description = 'bezbednoj visini FAZE 2'
                else:
                    retry_z = self.cube_z + self.APPROACH_OFFSET + 0.005
                    retry_description = 'završnom prilazu FAZE 2'

                self.get_logger().warn(f'MoveIt nije uspeo na {retry_description}. Pokušavam ponovo 5 mm VIŠE.')
                self.send_move_goal(self.cube_x, self.cube_y, retry_z)
                return

            if (
                self.phase in (1, 4, 5, 7, 9, 11)
                and self.move_retry_count < 1
                and self.last_move_target is not None
            ):
                self.move_retry_count += 1
                self.get_logger().warn(
                    f'MoveIt nije uspeo u FAZI {self.phase} (error_code = {error_code}). '
                    f'Pokušavam ponovo ISTI cilj (pokušaj {self.move_retry_count}/1)...'
                )
                time.sleep(0.5)
                self.send_move_goal(*self.last_move_target)
                return

            self.get_logger().error(f'MoveIt nije uspeo. Faza = {self.phase}')
            if self.phase == 13:
                self.get_logger().error('DROP NIJE USPEO ni nakon retry-ja.')

            return

        self.move_retry_count = 0

        if self.phase == 13 and self.drop_step == 'move_to_slot':
            self.get_logger().info('========================================')
            self.get_logger().info('ROBOT JE STIGAO NA SLOT SAFE POZICIJU PREKO FALLBACK PLANERA.')
            self.drop_step = 'vertical_cartesian'
            self.send_cartesian_drop()
            return

        if self.phase == 1:
            self.get_logger().info('ROBOT JE STIGAO IZNAD KOCKICE - XY PRILAZ ZAVRSEN.')
            self.phase = 2
            self.approach_retry_count = 0
            self.approach_step = 0

            safe_z = self.cube_z + self.APPROACH_SAFE_HEIGHT

            self.get_logger().info('----------------------------------------')
            self.get_logger().info('FAZA 1B - VERTIKALNO SPUSTANJE DO SAFE VISINE')
            self.get_logger().info(f'Target: ({self.cube_x:.4f}, {self.cube_y:.4f}, {safe_z:.4f})')
            self.get_logger().info('X/Y ostaju konstantni; sada se menja samo Z.')

            self.send_move_goal(self.cube_x, self.cube_y, safe_z)
            return

        if self.phase == 2:
            if self.approach_step == 0:
                self.approach_step = 1
                self.approach_retry_count = 0

                final_z = self.cube_z + self.APPROACH_OFFSET

                self.get_logger().info('----------------------------------------')
                self.get_logger().info('FAZA 2 - KORAK 2/2: ZAVRŠNI PRILAZ KOCKICI')
                self.get_logger().info(f'Target: ({self.cube_x:.4f}, {self.cube_y:.4f}, {final_z:.4f})')
                self.get_logger().info('X/Y ostaju na centru kockice; sada se spušta samo prema Z cilju.')

                self.send_move_goal(self.cube_x, self.cube_y, final_z)
                return

            self.get_logger().info('ROBOT JE STIGAO BLIZU KOCKICE.')
            self.phase = 3
            self.check_tcp_position()
            return

        if self.phase == 5:
            self.get_logger().info('========================================')
            self.get_logger().info('KOCKICA JE USPESNO PODIGNUTA!')
            self.get_logger().info(f'Gazebo cube: {self.real_cube_id}')
            self.get_logger().info(f'Boja kockice: {self.detected_color}')
            self.get_logger().info('========================================')

            self.go_to_surface('left', self.LEFT_SURFACE_WORLD)
            return

        if self.phase in (7, 9, 11):
            self.get_logger().info('========================================')
            self.get_logger().info(f'ROBOT JE STIGAO IZNAD {self.current_surface_name.upper()} POVRŠINE.')
            self.log_current_joints('JOINT KONFIGURACIJA NEPOSREDNO PRE FAZE 13')
            self.get_logger().info('SADA POZIVAM /detect_surface.')
            self.get_logger().info('========================================')

            self.request_surface_detection()
            return

        if self.phase == 13:
            self.get_logger().info('FAZA 13 je obrađena kroz Cartesian DROP.')
            return

        if self.phase == 14:
            self.get_logger().info('========================================')
            self.get_logger().info('KOCKICA JE ODVOJENA OD TCP-a.')
            self.get_logger().info('VRAĆAM ROBOTA NA AUTO_SCAN POZICIJU.')
            self.get_logger().info('========================================')

            self.phase = 15
            self.return_to_auto_scan()
            return

        if self.phase == 15:
            self.get_logger().info('========================================')
            self.get_logger().info('ROBOT SE VRATIO NA AUTO_SCAN POZICIJU.')
            self.get_logger().info('SPREMAN ZA SLEDEĆI KORAK.')
            self.get_logger().info('========================================')

            self.phase = 16
            return

    # =============================================================
    # TRENUTNI TCP - LINK_BASE FRAME
    # =============================================================

    def get_current_tcp_pose(self):
        self.get_logger().info('Cekam link_base -> link_tcp TF za FAZU 1...')

        for _ in range(100):
            try:
                if self.tf_buffer.can_transform(
                    'link_base', 'link_tcp', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.05)
                ):
                    transform = self.tf_buffer.lookup_transform('link_base', 'link_tcp', rclpy.time.Time())

                    pose = Pose()
                    pose.position.x = transform.transform.translation.x
                    pose.position.y = transform.transform.translation.y
                    pose.position.z = transform.transform.translation.z
                    pose.orientation.x = transform.transform.rotation.x
                    pose.orientation.y = transform.transform.rotation.y
                    pose.orientation.z = transform.transform.rotation.z
                    pose.orientation.w = transform.transform.rotation.w

                    return pose
            except Exception:
                pass

            rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().error('Nisam uspeo da procitam link_base -> link_tcp TF za FAZU 1.')
        return None

    # =============================================================
    # CARTESIAN DROP - STVARNO STANJE ROBOTA
    # =============================================================

    def get_current_tcp_pose_for_cartesian(self):
        self.get_logger().info('Cekam link_base -> link_tcp TF za Cartesian DROP...')

        for _ in range(200):
            try:
                if self.tf_buffer.can_transform(
                    'link_base', 'link_tcp', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.05)
                ):
                    transform = self.tf_buffer.lookup_transform('link_base', 'link_tcp', rclpy.time.Time())

                    pose = Pose()
                    pose.position.x = transform.transform.translation.x
                    pose.position.y = transform.transform.translation.y
                    pose.position.z = transform.transform.translation.z
                    pose.orientation.x = transform.transform.rotation.x
                    pose.orientation.y = transform.transform.rotation.y
                    pose.orientation.z = transform.transform.rotation.z
                    pose.orientation.w = transform.transform.rotation.w

                    return pose
            except Exception:
                pass

            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().error('Nisam uspeo da pročitam link_base -> link_tcp TF za DROP.')
        return None

    def build_cartesian_start_state(self):
        if not self.latest_joint_positions:
            self.get_logger().error('/joint_states nema podatke za Cartesian DROP.')
            return None

        state = RobotState()
        state.joint_state.name = list(self.latest_joint_positions.keys())
        state.joint_state.position = list(self.latest_joint_positions.values())
        state.is_diff = False

        return state

    def make_drop_waypoint(self, reference_pose, x, y, z):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation = reference_pose.orientation

        return pose

    def _slow_down_trajectory(self, trajectory):
        points = trajectory.joint_trajectory.points
        if not points:
            return

        original_times = []
        for point in points:
            original_times.append(
                point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            )

        if not original_times:
            return

        original_duration = original_times[-1]
        scale = max(1.0, self.DROP_TIME_SCALE)

        if original_duration > 0.0:
            minimum_duration = 1.0
            if original_duration < minimum_duration:
                scale = max(scale, minimum_duration / original_duration)

        for point, original_time in zip(points, original_times):
            new_time = original_time * scale
            sec = int(new_time)
            nanosec = int(round((new_time - sec) * 1e9))

            if nanosec >= 1_000_000_000:
                sec += 1
                nanosec -= 1_000_000_000

            point.time_from_start.sec = sec
            point.time_from_start.nanosec = nanosec

            for i in range(len(point.velocities)):
                point.velocities[i] /= scale

            acceleration_scale = scale * scale
            for i in range(len(point.accelerations)):
                point.accelerations[i] /= acceleration_scale

        self.get_logger().info(
            f'DROP trajektorija usporena x{scale:.2f}; originalno trajanje = {original_duration:.3f} s, '
            f'novo trajanje = {original_duration * scale:.3f} s.'
        )

    def _make_vertical_drop_waypoints(self, start_pose, surface_z, release_z):
        prepare_z = max(release_z, surface_z + self.DROP_PREPARE_CLEARANCE)

        prepare_pose = self.make_drop_waypoint(
            start_pose, start_pose.position.x, start_pose.position.y, prepare_z
        )

        release_pose = self.make_drop_waypoint(
            start_pose, start_pose.position.x, start_pose.position.y, release_z
        )

        return [start_pose, prepare_pose, release_pose], prepare_z

    def send_cartesian_drop(self):
        self.get_logger().info('========================================')
        self.get_logger().info('FAZA 13 - BEZBEDAN CARTESIAN DROP')

        if not self.cartesian_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/compute_cartesian_path nije dostupan.')
            return

        if not self.execute_trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('/execute_trajectory nije dostupan.')
            return

        start_pose = self.get_current_tcp_pose_for_cartesian()
        if start_pose is None:
            return

        start_state = self.build_cartesian_start_state()
        if start_state is None:
            return

        if self.current_drop_world is None:
            self.get_logger().error('Nema current_drop_world za Cartesian DROP.')
            return

        drop_base_x, drop_base_y, requested_drop_z = self.world_to_base(
            self.current_drop_world[0],
            self.current_drop_world[1],
            self.current_drop_world[2]
        )

        surface_world_z = self.current_surface_world[2]
        release_world_z = max(
            requested_drop_z, surface_world_z + self.DROP_MIN_TCP_CLEARANCE
        )

        _, _, release_base_z = self.world_to_base(
            self.current_drop_world[0], self.current_drop_world[1], release_world_z
        )

        surface_base_z = self.world_to_base(0.0, 0.0, surface_world_z)[2]

        waypoints, prepare_z = self._make_vertical_drop_waypoints(
            start_pose, surface_base_z, release_base_z
        )

        minimum_base_z = surface_world_z - self.ROBOT_BASE_Z + self.DROP_MIN_TCP_CLEARANCE

        if release_base_z < minimum_base_z:
            self.get_logger().error('DROP SAFETY CHECK: finalni TCP je prenizak!')
            return

        request = GetCartesianPath.Request()
        request.header.frame_id = 'link_base'
        request.start_state = start_state
        request.group_name = 'xarm7'
        request.link_name = 'link_tcp'
        request.waypoints = waypoints
        request.max_step = 0.005
        request.jump_threshold = 0.0
        request.prismatic_jump_threshold = 0.0
        request.revolute_jump_threshold = 0.0
        request.avoid_collisions = True
        request.max_cartesian_speed = self.DROP_MAX_TCP_SPEED
        request.cartesian_speed_limited_link = 'link_tcp'

        future = self.cartesian_client.call_async(request)

        def cartesian_result_callback(f):
            try:
                response = f.result()
            except Exception as e:
                self.get_logger().error(f'Greška pri /compute_cartesian_path: {e}')
                return

            if response.fraction < 0.99 or response.error_code.val != 1:
                self.get_logger().error('Bezbedni Cartesian DROP nije uspešno izračunat.')
                return

            trajectory = response.solution
            if not trajectory.joint_trajectory.points:
                self.get_logger().error('Cartesian DROP trajektorija nema tačke.')
                return

            self._slow_down_trajectory(trajectory)

            goal = ExecuteTrajectory.Goal()
            goal.trajectory = trajectory
            goal.controller_names = []

            execute_future = self.execute_trajectory_client.send_goal_async(goal)

            def execute_goal_callback(ef):
                try:
                    goal_handle = ef.result()
                except Exception as e:
                    self.get_logger().error(f'Greška pri slanju ExecuteTrajectory: {e}')
                    return

                if not goal_handle.accepted:
                    self.get_logger().error('ExecuteTrajectory je ODBIO bezbedni DROP.')
                    return

                result_future = goal_handle.get_result_async()

                def execute_result_callback(rf):
                    try:
                        result = rf.result().result
                        error_code = result.error_code.val
                    except Exception as e:
                        self.get_logger().error(f'Greška pri čitanju ExecuteTrajectory rezultata: {e}')
                        return

                    if error_code != 1:
                        self.get_logger().error('Bezbedni Cartesian DROP nije izvršen.')
                        return

                    self.get_logger().info('FAZA 13 - BEZBEDNI CARTESIAN DROP USPEŠAN.')
                    time.sleep(self.DROP_SETTLE_TIME)
                    self.detach_cube()

                result_future.add_done_callback(execute_result_callback)

            execute_future.add_done_callback(execute_goal_callback)

        future.add_done_callback(cartesian_result_callback)

    # =============================================================
    # TCP POZICIJA
    # =============================================================

    def check_tcp_position(self):
        self.get_logger().info('----------------------------------------')
        self.get_logger().info('PROVERA TCP POZICIJE')

        try:
            transform = self.tf_buffer.lookup_transform(
                'link_base', 'link_tcp', rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().error(f'TF nije dostupan: {e}')
            return

        tcp_x = transform.transform.translation.x
        tcp_y = transform.transform.translation.y
        tcp_z = transform.transform.translation.z

        self.get_logger().info(f'TCP LINK_BASE: ({tcp_x:.4f}, {tcp_y:.4f}, {tcp_z:.4f})')

        world_x, world_y, world_z = self.base_to_world(tcp_x, tcp_y, tcp_z)

        self.get_logger().info(f'TCP WORLD: ({world_x:.4f}, {world_y:.4f}, {world_z:.4f})')

        self.service_check_count = 0
        self.service_check_results = []

        self.check_closest_cube(tcp_x, tcp_y, tcp_z)

    # =============================================================
    # GET CLOSEST CUBE
    # =============================================================

    def check_closest_cube(self, base_x, base_y, base_z):
        if not self.closest_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('/get_closest_cube nije dostupan!')
            return

        request = GetClosestCube.Request()
        request.target_pose = Pose()
        request.target_pose.position.x = base_x
        request.target_pose.position.y = base_y
        request.target_pose.position.z = base_z
        request.target_pose.orientation.w = 1.0

        self.get_logger().info(f'Service provera {self.service_check_count + 1}/3')
        self.get_logger().info(f'Target LINK_BASE: ({base_x:.4f}, {base_y:.4f}, {base_z:.4f})')

        future = self.closest_client.call_async(request)
        future.add_done_callback(
            lambda f: self.service_result_callback(f, base_x, base_y, base_z)
        )

    # =============================================================
    # SERVICE RESULT (PROBLEM 1 REŠENJE INTEGRISANO)
    # =============================================================

    def service_result_callback(self, future, base_x, base_y, base_z):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'Greska service-a: {e}')
            return

        self.service_check_count += 1

        if response.found:
            self.get_logger().info(f'Service rezultat: {response.cube_id}')
            self.service_check_results.append(response.cube_id)
        else:
            self.get_logger().warn('Service nije pronasao kockicu!')
            self.service_check_results.append(None)

        if self.service_check_count < 3:
            time.sleep(0.15)
            self.check_closest_cube(base_x, base_y, base_z)
            return

        self.get_logger().info('----------------------------------------')
        self.get_logger().info('REZULTATI 3 SERVICE PROVERE')

        for i, result in enumerate(self.service_check_results, start=1):
            self.get_logger().info(f'  {i}: {result}')

        valid_results = [r for r in self.service_check_results if r is not None]

        # FALLBACK AKO SERVIS NIJE POTVRDIO IZBLIZA (PROBLEM 1 REŠENJE)
        if len(valid_results) == 0:
            self.get_logger().warn('SERVICE NIJEDNOM NIJE PRONASAO KOCKICU IZBLIZA (moguća senka ili prebliza kamera).')
            if self.expected_gazebo_id is not None:
                self.get_logger().warn(
                    f'FALLBACK AKTIVIRAN: Preuzimam originalni Gazebo ID iz primarnog skeniranja: {self.expected_gazebo_id}'
                )
                self.real_cube_id = self.expected_gazebo_id
                self.get_logger().info('========================================')
                self.get_logger().info('KOCKICA POTVRDJENA VIA FALLBACK!')
                self.get_logger().info(f'Gazebo ID = {self.real_cube_id}')
                self.get_logger().info(f'Detector ID = {self.detected_id}')
                self.get_logger().info(f'Boja = {self.detected_color}')
                self.get_logger().info('========================================')
                self.attach_cube()
                return
            else:
                self.get_logger().error('ATTACH SE NEĆE IZVRŠITI (Nema očekivanog ID-a).')
                return

        if len(set(valid_results)) != 1:
            self.get_logger().warn('SERVICE JE VRATIO RAZLICITE KOCKICE!')
            if self.expected_gazebo_id is not None:
                self.get_logger().warn(f'FALLBACK: Koristim primarno verifikovani Gazebo ID: {self.expected_gazebo_id}')
                self.real_cube_id = self.expected_gazebo_id
            else:
                self.real_cube_id = valid_results[0]
        else:
            self.real_cube_id = valid_results[0]

        if (
            self.expected_gazebo_id is not None
            and self.real_cube_id != self.expected_gazebo_id
        ):
            self.get_logger().error(
                f'PAŽNJA: Gazebo ID na stvarnoj poziciji ({self.real_cube_id}) se razlikuje '
                f'od proverenog kandidata ({self.expected_gazebo_id}).'
            )
            self.get_logger().error('ATTACH SE NEĆE IZVRŠITI.')
            self.goal_sent = False
            self.phase = 16
            self.expected_gazebo_id = None
            return

        self.get_logger().info('========================================')
        self.get_logger().info('KOCKICA POTVRDJENA!')
        self.get_logger().info(f'Gazebo ID = {self.real_cube_id}')
        self.get_logger().info(f'Detector ID = {self.detected_id}')
        self.get_logger().info(f'Boja = {self.detected_color}')
        self.get_logger().info('========================================')

        self.attach_cube()

    # =============================================================
    # ATTACH
    # =============================================================

    def attach_cube(self):
        if self.real_cube_id is None:
            self.get_logger().error('Nema validnog Gazebo cube ID-a!')
            return

        publisher = self.attach_publishers.get(self.real_cube_id)

        if publisher is None:
            self.get_logger().error(f'Nema attach publishera za {self.real_cube_id}!')
            return

        self.get_logger().info('========================================')
        self.get_logger().info('ATTACH KOCKICE')
        self.get_logger().info(f'Cube: {self.real_cube_id}')

        msg = Empty()
        publisher.publish(msg)

        self.get_logger().info('ATTACH PORUKA POSLATA!')

        self.phase = 5
        self.move_retry_count = 0
        lift_z = self.cube_z + self.LIFT_HEIGHT

        self.get_logger().info('FAZA 5 - PODIZANJE KOCKICE')
        self.send_move_goal(self.cube_x, self.cube_y, lift_z)

    # =============================================================
    # ODLASAK NA POVRŠINU
    # =============================================================

    def go_to_surface(self, surface_name, world_position):
        self.current_surface_name = surface_name
        self.current_surface_world = world_position
        self.detected_surface_color = None

        world_x = world_position[0]
        world_y = world_position[1]
        world_z = world_position[2] + self.SURFACE_HEIGHT_OFFSET

        base_x, base_y, base_z = self.world_to_base(world_x, world_y, world_z)

        if surface_name == 'left':
            self.phase = 7
        elif surface_name == 'middle':
            self.phase = 9
        elif surface_name == 'right':
            self.phase = 11

        self.move_retry_count = 0
        self.send_move_goal(base_x, base_y, base_z)

    # =============================================================
    # DETEKCIJA POVRŠINE
    # =============================================================

    def request_surface_detection(self):
        if self.surface_service_pending:
            return

        if not self.surface_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('/detect_surface nije dostupan!')
            return

        request = DetectSurface.Request()
        self.surface_service_pending = True

        future = self.surface_client.call_async(request)
        future.add_done_callback(self.surface_service_result_callback)

    def surface_service_result_callback(self, future):
        self.surface_service_pending = False

        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'Greška pri /detect_surface: {e}')
            return

        if not response.success:
            self.try_next_surface()
            return

        self.detected_surface_color = response.color.strip().upper()
        cube_color = self.detected_color.strip().upper()
        surface_color = self.detected_surface_color

        if cube_color == surface_color:
            self.go_to_drop_position()
            return

        self.try_next_surface()

    def try_next_surface(self):
        if self.current_surface_name == 'left':
            self.go_to_surface('middle', self.MIDDLE_SURFACE_WORLD)
            return

        if self.current_surface_name == 'middle':
            self.go_to_surface('right', self.RIGHT_SURFACE_WORLD)
            return

        if self.current_surface_name == 'right':
            self.get_logger().error('Nijedna površina ne odgovara boji kockice.')
            self.phase = 20
            return

    def go_to_drop_position(self):
        if self.current_surface_world is None:
            return

        color = self.detected_color.strip().upper()
        placed_count = self.placed_counts.get(color, 0)

        slot_index = min(placed_count, len(self.SURFACE_SLOT_OFFSETS) - 1)
        offset_x, offset_y = self.SURFACE_SLOT_OFFSETS[slot_index]

        self.current_drop_offset = (offset_x, offset_y)

        world_x = self.current_surface_world[0] + offset_x
        world_y = self.current_surface_world[1] + offset_y

        requested_drop_z = self.current_surface_world[2] + self.DROP_TCP_OFFSET
        minimum_safe_drop_z = self.current_surface_world[2] + self.DROP_MIN_TCP_CLEARANCE
        world_z = max(requested_drop_z, minimum_safe_drop_z)

        self.current_drop_world = (world_x, world_y, world_z)

        base_x, base_y, base_z = self.world_to_base(world_x, world_y, world_z)

        self.phase = 13
        self.drop_retry_count = 0

        current_pose = self.get_current_tcp_pose_for_cartesian()
        if current_pose is None:
            return

        self.drop_step = 'horizontal_cartesian'
        self.send_horizontal_cartesian_drop(current_pose, base_x, base_y)

    def send_horizontal_cartesian_drop(self, start_pose, target_x, target_y):
        if not self.cartesian_client.wait_for_service(timeout_sec=5.0):
            return

        if not self.execute_trajectory_client.wait_for_server(timeout_sec=5.0):
            return

        start_state = self.build_cartesian_start_state()
        if start_state is None:
            return

        target_pose = self.make_drop_waypoint(
            start_pose, target_x, target_y, start_pose.position.z
        )

        request = GetCartesianPath.Request()
        request.header.frame_id = 'link_base'
        request.start_state = start_state
        request.group_name = 'xarm7'
        request.link_name = 'link_tcp'
        request.waypoints = [target_pose]
        request.max_step = 0.005
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        request.max_cartesian_speed = 0.04
        request.cartesian_speed_limited_link = 'link_tcp'

        future = self.cartesian_client.call_async(request)

        def result_callback(f):
            try:
                response = f.result()
            except Exception as e:
                return

            if response.fraction < 0.99 or response.error_code.val != 1:
                self.drop_step = 'move_to_slot'
                self.send_drop_move_goal(target_x, target_y, start_pose.position.z)
                return

            trajectory = response.solution
            if not trajectory.joint_trajectory.points:
                return

            goal = ExecuteTrajectory.Goal()
            goal.trajectory = trajectory

            execute_future = self.execute_trajectory_client.send_goal_async(goal)

            def goal_callback(ef):
                try:
                    goal_handle = ef.result()
                except Exception:
                    return

                if not goal_handle.accepted:
                    self.drop_step = 'move_to_slot'
                    self.send_drop_move_goal(target_x, target_y, start_pose.position.z)
                    return

                result_future = goal_handle.get_result_async()

                def result_done(rf):
                    try:
                        result = rf.result().result
                        code = result.error_code.val
                    except Exception:
                        return

                    if code != 1:
                        self.drop_step = 'move_to_slot'
                        self.send_drop_move_goal(target_x, target_y, start_pose.position.z)
                        return

                    self.drop_step = 'vertical_cartesian'
                    self.send_cartesian_drop()

                result_future.add_done_callback(result_done)

            execute_future.add_done_callback(goal_callback)

        future.add_done_callback(result_callback)

    def send_drop_move_goal(self, x, y, z):
        if not self.move_client.wait_for_server(timeout_sec=5.0):
            return

        goal = MoveGroup.Goal()
        goal.request.group_name = 'xarm7'
        goal.request.max_velocity_scaling_factor = 0.15
        goal.request.max_acceleration_scaling_factor = 0.15
        goal.request.allowed_planning_time = self.DROP_PLANNING_TIME

        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5

        position = PositionConstraint()
        position.header.frame_id = 'link_base'
        position.link_name = 'link_tcp'

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.015]

        position.constraint_region.primitives.append(primitive)

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        position.constraint_region.primitive_poses.append(pose)
        position.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header.frame_id = 'link_base'
        orientation.link_name = 'link_tcp'
        orientation.orientation.x = 1.0
        orientation.orientation.w = 0.0

        orientation.absolute_x_axis_tolerance = self.DROP_ORIENTATION_TOLERANCE
        orientation.absolute_y_axis_tolerance = self.DROP_ORIENTATION_TOLERANCE
        orientation.absolute_z_axis_tolerance = self.DROP_ORIENTATION_TOLERANCE
        orientation.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position)
        constraints.orientation_constraints.append(orientation)
        goal.request.goal_constraints.append(constraints)

        future = self.move_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    # =============================================================
    # SEKUNDARNA SCAN POZICIJA (PROBLEM 2 IMPLEMENTACIJA)
    # =============================================================

    def move_to_secondary_scan(self):
        self.get_logger().info('========================================')
        self.get_logger().info('POKRET NA SEKUNDARNU SCAN POZICIJU')
        self.get_logger().info('========================================')

        if not self.move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveIt /move_action nije dostupan!')
            return

        goal = MoveGroup.Goal()
        goal.request.group_name = 'xarm7'
        goal.request.max_velocity_scaling_factor = 0.25
        goal.request.max_acceleration_scaling_factor = 0.25
        goal.request.allowed_planning_time = 5.0

        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        constraints = Constraints()
        joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7'
        ]

        for name, position in zip(joint_names, self.SECONDARY_SCAN_POSITION):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = position
            constraint.tolerance_above = 0.01
            constraint.tolerance_below = 0.01
            constraint.weight = 1.0

            constraints.joint_constraints.append(constraint)

        goal.request.goal_constraints.append(constraints)

        future = self.move_client.send_goal_async(goal)
        future.add_done_callback(self.secondary_scan_goal_response_callback)

    def secondary_scan_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Greška pri slanju sekundarnog scan cilja: {e}')
            return

        if not goal_handle.accepted:
            self.get_logger().error('MoveIt JE ODBIO POKRET NA SEKUNDARNI SCAN!')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.secondary_scan_result_callback)

    def secondary_scan_result_callback(self, future):
        try:
            result = future.result().result
            error_code = result.error_code.val
        except Exception as e:
            self.get_logger().error(f'Greška pri čitanju rezultata sekundarnog scana: {e}')
            return

        if error_code != 1:
            self.get_logger().error('Robot NIJE uspeo da stigne na SEKUNDARNU scan poziciju.')
            return

        self.get_logger().info('ROBOT JE NA SEKUNDARNOJ SCAN POZICIJI.')
        self.goal_sent = False
        self.candidate_selection_in_progress = False

    # =============================================================
    # POVRATAK NA AUTO_SCAN POZICIJU
    # =============================================================

    def return_to_auto_scan(self):
        self.get_logger().info('========================================')
        self.get_logger().info('POVRATAK NA PRIMARNU AUTO_SCAN POZICIJU')
        self.get_logger().info('========================================')

        if not self.move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveIt /move_action nije dostupan!')
            return

        goal = MoveGroup.Goal()
        goal.request.group_name = 'xarm7'
        goal.request.max_velocity_scaling_factor = 0.25
        goal.request.max_acceleration_scaling_factor = 0.25
        goal.request.allowed_planning_time = 5.0

        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        constraints = Constraints()
        joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7'
        ]

        for name, position in zip(joint_names, self.AUTO_SCAN_POSITION):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = position
            constraint.tolerance_above = 0.01
            constraint.tolerance_below = 0.01
            constraint.weight = 1.0

            constraints.joint_constraints.append(constraint)

        goal.request.goal_constraints.append(constraints)

        future = self.move_client.send_goal_async(goal)
        future.add_done_callback(self.auto_scan_return_goal_response_callback)

    def auto_scan_return_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Greška pri prihvatanju povratnog MoveIt cilja: {e}')
            return

        if not goal_handle.accepted:
            self.get_logger().error('MoveIt JE ODBIO POVRATAK NA AUTO_SCAN POZICIJU!')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.auto_scan_return_result_callback)

    def auto_scan_return_result_callback(self, future):
        try:
            result = future.result().result
            error_code = result.error_code.val
        except Exception as e:
            self.get_logger().error(f'Greška pri čitanju rezultata povratka na AUTO_SCAN: {e}')
            return

        if error_code != 1:
            if error_code == -4 and self.auto_scan_retry_count < 1:
                self.auto_scan_retry_count += 1
                time.sleep(0.5)
                self.return_to_auto_scan()
                return

            self.get_logger().error('Robot NIJE uspeo da se vrati na AUTO_SCAN poziciju.')
            return

        self.auto_scan_retry_count = 0
        self.goal_sent = False
        self.phase = 16
        self.scan_state = 'PRIMARY'  # Resetujemo stanje na primarnu poziciju
        self.empty_scan_count = 0

        self.get_logger().info('ROBOT JE NA PRIMARNOJ AUTO_SCAN POZICIJI.')

    # =============================================================
    # DETACH
    # =============================================================

    def detach_cube(self):
        if self.real_cube_id is None:
            self.get_logger().error('Nema cube ID-a za detach!')
            return

        publisher = self.detach_publishers.get(self.real_cube_id)

        if publisher is None:
            self.get_logger().error(f'Nema detach publishera za {self.real_cube_id}!')
            return

        msg = Empty()
        publisher.publish(msg)

        if self.real_cube_id is not None:
            self.completed_gazebo_ids.add(self.real_cube_id)

        self.expected_gazebo_id = None
        color = self.detected_color.strip().upper()

        if color in self.placed_counts:
            self.placed_counts[color] += 1
        else:
            self.placed_counts[color] = 1

        self.phase = 15
        self.return_to_auto_scan()


def main(args=None):
    rclpy.init(args=args)
    node = MoveToCube()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()