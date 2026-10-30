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
    # Robot ostaje iznad kockice/stola dok ne bude tačno iznad nje.
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

    # TCP je 20 cm iznad površine dok je skeniramo.
    SURFACE_HEIGHT_OFFSET = 0.15

    # Kada pronađemo odgovarajuću površinu,
    # spuštamo TCP blizu površine pre detach-a.
    #
    # Kockica je približno 4 cm visoka,
    # a TCP je prilikom attach-a blizu gornjeg dela kockice.
    # TCP ostaje oko 3 cm iznad površine; nakon detach-a kockica pada.
    # Ovo je namerno nisko da pad bude mali, ali da TCP ne mora da ulazi u površinu.
    DROP_TCP_OFFSET = 0.085

    # Faza 13 koristi poseban MoveIt cilj.
    # Pri spuštanju nije potrebno da TCP bude orijentisan toliko strogo
    # kao pri prilazu kockici. Previše stroga orijentaciona putanja je
    # bila glavni uzrok MoveIt error_code = -13
    # (GOAL_VIOLATES_PATH_CONSTRAINTS).
    DROP_ORIENTATION_TOLERANCE = 0.50
    DROP_PLANNING_TIME = 8.0

    # =============================================================
    # SLOTOVI ZA VIŠE KOCKICA NA ISTOJ POVRŠINI
    # =============================================================
    #
    # Slot 0 = centar površine (prva kockica).
    # Sledeće kockice dobijaju mali XY pomak tako da se ne
    # spuštaju direktno preko već postavljene kockice.
    #
    # Offseti su u WORLD koordinatama. Z visina ostaje ista:
    # DROP_TCP_OFFSET = 0.07 m.
    #
    SURFACE_SLOT_OFFSETS = [
        (-0.035, -0.05),
        (0.05, 0.05),
        (0.05, -0.05),
        (-0.05, 0.05),
        (0.00, -0.12),
    ]

    # =============================================================
    # AUTO-SCAN POVRATNA POZICIJA
    # =============================================================
    #
    # Ovo je ista proverena scan1 pozicija koju koristi auto_scan.
    # Nakon što se prva kockica odvoji, robot se vraća baš ovde.
    #
    AUTO_SCAN_POSITION = [
        -1.0015103,
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
        # FAZE
        # =========================================================
        #
        # 0  = čekamo detekciju
        # 1  = idemo iznad kockice
        # 2  = prilazimo kockici
        # 3  = proveravamo kockicu
        # 5  = podizanje
        #
        # 7  = LEFT
        # 9  = MIDDLE
        # 11 = RIGHT
        #
        # 13 = spuštanje na odgovarajuću površinu
        # 14 = detach
        # 15 = povratak na AUTO_SCAN poziciju
        # 16 = robot se vratio na AUTO_SCAN poziciju
        # 20 = završeno

        self.phase = 0

        # Ako MoveIt odbije završni prilaz kockici,
        # dozvoljavamo još jedan pokušaj 5 mm više.
        self.approach_retry_count = 0

        # FAZA 2:
        # 0 = prvi dolazak na bezbednu visinu iznad kockice
        # 1 = završni prilaz na APPROACH_OFFSET.
        self.approach_step = 0

        # Drop se ne ponavlja na većoj visini.
        # Kockica se odvaja na fiksnih ~3 cm iznad površine.
        self.drop_retry_count = 0

        self.goal_sent = False

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

        # =========================================================
        # IZBOR SLEDEĆE KOCKICE - PROVERA GAZEBO ID-a
        # =========================================================
        # Detector ID nije trajan identitet. Pre nego što pošaljemo
        # MoveIt prema kandidatu, proveravamo preko /get_closest_cube
        # koji stvarni Gazebo cube_* se nalazi na toj detektovanoj
        # poziciji. Time completed_gazebo_ids postaje glavni kriterijum.
        self.candidate_selection_in_progress = False
        self.selection_candidates = []
        self.selection_index = 0

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
        # Trajni identitet uspešno spuštenih kockica.
        # Gazebo ID (cube_0 ... cube_4) se koristi kao glavni identitet;
        # Detector ID je samo trenutni ID iz jednog /detected_cubes frame-a.
        self.completed_gazebo_ids = set()

        # Broj već spuštenih kockica po boji.
        # Ovo određuje sledeći slot na odgovarajućoj površini.
        self.placed_counts = {
            'RED': 0,
            'GREEN': 0,
            'BLUE': 0,
        }

        # Trenutni slot za drop. Čuva se tokom cele iteracije
        # da retry ne promeni mesto odlaganja.
        self.current_drop_offset = (0.0, 0.0)
        self.current_drop_world = None

        # AUTO_SCAN može povremeno vratiti CONTROL_FAILED (-4).
        # Dozvoljavamo jedan ponovni pokušaj istog joint cilja.
        self.auto_scan_retry_count = 0

        # =========================================================
        # DINAMIČKI BROJ KOCKICA
        # =========================================================
        # Broj kockica određujemo na početku rada na osnovu
        # /detected_cubes. Ne zaključavamo broj iz jednog frame-a,
        # već čekamo nekoliko uzastopnih frame-ova sa istim brojem.
        self.target_cube_count = None
        self.count_candidate = None
        self.count_stable_frames = 0
        self.COUNT_STABLE_REQUIRED = 5

        # =========================================================
        # LOG
        # =========================================================

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            '       MOVE TO CUBE - FINAL'
        )

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'Cube detector: /detected_cubes'
        )

        self.get_logger().info(
            'Surface detector: /detect_surface'
        )

        self.get_logger().info(
            'Surface order: LEFT -> MIDDLE -> RIGHT'
        )

        self.get_logger().info(
            'Robot base WORLD: '
            f'({self.ROBOT_BASE_X}, '
            f'{self.ROBOT_BASE_Y}, '
            f'{self.ROBOT_BASE_Z})'
        )

        self.get_logger().info(
            'Robot WORLD yaw: +90 deg'
        )

        self.get_logger().info(
            'Cekam /detected_cubes...'
        )

    # =============================================================
    # DEBUG - JOINT STANJE
    # =============================================================

    def joint_state_callback(self, msg):

        self.latest_joint_positions = {
            name: position
            for name, position in zip(
                msg.name,
                msg.position
            )
        }

    def log_current_joints(self, title):

        self.get_logger().info(
            '----------------------------------------'
        )
        self.get_logger().info(title)

        joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6',
            'joint7'
        ]

        if not self.latest_joint_positions:
            self.get_logger().warn(
                '/joint_states nema dostupne podatke.'
            )
            self.get_logger().info(
                '----------------------------------------'
            )
            return

        for name in joint_names:
            if name in self.latest_joint_positions:
                self.get_logger().info(
                    f'{name} = '
                    f'{self.latest_joint_positions[name]:.6f}'
                )
            else:
                self.get_logger().warn(
                    f'{name} nije pronađen u /joint_states.'
                )

        self.get_logger().info(
            '----------------------------------------'
        )

    # =============================================================
    # LINK_BASE -> WORLD
    # =============================================================

    def base_to_world(self, x, y, z):

        cos_yaw = math.cos(self.ROBOT_YAW)
        sin_yaw = math.sin(self.ROBOT_YAW)

        world_x = (
            self.ROBOT_BASE_X
            + cos_yaw * x
            - sin_yaw * y
        )

        world_y = (
            self.ROBOT_BASE_Y
            + sin_yaw * x
            + cos_yaw * y
        )

        world_z = (
            self.ROBOT_BASE_Z
            + z
        )

        return world_x, world_y, world_z

    # =============================================================
    # WORLD -> LINK_BASE
    # =============================================================

    def world_to_base(self, x, y, z):

        dx = x - self.ROBOT_BASE_X
        dy = y - self.ROBOT_BASE_Y

        cos_yaw = math.cos(self.ROBOT_YAW)
        sin_yaw = math.sin(self.ROBOT_YAW)

        base_x = (
            cos_yaw * dx
            + sin_yaw * dy
        )

        base_y = (
            -sin_yaw * dx
            + cos_yaw * dy
        )

        base_z = (
            z - self.ROBOT_BASE_Z
        )

        return base_x, base_y, base_z

    # =============================================================
    # CUBE DETECTION
    # =============================================================

    def cubes_callback(self, msg):

        # Već smo krenuli sa jednom kockicom.
        if self.goal_sent:
            return

        # =========================================================
        # ODREĐIVANJE UKUPNOG BROJA KOCKICA
        # =========================================================
        # Prvi stabilan broj detektovanih kockica postaje ukupan
        # broj iteracija. Posle toga se taj broj više NE menja.
        # Ovo sprečava da se zbog jednog lošeg frame-a, npr.
        # 5 -> 4 -> 5, promeni broj potrebnih iteracija.
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
                f'Broj detektovanih kockica: {current_count} ' \
                f'({self.count_stable_frames}/' \
                f'{self.COUNT_STABLE_REQUIRED})'
            )

            if self.count_stable_frames < self.COUNT_STABLE_REQUIRED:
                return

            self.target_cube_count = current_count

            self.get_logger().info(
                '========================================'
            )
            self.get_logger().info(
                f'UKUPAN BROJ KOCKICA ZAKLJUČAN: ' \
                f'{self.target_cube_count}'
            )
            self.get_logger().info(
                f'Move_to_cube će izvršiti najviše ' \
                f'{self.target_cube_count} iteracija.'
            )
            self.get_logger().info(
                'Ovaj broj se više ne menja tokom sortiranja.'
            )
            self.get_logger().info(
                '========================================'
            )

        # =========================================================
        # KRAJ - SVE PLANIRANE KOCKICE SU ZAVRŠENE
        # =========================================================
        if (
            self.target_cube_count is not None
            and len(self.completed_gazebo_ids) >= self.target_cube_count
        ):
            self.get_logger().info(
                '========================================'
            )
            self.get_logger().info(
                f'SVE KOCKICE SU SORTIRANE: '
                f'{len(self.completed_gazebo_ids)}/'
                f'{self.target_cube_count}'
            )
            self.get_logger().info(
                'SORTIRANJE JE ZAVRŠENO.'
            )
            self.get_logger().info(
                '========================================'
            )
            self.phase = 20
            return

        # Dok proveravamo kandidata preko service-a ne pokrećemo
        # novi izbor iz svakog sledećeg /detected_cubes frame-a.
        if self.candidate_selection_in_progress:
            return

        # =========================================================
        # NEMA VIŠE KOCKICA
        # =========================================================
        if len(msg.cubes) == 0:
            return

        # =========================================================
        # FORMIRAJ KANDIDATE
        # =========================================================
        #
        # Ne koristimo completed_detector_ids za trajni identitet.
        # Detector ID može da se promeni kada kockica nestane iz
        # slike. Kandidati se zato prvo sortiraju po udaljenosti,
        # a zatim se svaki kandidat proverava preko /get_closest_cube
        # da bismo dobili stvarni Gazebo ID.
        #
        cubes = []

        for cube in msg.cubes:
            x = cube.x
            y = cube.y
            z = cube.z

            distance = math.sqrt(
                x * x +
                y * y +
                z * z
            )

            cubes.append({
                'id': cube.id,
                'color': cube.color,
                'x': x,
                'y': y,
                'z': z,
                'distance': distance
            })

        cubes.sort(
            key=lambda c: c['distance']
        )

        self.selection_candidates = cubes
        self.selection_index = 0
        self.candidate_selection_in_progress = True

        self.get_logger().info(
            '========================================'
        )
        self.get_logger().info(
            f'PROVERA KANDIDATA ZA SLEDEĆU KOCKICU '
            f'({len(cubes)} kandidata)'
        )
        self.get_logger().info(
            f'Već završene Gazebo kockice: '
            f'{sorted(self.completed_gazebo_ids)}'
        )
        self.get_logger().info(
            '========================================'
        )

        self.probe_next_candidate()

    # =============================================================
    # PROVERA KANDIDATA PRE MOVEIT-A
    # =============================================================

    def probe_next_candidate(self):

        if self.selection_index >= len(self.selection_candidates):

            self.candidate_selection_in_progress = False

            self.get_logger().error(
                'Nijedan detektovani kandidat nije mogao da se '
                'poveže sa nezavršenom Gazebo kockicom.'
            )
            self.get_logger().error(
                'Ne šaljem MoveIt prema neproverenoj kockici.'
            )

            return

        candidate = self.selection_candidates[
            self.selection_index
        ]

        self.get_logger().info(
            '----------------------------------------'
        )
        self.get_logger().info(
            f'PROVERA KANDIDATA '
            f'{self.selection_index + 1}/'
            f'{len(self.selection_candidates)}'
        )
        self.get_logger().info(
            f'Detector ID: {candidate["id"]}'
        )
        self.get_logger().info(
            f'Boja: {candidate["color"]}'
        )
        self.get_logger().info(
            f'LINK_BASE XYZ: '
            f'({candidate["x"]:.4f}, '
            f'{candidate["y"]:.4f}, '
            f'{candidate["z"]:.4f})'
        )
        self.get_logger().info(
            f'Udaljenost: {candidate["distance"]:.4f} m'
        )

        if not self.closest_client.wait_for_service(
            timeout_sec=2.0
        ):
            self.get_logger().error(
                '/get_closest_cube nije dostupan '
                'tokom izbora kandidata.'
            )
            self.selection_index += 1
            self.probe_next_candidate()
            return

        request = GetClosestCube.Request()

        request.target_pose = Pose()

        request.target_pose.position.x = candidate['x']
        request.target_pose.position.y = candidate['y']
        request.target_pose.position.z = candidate['z']

        request.target_pose.orientation.w = 1.0

        future = self.closest_client.call_async(
            request
        )

        future.add_done_callback(
            self.candidate_probe_result_callback
        )

    def candidate_probe_result_callback(self, future):

        try:
            response = future.result()

        except Exception as e:

            self.get_logger().error(
                f'Greška pri proveri kandidata: {e}'
            )

            self.selection_index += 1
            self.probe_next_candidate()
            return

        candidate = self.selection_candidates[
            self.selection_index
        ]

        if not response.found:

            self.get_logger().warn(
                'Kandidat nije potvrđen preko '
                '/get_closest_cube.'
            )

            self.selection_index += 1
            self.probe_next_candidate()
            return

        gazebo_id = response.cube_id

        self.get_logger().info(
            f'Kandidat odgovara Gazebo kockici: '
            f'{gazebo_id}'
        )

        if gazebo_id in self.completed_gazebo_ids:

            self.get_logger().warn(
                f'{gazebo_id} je VEĆ ZAVRŠENA. '
                'Preskačem ovaj detector kandidat.'
            )

            self.selection_index += 1
            self.probe_next_candidate()
            return

        # =========================================================
        # KANDIDAT JE VALIDAN
        # =========================================================

        self.candidate_selection_in_progress = False
        self.goal_sent = True

        self.detected_id = candidate['id']
        self.detected_color = candidate['color']

        self.cube_x = candidate['x']
        self.cube_y = candidate['y']
        self.cube_z = candidate['z']

        self.real_cube_id = None
        self.expected_gazebo_id = gazebo_id

        self.get_logger().info(
            '========================================'
        )
        self.get_logger().info(
            'IZABRANA NOVA KOCKICA'
        )
        self.get_logger().info(
            f'Detector ID (trenutni frame): '
            f'{self.detected_id}'
        )
        self.get_logger().info(
            f'Gazebo ID (trajni identitet): '
            f'{gazebo_id}'
        )
        self.get_logger().info(
            f'Boja: {self.detected_color}'
        )
        self.get_logger().info(
            f'LINK_BASE XYZ: '
            f'({self.cube_x:.4f}, '
            f'{self.cube_y:.4f}, '
            f'{self.cube_z:.4f})'
        )
        self.get_logger().info(
            'MoveIt sada prilazi provereno izabranoj kockici.'
        )
        self.get_logger().info(
            '========================================'
        )

        # Ne moramo da verujemo samo pre-proveri. Kada robot zaista
        # stigne do kockice, postojeća 3x provera ponovo potvrđuje
        # stvarni Gazebo ID pre attach-a.
        target_z = (
            self.cube_z +
            self.ABOVE_CUBE
        )

        self.phase = 1
        self.approach_retry_count = 0
        self.approach_step = 0

        self.get_logger().info(
            'FAZA 1 - 15 cm IZNAD KOCKICE'
        )
        self.get_logger().info(
            f'Target LINK_BASE: '
            f'({self.cube_x:.4f}, '
            f'{self.cube_y:.4f}, '
            f'{target_z:.4f})'
        )

        self.send_move_goal(
            self.cube_x,
            self.cube_y,
            target_z
        )

    # =============================================================
    # MOVEIT CILJ
    # =============================================================

    def send_move_goal(self, x, y, z):

        self.get_logger().info(
            'Cekam MoveIt /move_action...'
        )

        if not self.move_client.wait_for_server(
            timeout_sec=5.0
        ):

            self.get_logger().error(
                'MoveIt /move_action nije dostupan!'
            )

            return

        goal = MoveGroup.Goal()

        goal.request.group_name = 'xarm7'

        goal.request.max_velocity_scaling_factor = 0.25
        goal.request.max_acceleration_scaling_factor = 0.25

        goal.request.allowed_planning_time = 5.0

        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        # =========================================================
        # POSITION
        # =========================================================

        position = PositionConstraint()

        position.header.frame_id = 'link_base'
        position.link_name = 'link_tcp'

        primitive = SolidPrimitive()

        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.015]

        position.constraint_region.primitives.append(
            primitive
        )

        pose = Pose()

        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        position.constraint_region.primitive_poses.append(
            pose
        )

        position.weight = 1.0

        # =========================================================
        # ORIENTATION
        # =========================================================

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

        # =========================================================
        # CONSTRAINTS
        # =========================================================

        constraints = Constraints()

        constraints.position_constraints.append(
            position
        )

        constraints.orientation_constraints.append(
            orientation
        )

        goal.request.goal_constraints.append(
            constraints
        )

        self.get_logger().info(
            f'Saljem cilj: '
            f'({x:.4f}, '
            f'{y:.4f}, '
            f'{z:.4f})'
        )

        future = self.move_client.send_goal_async(
            goal
        )

        future.add_done_callback(
            self.goal_response_callback
        )

    # =============================================================
    # MOVEIT ACCEPT
    # =============================================================

    def goal_response_callback(self, future):

        try:

            goal_handle = future.result()

        except Exception as e:

            self.get_logger().error(
                f'Greska pri prihvatanju '
                f'MoveIt cilja: {e}'
            )

            return

        if not goal_handle.accepted:

            self.get_logger().error(
                'MoveIt JE ODBIO CILJ!'
            )

            return

        self.get_logger().info(
            'Cilj prihvacen.'
        )

        self.get_logger().info(
            'Robot se pomera...'
        )

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.result_callback
        )

    # =============================================================
    # MOVEIT RESULT
    # =============================================================

    def result_callback(self, future):

        try:

            result = future.result().result

            error_code = result.error_code.val

        except Exception as e:

            self.get_logger().error(
                f'Greska pri citanju '
                f'MoveIt rezultata: {e}'
            )

            return

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            f'MoveIt error_code = {error_code}'
        )

        if error_code == -13:
            self.get_logger().warn(
                'MoveIt: GOAL_VIOLATES_PATH_CONSTRAINTS'
            )
        elif error_code == -12:
            self.get_logger().warn(
                'MoveIt: GOAL_IN_COLLISION'
            )
        elif error_code == -4:
            self.get_logger().warn(
                'MoveIt: CONTROL_FAILED'
            )

        self.get_logger().info(
            '========================================'
        )

        if error_code != 1:

            # =====================================================
            # FAZA 2 - CARTESIAN PRILAZ
            # =====================================================
            #
            # Isto kao Cartesian DROP: FAZA 2 se ne retry-uje kroz
            # običan MoveGroup pose cilj (taj put se više uopšte ne
            # koristi za fazu 2). Ako Cartesian computation/execution
            # padne, ostajemo u fazi 2 i prijavljujemo problem.
            #

            # =====================================================
            # FAZA 13 - CARTESIAN DROP
            # =====================================================
            #
            # Cartesian DROP se ne retry-uje kroz običan MoveGroup
            # pose cilj. Ako Cartesian computation/execution padne,
            # ostajemo u fazi 13 i prijavljujemo problem.
            #
            self.get_logger().error(
                f'MoveIt nije uspeo. '
                f'Faza = {self.phase}'
            )

            if self.phase == 13:
                self.get_logger().error(
                    'DROP NIJE USPEO ni nakon retry-ja.'
                )

            return

        # =========================================================
        # FAZA 1 -> FAZA 2
        # =========================================================

        if self.phase == 1:

            self.get_logger().info(
                'ROBOT JE STIGAO 15 cm '
                'IZNAD KOCKICE.'
            )

            self.phase = 2

            self.send_cartesian_approach()

            return

        # =========================================================
        # FAZA 2 se sada u potpunosti završava u
        # send_cartesian_approach(). Uspešan ExecuteTrajectory tamo
        # direktno prelazi u FAZU 3 (check_tcp_position()), isto
        # kao što FAZA 13 prelazi u detach_cube().
        # =========================================================

        # =========================================================
        # FAZA 5 -> LIFT ZAVRŠEN
        # =========================================================

        if self.phase == 5:

            self.get_logger().info(
                '========================================'
            )

            self.get_logger().info(
                'KOCKICA JE USPESNO PODIGNUTA!'
            )

            self.get_logger().info(
                f'Gazebo cube: {self.real_cube_id}'
            )

            self.get_logger().info(
                f'Boja kockice: {self.detected_color}'
            )

            self.get_logger().info(
                '========================================'
            )

            # Prvo idemo na LEVU površinu
            self.go_to_surface(
                'left',
                self.LEFT_SURFACE_WORLD
            )

            return

        # =========================================================
        # STIGLI IZNAD POVRŠINE
        # =========================================================

        if self.phase in (7, 9, 11):

            self.get_logger().info(
                '========================================'
            )

            self.get_logger().info(
                'ROBOT JE STIGAO IZNAD '
                f'{self.current_surface_name.upper()} '
                'POVRŠINE.'
            )

            self.log_current_joints(
                'JOINT KONFIGURACIJA NEPOSREDNO PRE FAZE 13'
            )

            self.get_logger().info(
                'SADA POZIVAM /detect_surface.'
            )

            self.get_logger().info(
                '========================================'
            )

            self.request_surface_detection()

            return

        # =========================================================
        # FAZA 13 -> SPUŠTANJE ZAVRŠENO
        # =========================================================

        if self.phase == 13:

            # FAZA 13 se završava u send_cartesian_drop().
            # ExecuteTrajectory uspeh tamo direktno poziva detach_cube().
            self.get_logger().info(
                'FAZA 13 je obrađena kroz Cartesian DROP.'
            )

            return

        # =========================================================
        # FAZA 14
        # =========================================================
        #
        # Detach je već poslat u detach_cube().
        # Ovde samo evidentiramo da je prva kockica završena.
        #

        if self.phase == 14:

            self.get_logger().info(
                '========================================'
            )

            self.get_logger().info(
                'KOCKICA JE ODVOJENA OD TCP-a.'
            )

            self.get_logger().info(
                'PRVA KOCKICA JE ZAVRŠENA.'
            )

            self.get_logger().info(
                'VRAĆAM ROBOTA NA AUTO_SCAN POZICIJU.'
            )

            self.get_logger().info(
                '========================================'
            )

            self.phase = 15

            self.return_to_auto_scan()

            return

        # =========================================================
        # FAZA 15
        # =========================================================
        #
        # Povratak na poziciju na kojoj je auto_scan detektovao
        # Broj kockica je određen stabilnom početnom detekcijom.
        #

        if self.phase == 15:

            self.get_logger().info(
                '========================================'
            )

            self.get_logger().info(
                'ROBOT SE VRATIO NA AUTO_SCAN POZICIJU.'
            )

            self.get_logger().info(
                'SPREMAN ZA SLEDEĆI KORAK.'
            )

            self.get_logger().info(
                '========================================'
            )

            self.phase = 16

            return

    # =============================================================
    # CARTESIAN DROP - STVARNO STANJE ROBOTA
    # =============================================================

    def send_cartesian_approach(self):

        self.get_logger().info(
            '========================================'
        )
        self.get_logger().info(
            'FAZA 2 - CARTESIAN PRILAZ KOCKICI'
        )
        self.get_logger().info(
            'Umesto običnog MoveIt pose cilja koristim '
            '/compute_cartesian_path.'
        )

        if not self.cartesian_client.wait_for_service(
            timeout_sec=5.0
        ):
            self.get_logger().error(
                '/compute_cartesian_path nije dostupan.'
            )
            return

        if not self.execute_trajectory_client.wait_for_server(
            timeout_sec=5.0
        ):
            self.get_logger().error(
                '/execute_trajectory nije dostupan.'
            )
            return

        start_pose = self.get_current_tcp_pose_for_cartesian()

        if start_pose is None:
            return

        start_state = self.build_cartesian_start_state()

        if start_state is None:
            return

        final_z = (
            self.cube_z +
            self.APPROACH_OFFSET
        )

        # Isti princip kao Cartesian DROP, samo obrnuto:
        # Početak = stvarna trenutna TCP poza (~15 cm iznad kockice,
        # iz FAZE 1).
        # WP1 = X/Y kockice, na ISTOJ (trenutnoj) visini kao start —
        # ovo hvata slučaj da FAZA 1 nije sletela tačno iznad centra.
        # WP2 = X/Y kockice, na konačnoj visini prilaza.
        #
        # Time forsiramo L-putanju umesto slobodnog MoveIt plana:
        #
        #       START (iznad kockice, ~15 cm)
        #         |
        #         | horizontalno poravnanje (obično ~0)
        #         +---------------- IZNAD KOCKICE
        #                              |
        #                              | pravo dole
        #                              v
        #                          PRILAZ KOCKICI
        #
        # Ovo je tačno ono što je ranije nedostajalo: obični
        # MoveGroup pose cilj daje MoveIt-u slobodu da bira BILO
        # KOJU putanju do cilja, pa je nekad "zaobilazio" i sudarao
        # se sa stolom. Cartesian putanja je unapred izračunata,
        # pravolinijska, i proverena na koliziju PRE nego što se
        # robot uopšte pomeri (avoid_collisions=True, fraction>=0.99).

        aligned_pose = self.make_drop_waypoint(
            start_pose,
            self.cube_x,
            self.cube_y,
            start_pose.position.z
        )

        final_pose = self.make_drop_waypoint(
            start_pose,
            self.cube_x,
            self.cube_y,
            final_z
        )

        waypoints = [
            start_pose,
            aligned_pose,
            final_pose
        ]

        self.get_logger().info(
            f'START TCP: '
            f'({start_pose.position.x:.4f}, '
            f'{start_pose.position.y:.4f}, '
            f'{start_pose.position.z:.4f})'
        )

        self.get_logger().info(
            f'IZNAD KOCKICE: '
            f'({self.cube_x:.4f}, '
            f'{self.cube_y:.4f}, '
            f'{start_pose.position.z:.4f})'
        )

        self.get_logger().info(
            f'PRILAZ KOCKICI: '
            f'({self.cube_x:.4f}, '
            f'{self.cube_y:.4f}, '
            f'{final_z:.4f})'
        )

        self.get_logger().info(
            'X/Y se menjaju samo do tačke iznad kockice; '
            'zatim se menja samo Z.'
        )

        request = GetCartesianPath.Request()

        request.header.frame_id = 'link_base'
        request.start_state = start_state
        request.group_name = 'xarm7'
        request.link_name = 'link_tcp'
        request.waypoints = waypoints

        # 5 mm max Cartesian segment.
        request.max_step = 0.005

        request.jump_threshold = 0.0
        request.prismatic_jump_threshold = 0.0
        request.revolute_jump_threshold = 0.0

        request.avoid_collisions = True

        # Sporo i kontrolisano, isto kao Cartesian DROP.
        request.max_cartesian_speed = 0.02
        request.cartesian_speed_limited_link = 'link_tcp'

        self.get_logger().info(
            'Pozivam /compute_cartesian_path za FAZU 2...'
        )

        future = self.cartesian_client.call_async(request)

        def cartesian_result_callback(f):

            try:
                response = f.result()

            except Exception as e:
                self.get_logger().error(
                    f'Greška pri /compute_cartesian_path: {e}'
                )
                return

            self.get_logger().info(
                '----------------------------------------'
            )

            self.get_logger().info(
                f'Cartesian fraction = {response.fraction:.3f}'
            )

            self.get_logger().info(
                f'Cartesian error_code = '
                f'{response.error_code.val}'
            )

            if response.fraction < 0.99:

                self.get_logger().error(
                    'Cartesian PRILAZ putanja nije dovoljno '
                    'kompletna.'
                )

                self.get_logger().error(
                    f'Potrebno >= 0.99, '
                    f'dobijeno {response.fraction:.3f}'
                )

                self.get_logger().error(
                    'Ne izvršavam trajektoriju.'
                )

                return

            if response.error_code.val != 1:

                self.get_logger().error(
                    'Cartesian PRILAZ nije uspešno izračunat.'
                )

                return

            if not response.solution.joint_trajectory.points:

                self.get_logger().error(
                    'Cartesian PRILAZ trajektorija nema tačke.'
                )

                return

            self.get_logger().info(
                'Cartesian PRILAZ putanja je validna.'
            )

            self.get_logger().info(
                f'Broj trajectory tačaka: '
                f'{len(response.solution.joint_trajectory.points)}'
            )

            goal = ExecuteTrajectory.Goal()
            goal.trajectory = response.solution
            goal.controller_names = []

            self.get_logger().info(
                'Šaljem trajektoriju na /execute_trajectory...'
            )

            execute_future = (
                self.execute_trajectory_client.send_goal_async(
                    goal
                )
            )

            def execute_goal_callback(ef):

                try:
                    goal_handle = ef.result()

                except Exception as e:
                    self.get_logger().error(
                        f'Greška pri slanju ExecuteTrajectory: {e}'
                    )
                    return

                if not goal_handle.accepted:

                    self.get_logger().error(
                        'ExecuteTrajectory je ODBIO Cartesian '
                        'PRILAZ.'
                    )
                    return

                self.get_logger().info(
                    'ExecuteTrajectory cilj prihvaćen.'
                )

                result_future = goal_handle.get_result_async()

                def execute_result_callback(rf):

                    try:
                        result = rf.result().result
                        error_code = result.error_code.val

                    except Exception as e:
                        self.get_logger().error(
                            f'Greška pri čitanju ExecuteTrajectory '
                            f'rezultata: {e}'
                        )
                        return

                    self.get_logger().info(
                        '========================================'
                    )

                    self.get_logger().info(
                        f'Cartesian ExecuteTrajectory '
                        f'error_code = {error_code}'
                    )

                    if error_code != 1:

                        self.get_logger().error(
                            'Cartesian PRILAZ nije izvršen.'
                        )

                        return

                    self.get_logger().info(
                        '========================================'
                    )

                    self.get_logger().info(
                        'FAZA 2 - CARTESIAN PRILAZ USPEŠAN.'
                    )

                    self.get_logger().info(
                        'ROBOT JE STIGAO BLIZU KOCKICE.'
                    )

                    self.get_logger().info(
                        'SADA PROVERAVAM TCP POZICIJU.'
                    )

                    self.phase = 3

                    self.check_tcp_position()

                result_future.add_done_callback(
                    execute_result_callback
                )

            execute_future.add_done_callback(
                execute_goal_callback
            )

        future.add_done_callback(
            cartesian_result_callback
        )

    def get_current_tcp_pose_for_cartesian(self):

        self.get_logger().info(
            'Cekam link_base -> link_tcp TF za Cartesian DROP...'
        )

        for _ in range(200):

            try:
                if self.tf_buffer.can_transform(
                    'link_base',
                    'link_tcp',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05)
                ):
                    transform = self.tf_buffer.lookup_transform(
                        'link_base',
                        'link_tcp',
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

                    return pose

            except Exception:
                pass

            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().error(
            'Nisam uspeo da pročitam link_base -> link_tcp TF za DROP.'
        )
        return None

    def build_cartesian_start_state(self):

        if not self.latest_joint_positions:
            self.get_logger().error(
                '/joint_states nema podatke za Cartesian DROP.'
            )
            return None

        state = RobotState()

        state.joint_state.name = list(
            self.latest_joint_positions.keys()
        )
        state.joint_state.position = list(
            self.latest_joint_positions.values()
        )

        state.is_diff = False

        return state

    def make_drop_waypoint(self, reference_pose, x, y, z):

        pose = Pose()

        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        pose.orientation = reference_pose.orientation

        return pose

    def send_cartesian_drop(self):

        self.get_logger().info(
            '========================================'
        )
        self.get_logger().info(
            'FAZA 13 - CARTESIAN DROP'
        )
        self.get_logger().info(
            'Umesto običnog MoveIt pose cilja koristim '
            '/compute_cartesian_path.'
        )

        if not self.cartesian_client.wait_for_service(
            timeout_sec=5.0
        ):
            self.get_logger().error(
                '/compute_cartesian_path nije dostupan.'
            )
            return

        if not self.execute_trajectory_client.wait_for_server(
            timeout_sec=5.0
        ):
            self.get_logger().error(
                '/execute_trajectory nije dostupan.'
            )
            return

        start_pose = self.get_current_tcp_pose_for_cartesian()

        if start_pose is None:
            return

        start_state = self.build_cartesian_start_state()

        if start_state is None:
            return

        if self.current_drop_world is None:
            self.get_logger().error(
                'Nema current_drop_world za Cartesian DROP.'
            )
            return

        drop_base_x, drop_base_y, drop_base_z = (
            self.world_to_base(
                self.current_drop_world[0],
                self.current_drop_world[1],
                self.current_drop_world[2]
            )
        )

        # Početak = stvarna trenutna TCP poza.
        # WP1 = izabrani slot, ali na ISTOJ početnoj visini.
        # WP2 = isti slot, konačna DROP Z.
        #
        # Time forsiramo L-putanju:
        #
        #       START
        #         |
        #         | horizontalno do SLOT-a
        #         +---------------- SLOT
        #                              |
        #                              |
        #                              v
        #                             DROP
        #
        # Horizontalni deo je veoma mali (slot offset 5 cm),
        # a završni deo je strogo vertikalan.

        slot_pose = self.make_drop_waypoint(
            start_pose,
            drop_base_x,
            drop_base_y,
            start_pose.position.z
        )

        final_pose = self.make_drop_waypoint(
            start_pose,
            drop_base_x,
            drop_base_y,
            drop_base_z
        )

        waypoints = [
            start_pose,
            slot_pose,
            final_pose
        ]

        self.get_logger().info(
            f'START TCP: '
            f'({start_pose.position.x:.4f}, '
            f'{start_pose.position.y:.4f}, '
            f'{start_pose.position.z:.4f})'
        )

        self.get_logger().info(
            f'SLOT SAFE: '
            f'({drop_base_x:.4f}, '
            f'{drop_base_y:.4f}, '
            f'{start_pose.position.z:.4f})'
        )

        self.get_logger().info(
            f'FINAL DROP: '
            f'({drop_base_x:.4f}, '
            f'{drop_base_y:.4f}, '
            f'{drop_base_z:.4f})'
        )

        self.get_logger().info(
            'X/Y se menjaju samo do SLOT-a; '
            'zatim se menja samo Z.'
        )

        request = GetCartesianPath.Request()

        request.header.frame_id = 'link_base'
        request.start_state = start_state
        request.group_name = 'xarm7'
        request.link_name = 'link_tcp'
        request.waypoints = waypoints

        # 5 mm max Cartesian segment.
        request.max_step = 0.005

        # Za prvi pravi drop ne koristimo dodatni jump filter.
        request.jump_threshold = 0.0
        request.prismatic_jump_threshold = 0.0
        request.revolute_jump_threshold = 0.0

        request.avoid_collisions = True

        # Sporo i kontrolisano.
        request.max_cartesian_speed = 0.05
        request.cartesian_speed_limited_link = 'link_tcp'

        self.get_logger().info(
            'Pozivam /compute_cartesian_path za FAZU 13...'
        )

        future = self.cartesian_client.call_async(request)

        def cartesian_result_callback(f):

            try:
                response = f.result()

            except Exception as e:
                self.get_logger().error(
                    f'Greška pri /compute_cartesian_path: {e}'
                )
                return

            self.get_logger().info(
                '----------------------------------------'
            )

            self.get_logger().info(
                f'Cartesian fraction = {response.fraction:.3f}'
            )

            self.get_logger().info(
                f'Cartesian error_code = '
                f'{response.error_code.val}'
            )

            if response.fraction < 0.99:

                self.get_logger().error(
                    'Cartesian DROP putanja nije dovoljno kompletna.'
                )

                self.get_logger().error(
                    f'Potrebno >= 0.99, '
                    f'dobijeno {response.fraction:.3f}'
                )

                self.get_logger().error(
                    'Ne izvršavam trajektoriju.'
                )

                return

            if response.error_code.val != 1:

                self.get_logger().error(
                    'Cartesian DROP nije uspešno izračunat.'
                )

                return

            if not response.solution.joint_trajectory.points:

                self.get_logger().error(
                    'Cartesian DROP trajektorija nema tačke.'
                )

                return

            self.get_logger().info(
                'Cartesian DROP putanja je validna.'
            )

            self.get_logger().info(
                f'Broj trajectory tačaka: '
                f'{len(response.solution.joint_trajectory.points)}'
            )

            goal = ExecuteTrajectory.Goal()
            goal.trajectory = response.solution
            goal.controller_names = []

            self.get_logger().info(
                'Šaljem trajektoriju na /execute_trajectory...'
            )

            execute_future = (
                self.execute_trajectory_client.send_goal_async(
                    goal
                )
            )

            def execute_goal_callback(ef):

                try:
                    goal_handle = ef.result()

                except Exception as e:
                    self.get_logger().error(
                        f'Greška pri slanju ExecuteTrajectory: {e}'
                    )
                    return

                if not goal_handle.accepted:

                    self.get_logger().error(
                        'ExecuteTrajectory je ODBIO Cartesian DROP.'
                    )
                    return

                self.get_logger().info(
                    'ExecuteTrajectory cilj prihvaćen.'
                )

                result_future = goal_handle.get_result_async()

                def execute_result_callback(rf):

                    try:
                        result = rf.result().result
                        error_code = result.error_code.val

                    except Exception as e:
                        self.get_logger().error(
                            f'Greška pri čitanju ExecuteTrajectory '
                            f'rezultata: {e}'
                        )
                        return

                    self.get_logger().info(
                        '========================================'
                    )

                    self.get_logger().info(
                        f'Cartesian ExecuteTrajectory '
                        f'error_code = {error_code}'
                    )

                    if error_code != 1:

                        self.get_logger().error(
                            'Cartesian DROP nije izvršen.'
                        )

                        return

                    self.get_logger().info(
                        '========================================'
                    )

                    self.get_logger().info(
                        'FAZA 13 - CARTESIAN DROP USPEŠAN.'
                    )

                    self.get_logger().info(
                        'Kockica je na DROP poziciji.'
                    )

                    self.get_logger().info(
                        'SADA ŠALJEM DETACH.'
                    )

                    self.detach_cube()

                result_future.add_done_callback(
                    execute_result_callback
                )

            execute_future.add_done_callback(
                execute_goal_callback
            )

        future.add_done_callback(
            cartesian_result_callback
        )

    # =============================================================
    # TCP POZICIJA
    # =============================================================

    def check_tcp_position(self):

        self.get_logger().info(
            '----------------------------------------'
        )

        self.get_logger().info(
            'PROVERA TCP POZICIJE'
        )

        try:

            transform = (
                self.tf_buffer.lookup_transform(
                    'link_base',
                    'link_tcp',
                    rclpy.time.Time()
                )
            )

        except Exception as e:

            self.get_logger().error(
                f'TF nije dostupan: {e}'
            )

            return

        tcp_x = (
            transform.transform.translation.x
        )

        tcp_y = (
            transform.transform.translation.y
        )

        tcp_z = (
            transform.transform.translation.z
        )

        self.get_logger().info(
            f'TCP LINK_BASE: '
            f'({tcp_x:.4f}, '
            f'{tcp_y:.4f}, '
            f'{tcp_z:.4f})'
        )

        world_x, world_y, world_z = (
            self.base_to_world(
                tcp_x,
                tcp_y,
                tcp_z
            )
        )

        self.get_logger().info(
            f'TCP WORLD: '
            f'({world_x:.4f}, '
            f'{world_y:.4f}, '
            f'{world_z:.4f})'
        )

        # =========================================================
        # RESET SERVICE PROVERE
        # =========================================================

        self.service_check_count = 0
        self.service_check_results = []

        self.check_closest_cube(
            tcp_x,
            tcp_y,
            tcp_z
        )

    # =============================================================
    # GET CLOSEST CUBE
    # =============================================================

    def check_closest_cube(
        self,
        base_x,
        base_y,
        base_z
    ):

        if not self.closest_client.wait_for_service(
            timeout_sec=2.0
        ):

            self.get_logger().error(
                '/get_closest_cube nije dostupan!'
            )

            return

        request = GetClosestCube.Request()

        request.target_pose = Pose()

        request.target_pose.position.x = base_x
        request.target_pose.position.y = base_y
        request.target_pose.position.z = base_z

        request.target_pose.orientation.w = 1.0

        self.get_logger().info(
            f'Service provera '
            f'{self.service_check_count + 1}/3'
        )

        self.get_logger().info(
            f'Target LINK_BASE: '
            f'({base_x:.4f}, '
            f'{base_y:.4f}, '
            f'{base_z:.4f})'
        )

        future = (
            self.closest_client.call_async(
                request
            )
        )

        future.add_done_callback(
            lambda f: self.service_result_callback(
                f,
                base_x,
                base_y,
                base_z
            )
        )

    # =============================================================
    # SERVICE RESULT
    # =============================================================

    def service_result_callback(
        self,
        future,
        base_x,
        base_y,
        base_z
    ):

        try:

            response = future.result()

        except Exception as e:

            self.get_logger().error(
                f'Greska service-a: {e}'
            )

            return

        self.service_check_count += 1

        if response.found:

            self.get_logger().info(
                f'Service rezultat: '
                f'{response.cube_id}'
            )

            self.service_check_results.append(
                response.cube_id
            )

        else:

            self.get_logger().warn(
                'Service nije pronasao kockicu!'
            )

            self.service_check_results.append(
                None
            )

        # =========================================================
        # JOŠ NISMO 3 PUTA
        # =========================================================

        if self.service_check_count < 3:

            time.sleep(0.15)

            self.check_closest_cube(
                base_x,
                base_y,
                base_z
            )

            return

        # =========================================================
        # REZULTATI
        # =========================================================

        self.get_logger().info(
            '----------------------------------------'
        )

        self.get_logger().info(
            'REZULTATI 3 SERVICE PROVERE'
        )

        for i, result in enumerate(
            self.service_check_results,
            start=1
        ):

            self.get_logger().info(
                f'  {i}: {result}'
            )

        valid_results = [
            r
            for r in self.service_check_results
            if r is not None
        ]

        # =========================================================
        # NEMA KOCKICE
        # =========================================================

        if len(valid_results) == 0:

            self.get_logger().error(
                'SERVICE NIJEDNOM NIJE '
                'PRONASAO KOCKICU!'
            )

            self.get_logger().error(
                'ATTACH SE NECE IZVRSITI.'
            )

            return

        # =========================================================
        # RAZLIČITI REZULTATI
        # =========================================================

        if len(set(valid_results)) != 1:

            self.get_logger().error(
                'SERVICE JE VRATIO '
                'RAZLICITE KOCKICE!'
            )

            self.get_logger().error(
                'ATTACH SE NECE IZVRSITI.'
            )

            return

        # =========================================================
        # POTVRĐENA KOCKICA
        # =========================================================

        self.real_cube_id = valid_results[0]

        if (
            self.expected_gazebo_id is not None
            and self.real_cube_id != self.expected_gazebo_id
        ):
            self.get_logger().error(
                'PAŽNJA: Gazebo ID na stvarnoj poziciji '
                f'({self.real_cube_id}) se razlikuje od '
                f'proverenog kandidata '
                f'({self.expected_gazebo_id}).'
            )
            self.get_logger().error(
                'ATTACH SE NEĆE IZVRŠITI.'
            )
            self.goal_sent = False
            self.phase = 16
            self.expected_gazebo_id = None
            return

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'KOCKICA POTVRDJENA!'
        )

        self.get_logger().info(
            f'Gazebo ID = {self.real_cube_id}'
        )

        self.get_logger().info(
            f'Detector ID = {self.detected_id}'
        )

        self.get_logger().info(
            f'Boja = {self.detected_color}'
        )

        self.get_logger().info(
            '========================================'
        )

        self.attach_cube()

    # =============================================================
    # ATTACH
    # =============================================================

    def attach_cube(self):

        if self.real_cube_id is None:

            self.get_logger().error(
                'Nema validnog Gazebo cube ID-a!'
            )

            return

        publisher = (
            self.attach_publishers.get(
                self.real_cube_id
            )
        )

        if publisher is None:

            self.get_logger().error(
                f'Nema attach publishera za '
                f'{self.real_cube_id}!'
            )

            return

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'ATTACH KOCKICE'
        )

        self.get_logger().info(
            f'Cube: {self.real_cube_id}'
        )

        self.get_logger().info(
            f'Topic: '
            f'/gz/{self.real_cube_id}/attach'
        )

        msg = Empty()

        publisher.publish(msg)

        self.get_logger().info(
            'ATTACH PORUKA POSLATA!'
        )

        self.get_logger().info(
            'Kockica je sada spojena sa TCP-om.'
        )

        # =========================================================
        # LIFT
        # =========================================================

        self.phase = 5

        lift_z = (
            self.cube_z +
            self.LIFT_HEIGHT
        )

        self.get_logger().info(
            '----------------------------------------'
        )

        self.get_logger().info(
            'FAZA 3 - PODIZANJE KOCKICE'
        )

        self.get_logger().info(
            f'Lift target: '
            f'({self.cube_x:.4f}, '
            f'{self.cube_y:.4f}, '
            f'{lift_z:.4f})'
        )

        self.send_move_goal(
            self.cube_x,
            self.cube_y,
            lift_z
        )

    # =============================================================
    # ODLASAK NA POVRŠINU
    # =============================================================

    def go_to_surface(
        self,
        surface_name,
        world_position
    ):

        self.current_surface_name = surface_name
        self.current_surface_world = world_position

        self.detected_surface_color = None

        # =========================================================
        # OVDE NE DETEKTUJEMO POVRŠINU
        #
        # Prvo se robot pomera.
        # Tek nakon MoveIt rezultata pozivamo /detect_surface.
        # =========================================================

        world_x = world_position[0]
        world_y = world_position[1]

        world_z = (
            world_position[2]
            + self.SURFACE_HEIGHT_OFFSET
        )

        base_x, base_y, base_z = (
            self.world_to_base(
                world_x,
                world_y,
                world_z
            )
        )

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            f'IDEMO NA '
            f'{surface_name.upper()} POVRŠINU'
        )

        self.get_logger().info(
            'Robot i dalje DRŽI KOCKICU.'
        )

        self.get_logger().info(
            f'SURFACE WORLD: '
            f'({world_position[0]:.4f}, '
            f'{world_position[1]:.4f}, '
            f'{world_position[2]:.4f})'
        )

        self.get_logger().info(
            f'TCP TARGET WORLD: '
            f'({world_x:.4f}, '
            f'{world_y:.4f}, '
            f'{world_z:.4f})'
        )

        self.get_logger().info(
            f'TCP TARGET LINK_BASE: '
            f'({base_x:.4f}, '
            f'{base_y:.4f}, '
            f'{base_z:.4f})'
        )

        self.get_logger().info(
            '========================================'
        )

        # =========================================================
        # FAZA
        # =========================================================

        if surface_name == 'left':

            self.phase = 7

        elif surface_name == 'middle':

            self.phase = 9

        elif surface_name == 'right':

            self.phase = 11

        else:

            self.get_logger().error(
                f'Nepoznata površina: '
                f'{surface_name}'
            )

            return

        # =========================================================
        # MOVE
        # =========================================================

        self.send_move_goal(
            base_x,
            base_y,
            base_z
        )

    # =============================================================
    # DETEKCIJA POVRŠINE
    # =============================================================

    def request_surface_detection(self):

        if self.surface_service_pending:

            self.get_logger().warn(
                '/detect_surface je već u toku.'
            )

            return

        if self.current_surface_name is None:

            self.get_logger().error(
                'Nema definisane trenutne površine!'
            )

            return

        self.get_logger().info(
            '----------------------------------------'
        )

        self.get_logger().info(
            'POZIVAM /detect_surface'
        )

        self.get_logger().info(
            f'Površina: '
            f'{self.current_surface_name.upper()}'
        )

        self.get_logger().info(
            'Robot je STIGAO iznad površine.'
        )

        self.get_logger().info(
            'Skeniranje boje...'
        )

        if not self.surface_client.wait_for_service(
            timeout_sec=2.0
        ):

            self.get_logger().error(
                '/detect_surface nije dostupan!'
            )

            return

        request = DetectSurface.Request()

        self.surface_service_pending = True

        future = (
            self.surface_client.call_async(
                request
            )
        )

        future.add_done_callback(
            self.surface_service_result_callback
        )

    # =============================================================
    # SURFACE SERVICE RESULT
    # =============================================================

    def surface_service_result_callback(
        self,
        future
    ):

        self.surface_service_pending = False

        try:

            response = future.result()

        except Exception as e:

            self.get_logger().error(
                f'Greška pri /detect_surface: '
                f'{e}'
            )

            return

        self.get_logger().info(
            '========================================'
        )

        # =========================================================
        # DETEKCIJA NIJE USPELA
        # =========================================================

        if not response.success:

            self.get_logger().warn(
                'Surface detector nije potvrdio '
                'površinu.'
            )

            self.get_logger().warn(
                f'Confidence: '
                f'{response.confidence:.3f}'
            )

            self.get_logger().info(
                'Pokušaću sledeću površinu.'
            )

            self.get_logger().info(
                '========================================'
            )

            self.try_next_surface()

            return

        # =========================================================
        # USPEŠNA DETEKCIJA
        # =========================================================

        self.detected_surface_color = (
            response.color.strip().upper()
        )

        confidence = response.confidence

        cube_color = (
            self.detected_color.strip().upper()
        )

        surface_color = (
            self.detected_surface_color
        )

        self.get_logger().info(
            f'POVRŠINA POTVRDJENA: '
            f'{surface_color}'
        )

        self.get_logger().info(
            f'Confidence: '
            f'{confidence:.3f}'
        )

        self.get_logger().info(
            f'BOJA KOCKICE: '
            f'{cube_color}'
        )

        self.get_logger().info(
            f'TRENUTNA POVRŠINA: '
            f'{self.current_surface_name.upper()}'
        )

        self.get_logger().info(
            '========================================'
        )

        # =========================================================
        # BOJE SE POKLAPAJU
        # =========================================================

        if cube_color == surface_color:

            self.get_logger().info(
                '========================================'
            )

            self.get_logger().info(
                'POKLAPANJE BOJA!'
            )

            self.get_logger().info(
                f'KOCKICA  = {cube_color}'
            )

            self.get_logger().info(
                f'POVRŠINA = {surface_color}'
            )

            self.get_logger().info(
                f'{self.current_surface_name.upper()} '
                'JE ODGOVARAJUĆA.'
            )

            self.get_logger().info(
                'Spuštamo kockicu.'
            )

            self.get_logger().info(
                '========================================'
            )

            self.go_to_drop_position()

            return

        # =========================================================
        # BOJE SE NE POKLAPAJU
        # =========================================================

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'BOJE SE NE POKLAPAJU!'
        )

        self.get_logger().info(
            f'KOCKICA  = {cube_color}'
        )

        self.get_logger().info(
            f'POVRŠINA = {surface_color}'
        )

        self.get_logger().info(
            'Tražim sledeću površinu.'
        )

        self.get_logger().info(
            '========================================'
        )

        self.try_next_surface()

    # =============================================================
    # SLEDEĆA POVRŠINA
    # =============================================================

    def try_next_surface(self):

        if self.current_surface_name == 'left':

            self.get_logger().info(
                'LEVA površina nije odgovarajuća.'
            )

            self.get_logger().info(
                'Idemo na SREDNJU površinu.'
            )

            self.go_to_surface(
                'middle',
                self.MIDDLE_SURFACE_WORLD
            )

            return

        if self.current_surface_name == 'middle':

            self.get_logger().info(
                'SREDNJA površina nije odgovarajuća.'
            )

            self.get_logger().info(
                'Idemo na DESNU površinu.'
            )

            self.go_to_surface(
                'right',
                self.RIGHT_SURFACE_WORLD
            )

            return

        if self.current_surface_name == 'right':

            self.get_logger().error(
                'Nijedna površina ne odgovara '
                'boji kockice.'
            )

            self.get_logger().error(
                f'Boja kockice: '
                f'{self.detected_color}'
            )

            self.get_logger().error(
                'Kockica neće biti spuštena.'
            )

            self.phase = 20

            return

    # =============================================================
    # POZICIJA ZA SPUŠTANJE
    # =============================================================

    def go_to_drop_position(self):

        if self.current_surface_world is None:

            self.get_logger().error(
                'Nema pozicije površine!'
            )

            return

        color = self.detected_color.strip().upper()
        placed_count = self.placed_counts.get(color, 0)

        # Za pet kockica koristimo pet unapred definisanih slotova.
        # Ako se ikada pojavi više od pet, poslednji slot se ponavlja
        # umesto da se izadje iz liste.
        slot_index = min(
            placed_count,
            len(self.SURFACE_SLOT_OFFSETS) - 1
        )

        offset_x, offset_y = (
            self.SURFACE_SLOT_OFFSETS[slot_index]
        )

        self.current_drop_offset = (
            offset_x,
            offset_y
        )

        world_x = (
            self.current_surface_world[0] +
            offset_x
        )

        world_y = (
            self.current_surface_world[1] +
            offset_y
        )

        # VISINA OSTaje 0.07 m — ovo ne menjamo.
        world_z = (
            self.current_surface_world[2] +
            self.DROP_TCP_OFFSET
        )

        self.current_drop_world = (
            world_x,
            world_y,
            world_z
        )

        base_x, base_y, base_z = (
            self.world_to_base(
                world_x,
                world_y,
                world_z
            )
        )

        self.phase = 13
        self.drop_retry_count = 0
        self.DROP_ORIENTATION_TOLERANCE = 0.50

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'FAZA 13 - SPUŠTANJE KOCKICE'
        )

        self.get_logger().info(
            f'Surface: {self.current_surface_name.upper()}'
        )

        self.get_logger().info(
            f'Boja: {color}'
        )

        self.get_logger().info(
            f'Već postavljenih {color} kockica: '
            f'{placed_count}'
        )

        self.get_logger().info(
            f'Koristim SLOT: {slot_index + 1}'
        )

        self.get_logger().info(
            f'SLOT OFFSET WORLD: '
            f'dX={offset_x:+.3f} m, '
            f'dY={offset_y:+.3f} m'
        )

        self.get_logger().info(
            f'TCP DROP WORLD: '
            f'({world_x:.4f}, '
            f'{world_y:.4f}, '
            f'{world_z:.4f})'
        )

        self.get_logger().info(
            f'TCP DROP LINK_BASE: '
            f'({base_x:.4f}, '
            f'{base_y:.4f}, '
            f'{base_z:.4f})'
        )

        self.get_logger().info(
            'DROP_TCP_OFFSET ostaje 0.07 m.'
        )

        self.get_logger().info(
            'Kockica se spušta na izabrani slot...'
        )

        self.get_logger().info(
            '========================================'
        )

        # FAZA 13 više ne koristi običan MoveGroup pose goal.
        # Koristimo stvarni TCP + GetCartesianPath + ExecuteTrajectory
        # da završni spust bude kontrolisana geometrijska putanja.
        self.send_cartesian_drop()

    # =============================================================
    # POSEBAN MOVEIT CILJ ZA FAZU 13
    # =============================================================

    def send_drop_move_goal(self, x, y, z):

        self.get_logger().info(
            'Cekam MoveIt /move_action za DROP...'
        )

        if not self.move_client.wait_for_server(
            timeout_sec=5.0
        ):

            self.get_logger().error(
                'MoveIt /move_action nije dostupan!'
            )

            return

        goal = MoveGroup.Goal()

        goal.request.group_name = 'xarm7'

        # Spuštanje ide sporije i sa više vremena za planiranje.
        goal.request.max_velocity_scaling_factor = 0.15
        goal.request.max_acceleration_scaling_factor = 0.15
        goal.request.allowed_planning_time = self.DROP_PLANNING_TIME

        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5

        # =========================================================
        # POSITION
        # =========================================================

        position = PositionConstraint()

        position.header.frame_id = 'link_base'
        position.link_name = 'link_tcp'

        primitive = SolidPrimitive()

        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.015]

        position.constraint_region.primitives.append(
            primitive
        )

        pose = Pose()

        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        position.constraint_region.primitive_poses.append(
            pose
        )

        position.weight = 1.0

        # =========================================================
        # ORIENTATION - OPUŠTENA ZA FAZU 13
        # =========================================================

        orientation = OrientationConstraint()

        orientation.header.frame_id = 'link_base'
        orientation.link_name = 'link_tcp'

        orientation.orientation.x = 1.0
        orientation.orientation.y = 0.0
        orientation.orientation.z = 0.0
        orientation.orientation.w = 0.0

        orientation.absolute_x_axis_tolerance = (
            self.DROP_ORIENTATION_TOLERANCE
        )
        orientation.absolute_y_axis_tolerance = (
            self.DROP_ORIENTATION_TOLERANCE
        )
        orientation.absolute_z_axis_tolerance = (
            self.DROP_ORIENTATION_TOLERANCE
        )

        orientation.weight = 1.0

        # =========================================================
        # CONSTRAINTS
        # =========================================================

        constraints = Constraints()

        constraints.position_constraints.append(
            position
        )

        constraints.orientation_constraints.append(
            orientation
        )

        goal.request.goal_constraints.append(
            constraints
        )

        self.get_logger().info(
            '----------------------------------------'
        )

        self.get_logger().info(
            'FAZA 13 - POSEBAN DROP MOVEIT CILJ'
        )

        self.get_logger().info(
            f'Drop target: '
            f'({x:.4f}, {y:.4f}, {z:.4f})'
        )

        self.get_logger().info(
            f'Drop orientation tolerance: '
            f'{self.DROP_ORIENTATION_TOLERANCE:.2f} rad'
        )

        self.get_logger().info(
            f'Planning time: '
            f'{self.DROP_PLANNING_TIME:.1f} s'
        )

        self.get_logger().info(
            'Replan attempts: 5'
        )

        self.log_current_joints(
            'JOINT KONFIGURACIJA NEPOSREDNO PRE DROP MOVEIT CILJA'
        )

        self.get_logger().info(
            f'DROP TARGET ZA DEBUG: '
            f'X={x:.4f}, Y={y:.4f}, Z={z:.4f}'
        )

        if self.current_surface_world is not None:
            self.get_logger().info(
                f'SURFACE WORLD ZA DEBUG: '
                f'X={self.current_surface_world[0]:.4f}, '
                f'Y={self.current_surface_world[1]:.4f}, '
                f'Z={self.current_surface_world[2]:.4f}'
            )

        self.get_logger().info(
            f'SLOT OFFSET ZA DEBUG: '
            f'dX={self.current_drop_offset[0]:+.4f}, '
            f'dY={self.current_drop_offset[1]:+.4f}'
        )

        self.get_logger().info(
            'Šaljem DROP cilj...'
        )

        future = self.move_client.send_goal_async(
            goal
        )

        future.add_done_callback(
            self.goal_response_callback
        )

    # =============================================================
    # POVRATAK NA AUTO_SCAN POZICIJU
    # =============================================================

    def return_to_auto_scan(self):

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'POVRATAK NA AUTO_SCAN POZICIJU'
        )

        self.get_logger().info(
            'Robot se vraća na joint poziciju '
            'na kojoj je detektovano 5 kockica.'
        )

        for i, value in enumerate(
            self.AUTO_SCAN_POSITION,
            start=1
        ):
            self.get_logger().info(
                f'joint{i} = {value:.4f}'
            )

        self.get_logger().info(
            '========================================'
        )

        if not self.move_client.wait_for_server(
            timeout_sec=5.0
        ):

            self.get_logger().error(
                'MoveIt /move_action nije dostupan!'
            )

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
            self.AUTO_SCAN_POSITION
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

        goal.request.goal_constraints.append(
            constraints
        )

        future = self.move_client.send_goal_async(
            goal
        )

        future.add_done_callback(
            self.auto_scan_return_goal_response_callback
        )

    # =============================================================
    # AUTO_SCAN POVRATAK - ACCEPT
    # =============================================================

    def auto_scan_return_goal_response_callback(
        self,
        future
    ):

        try:

            goal_handle = future.result()

        except Exception as e:

            self.get_logger().error(
                f'Greška pri prihvatanju povratnog '
                f'MoveIt cilja: {e}'
            )

            return

        if not goal_handle.accepted:

            self.get_logger().error(
                'MoveIt JE ODBIO POVRATAK NA AUTO_SCAN POZICIJU!'
            )

            return

        self.get_logger().info(
            'Povratni MoveIt cilj prihvaćen.'
        )

        self.get_logger().info(
            'Robot se vraća na AUTO_SCAN poziciju...'
        )

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.auto_scan_return_result_callback
        )

    # =============================================================
    # AUTO_SCAN POVRATAK - RESULT
    # =============================================================

    def auto_scan_return_result_callback(
        self,
        future
    ):

        try:

            result = future.result().result

            error_code = result.error_code.val

        except Exception as e:

            self.get_logger().error(
                f'Greška pri čitanju rezultata '
                f'povratka na AUTO_SCAN: {e}'
            )

            return

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            f'Povratak na AUTO_SCAN '
            f'MoveIt error_code = {error_code}'
        )

        self.get_logger().info(
            '========================================'
        )

        if error_code != 1:

            # -----------------------------------------------------
            # AUTO_SCAN RETRY
            # -----------------------------------------------------
            # Povratak može povremeno vratiti CONTROL_FAILED (-4).
            # Ne menjamo cilj niti pravimo novu putanju — samo
            # ponavljamo isti joint cilj jednom.
            if (
                error_code == -4 and
                self.auto_scan_retry_count < 1
            ):

                self.auto_scan_retry_count += 1

                self.get_logger().warn(
                    'AUTO_SCAN povratak je vratio -4 '
                    '(CONTROL_FAILED).'
                )
                self.get_logger().warn(
                    'Pokušavam ponovo ISTI AUTO_SCAN cilj.'
                )

                # Kratka pauza da kontroler završi prethodni ciklus.
                time.sleep(0.5)

                self.return_to_auto_scan()
                return

            self.get_logger().error(
                'Robot NIJE uspeo da se vrati '
                'na AUTO_SCAN poziciju.'
            )

            return

        # Povratak uspeo. Sledeća detekcija treba da bira samo
        # kockice koje još nisu završene.
        self.auto_scan_retry_count = 0
        self.goal_sent = False
        self.phase = 16

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'ROBOT JE NA AUTO_SCAN POZICIJI.'
        )

        remaining_count = max(
            0,
            self.target_cube_count - len(self.completed_gazebo_ids)
        ) if self.target_cube_count is not None else 0

        self.get_logger().info(
            f'ZAVRŠENO KOCKICA: '
            f'{len(self.completed_gazebo_ids)}/{self.target_cube_count}'
        )

        if remaining_count > 0:
            self.get_logger().info(
                f'PREOSTALO: {remaining_count}'
            )
            self.get_logger().info(
                'Čekam sledeći frame i biram '
                'najbližu preostalu kockicu.'
            )
        else:
            self.get_logger().info(
                'SVE KOCKICE SU SORTIRANE.'
            )

        self.get_logger().info(
            '========================================'
        )

    # =============================================================
    # DETACH
    # =============================================================

    def detach_cube(self):

        if self.real_cube_id is None:

            self.get_logger().error(
                'Nema cube ID-a za detach!'
            )

            return

        publisher = (
            self.detach_publishers.get(
                self.real_cube_id
            )
        )

        if publisher is None:

            self.get_logger().error(
                f'Nema detach publishera za '
                f'{self.real_cube_id}!'
            )

            return

        self.get_logger().info(
            '----------------------------------------'
        )

        self.get_logger().info(
            'DETACH KOCKICE'
        )

        self.get_logger().info(
            f'Cube: {self.real_cube_id}'
        )

        self.get_logger().info(
            f'Topic: '
            f'/gz/{self.real_cube_id}/detach'
        )

        msg = Empty()

        publisher.publish(msg)

        self.get_logger().info(
            'DETACH PORUKA POSLATA!'
        )

        self.get_logger().info(
            'Kockica je sada slobodna.'
        )

        # =========================================================
        # EVIDENCIJA ZAVRŠENE KOCKICE
        # =========================================================
        if self.real_cube_id is not None:
            self.completed_gazebo_ids.add(
                self.real_cube_id
            )

        self.expected_gazebo_id = None

        color = self.detected_color.strip().upper()

        if color in self.placed_counts:
            self.placed_counts[color] += 1
        else:
            self.placed_counts[color] = 1

        self.get_logger().info(
            '----------------------------------------'
        )

        self.get_logger().info(
            'KOCKICA JE EVIDENTIRANA KAO ZAVRŠENA.'
        )

        self.get_logger().info(
            f'Boja: {color}'
        )

        self.get_logger().info(
            f'Ukupno {color} na površini: '
            f'{self.placed_counts[color]}'
        )

        self.get_logger().info(
            f'Ukupno završeno: '
            f'{len(self.completed_gazebo_ids)}/{self.target_cube_count}'
        )

        # =========================================================
        # ODMAH POSLE DETACH-a -> POVRATAK NA AUTO_SCAN
        # =========================================================
        #
        # Ovde NE čekamo novi MoveIt result callback.
        # Detach je ROS/Gazebo topic poruka, pa se nakon slanja
        # ne poziva result_callback() ponovo.
        #
        # Zato direktno prelazimo na fazu povratka.
        #

        self.phase = 15

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'DETACH ZAVRŠEN.'
        )

        self.get_logger().info(
            'VRAĆAM ROBOTA NA AUTO_SCAN POZICIJU.'
        )

        self.get_logger().info(
            '========================================'
        )

        self.return_to_auto_scan()

    # =============================================================
    # MAIN
    # =============================================================

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