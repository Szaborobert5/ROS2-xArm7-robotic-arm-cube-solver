import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Empty

from cube_interfaces.msg import CubeDetectionArray

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint
)

from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import SolidPrimitive


class MoveToCube(Node):

    def __init__(self):

        super().__init__('move_to_cube')

        # =====================================================
        # DETEKTOVANE KOCKICE
        # =====================================================

        self.cube_subscription = self.create_subscription(
            CubeDetectionArray,
            '/detected_cubes',
            self.cubes_callback,
            10
        )

        # =====================================================
        # MOVEIT
        # =====================================================

        self.move_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

        # =====================================================
        # ATTACH PUBLISHERS
        # =====================================================

        self.attach_publishers = {}

        for i in range(5):

            cube_id = f'cube_{i}'

            topic_name = f'/gz/{cube_id}/attach'

            self.attach_publishers[cube_id] = self.create_publisher(
                Empty,
                topic_name,
                10
            )

        # =====================================================
        # STANJE
        # =====================================================

        self.goal_sent = False

        # Faze kretanja:
        #
        # 0 = još ništa
        # 1 = stigao 15 cm iznad
        # 2 = stigao 3 cm iznad
        # 3 = attach izvršen
        # 4 = kockica podignuta
        #
        self.phase = 0

        # =====================================================
        # PODACI O KOCKICI
        # =====================================================

        self.cube_id = None
        self.cube_color = None

        self.cube_x = None
        self.cube_y = None
        self.cube_z = None

        # =====================================================
        # START
        # =====================================================

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            '        MOVE TO CUBE START'
        )

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'Cekam podatke sa /detected_cubes...'
        )


    # =========================================================
    # DETEKCIJA KOCKICA
    # =========================================================

    def cubes_callback(self, msg):

        if self.goal_sent:
            return

        if len(msg.cubes) == 0:
            return

        if len(msg.cubes) < 5:

            self.get_logger().info(
                f'Detekcija: {len(msg.cubes)}/5'
            )

            return

        # -----------------------------------------------------
        # SPRECAVAMO DA SE CILJ POSALJE VISE PUTA
        # -----------------------------------------------------

        self.goal_sent = True

        self.get_logger().info(
            '5/5 KOCKICA DETEKTOVANO!'
        )

        # =====================================================
        # PRONALAZIMO NAJBLIŽU KOCKICU
        # =====================================================

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
            key=lambda cube: cube['distance']
        )

        closest = cubes[0]

        # =====================================================
        # SACUVAJ PODATKE
        # =====================================================

        self.cube_id = f'cube_{closest["id"] - 1}'
        self.cube_color = closest['color']

        self.cube_x = closest['x']
        self.cube_y = closest['y']
        self.cube_z = closest['z']

        # =====================================================
        # ISPIS
        # =====================================================

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            f'NAJBLIŽA KOCKICA: #{self.cube_id}'
        )

        self.get_logger().info(
            f'BOJA: {self.cube_color}'
        )

        self.get_logger().info(
            f'Kockica XYZ: '
            f'({self.cube_x:.3f}, '
            f'{self.cube_y:.3f}, '
            f'{self.cube_z:.3f})'
        )

        self.get_logger().info(
            f'Udaljenost: '
            f'{closest["distance"]:.3f} m'
        )

        # =====================================================
        # PRVI CILJ
        #
        # 15 cm IZNAD KOCKICE
        # =====================================================

        target_z = self.cube_z + 0.15

        self.get_logger().info(
            '----------------------------------------'
        )

        self.get_logger().info(
            'PRVI CILJ - 15 cm IZNAD KOCKICE'
        )

        self.get_logger().info(
            f'X = {self.cube_x:.3f}'
        )

        self.get_logger().info(
            f'Y = {self.cube_y:.3f}'
        )

        self.get_logger().info(
            f'Z = {target_z:.3f}'
        )

        self.get_logger().info(
            '========================================'
        )

        # =====================================================
        # ŠALJI PRVI MOVEIT CILJ
        # =====================================================

        self.send_move_goal(
            self.cube_x,
            self.cube_y,
            target_z
        )


    # =========================================================
    # MOVEIT CILJ
    # =========================================================

    def send_move_goal(
        self,
        x,
        y,
        z
    ):

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

        self.get_logger().info(
            'MoveIt je dostupan.'
        )

        # =====================================================
        # MOVE GROUP
        # =====================================================

        goal_msg = MoveGroup.Goal()

        goal_msg.request.group_name = 'xarm7'

        # =====================================================
        # BRZINA
        # =====================================================

        goal_msg.request.max_velocity_scaling_factor = 0.4

        goal_msg.request.max_acceleration_scaling_factor = 0.4

        # =====================================================
        # PLANNING
        # =====================================================

        goal_msg.planning_options.plan_only = False

        goal_msg.planning_options.replan = True

        goal_msg.planning_options.replan_attempts = 2

        # =====================================================
        # POSITION CONSTRAINT
        # =====================================================

        position_constraint = PositionConstraint()

        position_constraint.header.frame_id = 'link_base'

        position_constraint.link_name = 'link_tcp'

        primitive = SolidPrimitive()

        primitive.type = SolidPrimitive.SPHERE

        # Malo veća tolerancija nego ranije.
        # Ovo pomaže da MoveIt lakše pronađe put.

        primitive.dimensions = [0.015]

        position_constraint.constraint_region.primitives.append(
            primitive
        )

        target_point = Point()

        target_point.x = x
        target_point.y = y
        target_point.z = z

        position_constraint.constraint_region.primitive_poses.append(
            Pose(
                position=target_point
            )
        )

        position_constraint.weight = 1.0

        # =====================================================
        # ORIENTATION
        # =====================================================

        orientation_constraint = OrientationConstraint()

        orientation_constraint.header.frame_id = 'link_base'

        orientation_constraint.link_name = 'link_tcp'

        # TCP okrenut prema dole

        orientation_constraint.orientation.x = 1.0
        orientation_constraint.orientation.y = 0.0
        orientation_constraint.orientation.z = 0.0
        orientation_constraint.orientation.w = 0.0

        orientation_constraint.absolute_x_axis_tolerance = 0.20
        orientation_constraint.absolute_y_axis_tolerance = 0.20
        orientation_constraint.absolute_z_axis_tolerance = 0.20

        orientation_constraint.weight = 1.0

        # =====================================================
        # CONSTRAINTS
        # =====================================================

        constraints = Constraints()

        constraints.position_constraints.append(
            position_constraint
        )

        constraints.orientation_constraints.append(
            orientation_constraint
        )

        goal_msg.request.goal_constraints.append(
            constraints
        )

        # =====================================================
        # PLANNING TIME
        # =====================================================

        goal_msg.request.allowed_planning_time = 3.0

        # =====================================================
        # SEND
        # =====================================================

        self.get_logger().info(
            'Saljem MoveIt cilj...'
        )

        future = self.move_client.send_goal_async(
            goal_msg
        )

        future.add_done_callback(
            self.goal_response_callback
        )


    # =========================================================
    # MOVEIT GOAL RESPONSE
    # =========================================================

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.get_logger().error(
                'MoveIt je odbio cilj!'
            )

            return

        self.get_logger().info(
            'Cilj prihvacen. Robot se pomera...'
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.result_callback
        )


    # =========================================================
    # MOVEIT RESULT
    # =========================================================

    def result_callback(self, future):

        result = future.result().result

        error_code = result.error_code.val

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            f'MoveIt zavrsio. '
            f'error_code = {error_code}'
        )

        # =====================================================
        # USPEŠNO KRETANJE
        # =====================================================

        if error_code == 1:

            # =================================================
            # FAZA 0 -> 1
            #
            # STIGAO 15 cm IZNAD
            # =================================================

            if self.phase == 0:

                self.phase = 1

                self.get_logger().info(
                    'ROBOT JE STIGAO 15 cm IZNAD KOCKICE.'
                )

                # -------------------------------------------------
                # DRUGI CILJ
                #
                # 3 cm IZNAD KOCKICE
                # -------------------------------------------------

                descend_z = self.cube_z + 0.03

                self.get_logger().info(
                    '----------------------------------------'
                )

                self.get_logger().info(
                    'DRUGI CILJ - 3 cm IZNAD KOCKICE'
                )

                self.get_logger().info(
                    f'X = {self.cube_x:.3f}'
                )

                self.get_logger().info(
                    f'Y = {self.cube_y:.3f}'
                )

                self.get_logger().info(
                    f'Z = {descend_z:.3f}'
                )

                self.get_logger().info(
                    'Robot se sada spusta...'
                )

                self.send_move_goal(
                    self.cube_x,
                    self.cube_y,
                    descend_z
                )

            # =================================================
            # FAZA 1 -> 2
            #
            # STIGAO 3 cm IZNAD
            # =================================================

            elif self.phase == 1:

                self.phase = 2

                self.get_logger().info(
                    'ROBOT JE STIGAO 3 cm IZNAD KOCKICE.'
                )

                self.get_logger().info(
                    'SPUSTANJE USPESNO ZAVRSENO.'
                )

                # -------------------------------------------------
                # ATTACH
                # -------------------------------------------------

                self.attach_cube()

            # =================================================
            # FAZA 3 -> 4
            #
            # KOCKICA PODIGNUTA
            # =================================================

            elif self.phase == 3:

                self.phase = 4

                self.get_logger().info(
                    '========================================'
                )

                self.get_logger().info(
                    'KOCKICA JE USPESNO PODIGNUTA!'
                )

                self.get_logger().info(
                    f'KOCKICA: {self.cube_id}'
                )

                self.get_logger().info(
                    f'BOJA: {self.cube_color}'
                )

                self.get_logger().info(
                    '========================================'
                )

        # =====================================================
        # NEUSPEH
        # =====================================================

        else:

            self.get_logger().error(
                'MoveIt nije uspesno zavrsio kretanje.'
            )

            self.get_logger().error(
                f'Faza u kojoj je doslo do problema: '
                f'{self.phase}'
            )

        self.get_logger().info(
            '========================================'
        )


    # =========================================================
    # ATTACH KOCKICE
    # =========================================================

    def attach_cube(self):

        self.get_logger().info(
            '----------------------------------------'
        )

        self.get_logger().info(
            'ATTACH KOCKICE'
        )

        self.get_logger().info(
            f'Kockica: {self.cube_id}'
        )

        self.get_logger().info(
            f'Topic: /gz/{self.cube_id}/attach'
        )

        # =====================================================
        # PROVERA DA LI POSTOJI PUBLISHER
        # =====================================================

        publisher = self.attach_publishers.get(
            self.cube_id
        )

        if publisher is None:

            self.get_logger().error(
                f'Nema attach publisher za '
                f'{self.cube_id}!'
            )

            return

        # =====================================================
        # POŠALJI EMPTY PORUKU
        # =====================================================

        msg = Empty()

        publisher.publish(msg)

        self.get_logger().info(
            'ATTACH PORUKA POSLATA!'
        )

        self.get_logger().info(
            'Kockica je sada spojena sa TCP-om.'
        )

        # =====================================================
        # PRELAZIMO NA PODIZANJE
        # =====================================================

        self.phase = 3

        lift_z = self.cube_z + 0.20

        self.get_logger().info(
            '----------------------------------------'
        )

        self.get_logger().info(
            'TREĆI CILJ - PODIZANJE KOCKICE'
        )

        self.get_logger().info(
            f'X = {self.cube_x:.3f}'
        )

        self.get_logger().info(
            f'Y = {self.cube_y:.3f}'
        )

        self.get_logger().info(
            f'Z = {lift_z:.3f}'
        )

        self.get_logger().info(
            'Robot sada podize kockicu...'
        )

        self.get_logger().info(
            '========================================'
        )

        # =====================================================
        # MOVE UP
        # =====================================================

        self.send_move_goal(
            self.cube_x,
            self.cube_y,
            lift_z
        )


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