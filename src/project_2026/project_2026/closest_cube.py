#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from cube_interfaces.srv import GetClosestCube


class CubeTracker(Node):

    def __init__(self):

        super().__init__('cube_tracker_node')

        # =====================================================
        # PODEŠAVANJA
        # =====================================================

        self.NUM_CUBES = 5

        # Maksimalna dozvoljena udaljenost između
        # target_pose i centra kockice.
        #
        # Za testiranje koristimo malo veći threshold.
        self.threshold = 0.10

        # =====================================================
        # ROBOT - WORLD POZICIJA
        # =====================================================

        # UF_ROBOT se u task.launch.py nalazi na:
        #
        # x = -0.200
        # y = -0.500
        # z =  1.021
        #
        # link_base je root link robota.

        self.robot_world_x = -0.200000
        self.robot_world_y = -0.500000
        self.robot_world_z = 1.021000

        # =====================================================
        # ROBOT - WORLD ORIJENTACIJA
        # =====================================================

        # UF_ROBOT je zarotiran +90 stepeni oko Z ose.
        #
        # Za WORLD -> link_base koristimo -90 stepeni.

        self.robot_yaw = math.pi / 2.0

        # =====================================================
        # POZICIJE KOCKICA
        # =====================================================

        self.cube_positions = {}

        for i in range(self.NUM_CUBES):

            cube_id = f'cube_{i}'

            topic = f'/model/{cube_id}/pose'

            self.create_subscription(
                TransformStamped,
                topic,
                lambda msg, cid=cube_id:
                self.cube_pose_callback(msg, cid),
                10
            )

        # =====================================================
        # SERVICE
        # =====================================================

        self.service = self.create_service(
            GetClosestCube,
            '/get_closest_cube',
            self.get_closest_cube_callback
        )

        # =====================================================
        # LOG
        # =====================================================

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            '       CLOSEST CUBE TRACKER'
        )

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            f'Broj kockica: {self.NUM_CUBES}'
        )

        self.get_logger().info(
            f'Robot WORLD position: '
            f'({self.robot_world_x:.3f}, '
            f'{self.robot_world_y:.3f}, '
            f'{self.robot_world_z:.3f})'
        )

        self.get_logger().info(
            'Robot WORLD yaw: +90 deg'
        )

        self.get_logger().info(
            'WORLD -> link_base transform: ENABLED'
        )

        self.get_logger().info(
            f'Closest cube threshold: '
            f'{self.threshold:.3f} m'
        )

        self.get_logger().info(
            'Cekam podatke sa svih 5 cube topic-a...'
        )

        self.get_logger().info(
            'Service "/get_closest_cube" '
            'is online and listening.'
        )

        self.get_logger().info(
            '========================================'
        )

    # =========================================================
    # CUBE POSE CALLBACK
    # =========================================================

    def cube_pose_callback(
        self,
        msg: TransformStamped,
        cube_id: str
    ):

        x = msg.transform.translation.x
        y = msg.transform.translation.y
        z = msg.transform.translation.z

        self.cube_positions[cube_id] = (
            x,
            y,
            z
        )

    # =========================================================
    # WORLD -> LINK_BASE
    # =========================================================

    def world_to_link_base(
        self,
        world_x,
        world_y,
        world_z
    ):

        # -----------------------------------------------------
        # 1. TRANSLACIJA
        # -----------------------------------------------------

        dx = (
            world_x -
            self.robot_world_x
        )

        dy = (
            world_y -
            self.robot_world_y
        )

        dz = (
            world_z -
            self.robot_world_z
        )

        # -----------------------------------------------------
        # 2. ROTACIJA
        # -----------------------------------------------------

        # Robot ima +90 deg yaw u WORLD-u.
        #
        # WORLD -> BASE je zato -90 deg.
        #
        # x_base = dy
        # y_base = -dx
        #
        # z ostaje isti.

        base_x = dy
        base_y = -dx
        base_z = dz

        return (
            base_x,
            base_y,
            base_z
        )

    # =========================================================
    # GET CLOSEST CUBE
    # =========================================================

    def get_closest_cube_callback(
        self,
        request,
        response
    ):

        # =====================================================
        # PROVERA KOCKICA
        # =====================================================

        if len(self.cube_positions) < self.NUM_CUBES:

            self.get_logger().warn(
                f'Nisu primljene sve kockice: '
                f'{len(self.cube_positions)}/'
                f'{self.NUM_CUBES}'
            )

            response.cube_id = ''
            response.found = False

            return response

        # =====================================================
        # TARGET - LINK_BASE
        # =====================================================

        target_x = request.target_pose.position.x
        target_y = request.target_pose.position.y
        target_z = request.target_pose.position.z

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'GET CLOSEST CUBE'
        )

        self.get_logger().info(
            f'Target LINK_BASE XYZ = '
            f'({target_x:.4f}, '
            f'{target_y:.4f}, '
            f'{target_z:.4f})'
        )

        # =====================================================
        # TRAŽENJE NAJBLIŽE KOCKICE
        # =====================================================

        closest_cube = None
        closest_distance = float('inf')

        for cube_id in sorted(self.cube_positions.keys()):

            # -------------------------------------------------
            # WORLD POZICIJA
            # -------------------------------------------------

            world_x, world_y, world_z = (
                self.cube_positions[cube_id]
            )

            # -------------------------------------------------
            # WORLD -> LINK_BASE
            # -------------------------------------------------

            base_x, base_y, base_z = (
                self.world_to_link_base(
                    world_x,
                    world_y,
                    world_z
                )
            )

            # -------------------------------------------------
            # DISTANCE
            # -------------------------------------------------

            dx = base_x - target_x
            dy = base_y - target_y
            dz = base_z - target_z

            distance = math.sqrt(
                dx * dx +
                dy * dy +
                dz * dz
            )

            # -------------------------------------------------
            # DEBUG
            # -------------------------------------------------

            self.get_logger().info(
                f'{cube_id}: '
                f'WORLD=('
                f'{world_x:.4f}, '
                f'{world_y:.4f}, '
                f'{world_z:.4f}) '
                f'-> '
                f'LINK_BASE=('
                f'{base_x:.4f}, '
                f'{base_y:.4f}, '
                f'{base_z:.4f}) '
                f'distance={distance:.4f} m'
            )

            # -------------------------------------------------
            # MINIMUM
            # -------------------------------------------------

            if distance < closest_distance:

                closest_distance = distance
                closest_cube = cube_id

        # =====================================================
        # NEMA REZULTATA
        # =====================================================

        if closest_cube is None:

            response.cube_id = ''
            response.found = False

            self.get_logger().error(
                'Nijedna kockica nije pronađena.'
            )

            return response

        # =====================================================
        # THRESHOLD
        # =====================================================

        if closest_distance > self.threshold:

            response.cube_id = ''
            response.found = False

            self.get_logger().warn(
                f'Najbliža kockica je {closest_cube}, '
                f'ali je udaljena '
                f'{closest_distance:.4f} m '
                f'(threshold={self.threshold:.4f} m).'
            )

            self.get_logger().warn(
                'Target i detektovane kockice verovatno '
                'nisu u istom koordinatnom sistemu.'
            )

            self.get_logger().info(
                '========================================'
            )

            return response

        # =====================================================
        # USPEH
        # =====================================================

        response.cube_id = closest_cube
        response.found = True

        world_x, world_y, world_z = (
            self.cube_positions[closest_cube]
        )

        base_x, base_y, base_z = (
            self.world_to_link_base(
                world_x,
                world_y,
                world_z
            )
        )

        # =====================================================
        # LOG
        # =====================================================

        self.get_logger().info(
            '----------------------------------------'
        )

        self.get_logger().info(
            'NAJBLIŽA KOCKICA PRONAĐENA!'
        )

        self.get_logger().info(
            f'Cube ID: {closest_cube}'
        )

        self.get_logger().info(
            f'WORLD XYZ: '
            f'({world_x:.4f}, '
            f'{world_y:.4f}, '
            f'{world_z:.4f})'
        )

        self.get_logger().info(
            f'LINK_BASE XYZ: '
            f'({base_x:.4f}, '
            f'{base_y:.4f}, '
            f'{base_z:.4f})'
        )

        self.get_logger().info(
            f'Distance to target: '
            f'{closest_distance:.4f} m'
        )

        self.get_logger().info(
            f'Result: {closest_cube}'
        )

        self.get_logger().info(
            '========================================'
        )

        return response


# =============================================================
# MAIN
# =============================================================

def main(args=None):

    rclpy.init(args=args)

    node = CubeTracker()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()