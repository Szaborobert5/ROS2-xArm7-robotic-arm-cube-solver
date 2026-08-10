import math

import rclpy
from rclpy.node import Node

from cube_interfaces.msg import CubeDetectionArray


class CubeOrderNode(Node):

    def __init__(self):
        super().__init__('cube_order')

        self.subscription = self.create_subscription(
            CubeDetectionArray,
            '/detected_cubes',
            self.cubes_callback,
            10
        )

        self.last_order = None

        self.get_logger().info('========================================')
        self.get_logger().info('          CUBE ORDER')
        self.get_logger().info('========================================')
        self.get_logger().info('Cekam podatke sa /detected_cubes...')

    def cubes_callback(self, msg):

        # Nemamo jos nijednu kockicu
        if len(msg.cubes) == 0:
            return

        cubes = []

        # -------------------------------------------------
        # Uzimamo sve detektovane kockice
        # -------------------------------------------------

        for cube in msg.cubes:

            x = cube.x
            y = cube.y
            z = cube.z

            # Udaljenost od baze robota
            distance = math.sqrt(
                x ** 2 +
                y ** 2 +
                z ** 2
            )

            cubes.append({
                'id': cube.id,
                'color': cube.color,
                'x': x,
                'y': y,
                'z': z,
                'distance': distance
            })

        # -------------------------------------------------
        # Sortiranje od najbliže ka najdaljoj
        # -------------------------------------------------

        cubes.sort(
            key=lambda cube: cube['distance']
        )

        # Redosled ID-jeva
        current_order = tuple(
            cube['id'] for cube in cubes
        )

        # Ako je redosled isti kao prošli put,
        # nema potrebe da ponovo ispisujemo sve.
        if current_order == self.last_order:
            return

        self.last_order = current_order

        # -------------------------------------------------
        # ISPIS
        # -------------------------------------------------

        self.get_logger().info('')
        self.get_logger().info('========================================')
        self.get_logger().info('          REDOSLED KOCKICA')
        self.get_logger().info('========================================')

        for i, cube in enumerate(cubes, start=1):

            self.get_logger().info(
                f'{i}. Cube #{cube["id"]} | '
                f'{cube["color"]}'
            )

            self.get_logger().info(
                f'   XYZ: '
                f'({cube["x"]:.3f}, '
                f'{cube["y"]:.3f}, '
                f'{cube["z"]:.3f})'
            )

            self.get_logger().info(
                f'   Distance: '
                f'{cube["distance"]:.3f} m'
            )

        self.get_logger().info('========================================')

        # -------------------------------------------------
        # Prva kockica
        # -------------------------------------------------

        closest = cubes[0]

        self.get_logger().info(
            f'NAJBLIŽA KOCKICA: '
            f'#{closest["id"]} | {closest["color"]}'
        )

        self.get_logger().info(
            f'XYZ: '
            f'({closest["x"]:.3f}, '
            f'{closest["y"]:.3f}, '
            f'{closest["z"]:.3f})'
        )

        self.get_logger().info(
            f'Udaljenost: '
            f'{closest["distance"]:.3f} m'
        )

        self.get_logger().info('========================================')


def main(args=None):

    rclpy.init(args=args)

    node = CubeOrderNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()