import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from cube_interfaces.srv import GetClosestCube


class CubeTrackerNode(Node):
    def __init__(self):
        super().__init__('cube_tracker_node')

        # Declare and retrieve the NUM_CUBES parameter (defaulting to 5 if not passed)
        self.declare_parameter('NUM_CUBES', 5)
        self.num_cubes = self.get_parameter('NUM_CUBES').get_parameter_value().integer_value

        # Internal storage for the latest positions: { 'cube_0': Vector3, 'cube_1': Vector3, ... }
        self.cube_positions = {}
        self.threshold = 0.02  # threshold in meters

        self.get_logger().info(f'Starting tracker for {self.num_cubes} cubes.')

        # Dynamically build subscribers based on the parameter count
        for i in range(self.num_cubes):
            cube_id = f'cube_{i}'
            topic_name = f'/model/{cube_id}/pose'

            # pass a default keyword argument (cid=cube_id) to the lambda function
            # to break Python's late-binding closures and lock in the correct ID string.
            self.create_subscription(
                TransformStamped,
                topic_name,
                lambda msg, cid=cube_id: self.cube_pose_callback(msg, cid),
                10
            )

        # Initialize the service server
        self.srv = self.create_service(
            GetClosestCube,
            'get_closest_cube',
            self.get_closest_cube_callback
        )
        self.get_logger().info('Service "/get_closest_cube" is online and listening.')

    def cube_pose_callback(self, msg, cube_id):
        """Stores the latest translation coordinates for the specific cube."""
        self.cube_positions[cube_id] = msg.transform.translation

    def get_closest_cube_callback(self, request, response):
        """Processes incoming service queries to find the nearest tracked cube."""
        target = request.target_pose.position
        closest_id = ""
        min_dist = float('inf')

        # Ensure we actually have received data before computing
        if not self.cube_positions:
            response.cube_id = ""
            response.found = False
            self.get_logger().warn("Service requested, but no cube pose data has been received yet!")
            return response

        # Compute Euclidean distance to all tracked cubes
        for cube_id, pos in self.cube_positions.items():
            dist = math.sqrt(
                (target.x - pos.x)**2 +
                (target.y - pos.y)**2 +
                (target.z - pos.z)**2
            )
            if dist < min_dist:
                min_dist = dist
                closest_id = cube_id

        # Evaluate findings against threshold constraint
        if min_dist <= self.threshold:
            response.cube_id = closest_id
            response.found = True
            self.get_logger().info(f"Target found! Closest: {closest_id} (Distance: {min_dist:.3f}m)")
        else:
            response.cube_id = ""
            response.found = False
            self.get_logger().info(f"No cube within threshold. Nearest was {closest_id} at {min_dist:.3f}m")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = CubeTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
