import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge

from cube_interfaces.msg import CubeDetection, CubeDetectionArray

import tf2_ros
from tf2_geometry_msgs import do_transform_point

import cv2
import numpy as np


class CubeDetector(Node):

    def __init__(self):

        super().__init__('cube_detector')

        self.bridge = CvBridge()

        # =====================================================
        # TF2
        # =====================================================

        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # =====================================================
        # FRAME-OVI
        # =====================================================

        self.camera_frame = 'camera_depth_optical_frame'
        self.robot_frame = 'link_base'

        # =====================================================
        # PODACI KAMERE
        # =====================================================

        self.latest_depth = None
        self.camera_info = None

        # =====================================================
        # RGB IMAGE
        # =====================================================

        self.color_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.color_callback,
            10
        )

        # =====================================================
        # DEPTH IMAGE
        # =====================================================

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/depth_image',
            self.depth_callback,
            10
        )

        # =====================================================
        # CAMERA INFO
        # =====================================================

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/depth/camera_info',
            self.camera_info_callback,
            10
        )

        # =====================================================
        # CUBE COUNT
        # =====================================================

        self.cube_count_pub = self.create_publisher(
            Int32,
            '/cube_count',
            10
        )

        # =====================================================
        # DETECTED CUBES
        # =====================================================

        self.cubes_pub = self.create_publisher(
            CubeDetectionArray,
            '/detected_cubes',
            10
        )

        # =====================================================
        # START MESSAGE
        # =====================================================

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'Cube Detector + TF2 pokrenut'
        )

        self.get_logger().info(
            f'Camera frame: {self.camera_frame}'
        )

        self.get_logger().info(
            f'Robot frame: {self.robot_frame}'
        )

        self.get_logger().info(
            '========================================'
        )

    # =========================================================
    # CAMERA INFO CALLBACK
    # =========================================================

    def camera_info_callback(self, msg):

        self.camera_info = msg

    # =========================================================
    # DEPTH CALLBACK
    # =========================================================

    def depth_callback(self, msg):

        try:

            self.latest_depth = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='32FC1'
            )

        except Exception as e:

            self.get_logger().error(
                f'Greska pri konverziji depth slike: {e}'
            )

    # =========================================================
    # COLOR CALLBACK
    # =========================================================

    def color_callback(self, msg):

        try:

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

        except Exception as e:

            self.get_logger().error(
                f'Greska pri konverziji RGB slike: {e}'
            )

            return

        # =====================================================
        # PROVERA DEPTH
        # =====================================================

        if self.latest_depth is None:
            return

        # =====================================================
        # PROVERA CAMERA INFO
        # =====================================================

        if self.camera_info is None:
            return

        # =====================================================
        # RGB -> HSV
        # =====================================================

        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )

        # =====================================================
        # RED
        # =====================================================

        red_lower_1 = np.array(
            [0, 100, 80]
        )

        red_upper_1 = np.array(
            [10, 255, 255]
        )

        red_lower_2 = np.array(
            [170, 100, 80]
        )

        red_upper_2 = np.array(
            [179, 255, 255]
        )

        red_mask_1 = cv2.inRange(
            hsv,
            red_lower_1,
            red_upper_1
        )

        red_mask_2 = cv2.inRange(
            hsv,
            red_lower_2,
            red_upper_2
        )

        red_mask = cv2.bitwise_or(
            red_mask_1,
            red_mask_2
        )

        # =====================================================
        # GREEN
        # =====================================================

        green_lower = np.array(
            [35, 80, 70]
        )

        green_upper = np.array(
            [85, 255, 255]
        )

        green_mask = cv2.inRange(
            hsv,
            green_lower,
            green_upper
        )

        # =====================================================
        # BLUE
        # =====================================================

        blue_lower = np.array(
            [90, 80, 50]
        )

        blue_upper = np.array(
            [140, 255, 255]
        )

        blue_mask = cv2.inRange(
            hsv,
            blue_lower,
            blue_upper
        )

        # =====================================================
        # DETEKCIJE
        # =====================================================

        detections = []

        self.find_all_cubes(
            red_mask,
            'RED',
            detections
        )

        self.find_all_cubes(
            green_mask,
            'GREEN',
            detections
        )

        self.find_all_cubes(
            blue_mask,
            'BLUE',
            detections
        )

        # =====================================================
        # SORTIRANJE
        # =====================================================

        detections.sort(
            key=lambda d: (d['cy'], d['cx'])
        )

        # Maksimalno 5 kockica

        detections = detections[:5]

        # =====================================================
        # CUBE COUNT
        # =====================================================

        count_msg = Int32()

        count_msg.data = len(detections)

        self.cube_count_pub.publish(
            count_msg
        )

        # =====================================================
        # PROCESS EACH CUBE
        # =====================================================

        for index, detection in enumerate(
            detections,
            start=1
        ):

            detection['id'] = index

            self.process_detection(
                frame,
                detection
            )

        # =====================================================
        # DETECTED CUBES MESSAGE
        # =====================================================

        detected_msg = CubeDetectionArray()

        for detection in detections:

            # Ako TF nije uspeo,
            # nemoj slati kockicu

            if (
                'base_X' not in detection or
                'base_Y' not in detection or
                'base_Z' not in detection
            ):
                continue

            cube_msg = CubeDetection()

            cube_msg.id = detection['id']

            cube_msg.color = detection['color']

            # Koordinate su u link_base frame-u

            cube_msg.x = detection['base_X']
            cube_msg.y = detection['base_Y']
            cube_msg.z = detection['base_Z']

            detected_msg.cubes.append(
                cube_msg
            )

        self.cubes_pub.publish(
            detected_msg
        )

        # =====================================================
        # STATUS
        # =====================================================

        cv2.putText(
            frame,
            f'Cubes detected: {len(detections)}/5',
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2
        )

        # =====================================================
        # DISPLAY
        # =====================================================

        cv2.imshow(
            'Cube Detector - RGB + Depth + TF',
            frame
        )

        cv2.waitKey(1)

    # =========================================================
    # FIND ALL CUBES
    # =========================================================

    def find_all_cubes(
        self,
        mask,
        color_name,
        detections
    ):

        # =====================================================
        # MORPHOLOGICAL FILTERING
        # =====================================================

        kernel = np.ones(
            (5, 5),
            np.uint8
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        # =====================================================
        # CONTOURS
        # =====================================================

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # =====================================================
        # PROCESS CONTOURS
        # =====================================================

        for contour in contours:

            area = cv2.contourArea(
                contour
            )

            if area < 250:
                continue

            x, y, w, h = cv2.boundingRect(
                contour
            )

            if h == 0:
                continue

            aspect_ratio = w / float(h)

            if aspect_ratio < 0.35:
                continue

            if aspect_ratio > 2.5:
                continue

            # =================================================
            # CENTROID
            # =================================================

            M = cv2.moments(
                contour
            )

            if M['m00'] == 0:
                continue

            cx = int(
                M['m10'] / M['m00']
            )

            cy = int(
                M['m01'] / M['m00']
            )

            detections.append({

                'color': color_name,

                'cx': cx,
                'cy': cy,

                'x': x,
                'y': y,

                'w': w,
                'h': h,

                'area': area
            })

    # =========================================================
    # PROCESS DETECTION
    # ROBUST DEPTH VERSION
    # =========================================================

    def process_detection(
        self,
        frame,
        detection
    ):

        cube_id = detection['id']
        color = detection['color']

        cx = detection['cx']
        cy = detection['cy']

        x = detection['x']
        y = detection['y']

        w = detection['w']
        h = detection['h']

        # =====================================================
        # DEPTH DIMENSIONS
        # =====================================================

        depth_height, depth_width = (
            self.latest_depth.shape
        )

        if (
            cx < 0 or
            cx >= depth_width or
            cy < 0 or
            cy >= depth_height
        ):

            self.get_logger().warn(
                f'Cube #{cube_id}: '
                'centar van depth slike.'
            )

            return

        # =====================================================
        # INNER BOUNDING BOX
        #
        # Koristimo unutrasnjih 60%
        # bounding box-a.
        # =====================================================

        inner_x1 = int(
            x + 0.20 * w
        )

        inner_x2 = int(
            x + 0.80 * w
        )

        inner_y1 = int(
            y + 0.20 * h
        )

        inner_y2 = int(
            y + 0.80 * h
        )

        # =====================================================
        # CLAMP
        # =====================================================

        inner_x1 = max(
            0,
            min(
                inner_x1,
                depth_width - 1
            )
        )

        inner_x2 = max(
            0,
            min(
                inner_x2,
                depth_width
            )
        )

        inner_y1 = max(
            0,
            min(
                inner_y1,
                depth_height - 1
            )
        )

        inner_y2 = max(
            0,
            min(
                inner_y2,
                depth_height
            )
        )

        if (
            inner_x2 <= inner_x1 or
            inner_y2 <= inner_y1
        ):

            self.get_logger().warn(
                f'Cube #{cube_id}: '
                'los bounding box.'
            )

            return

        # =====================================================
        # DEPTH REGION
        # =====================================================

        depth_region = self.latest_depth[
            inner_y1:inner_y2,
            inner_x1:inner_x2
        ]

        # =====================================================
        # VALID DEPTH
        #
        # 0.05m - 3m
        # =====================================================

        valid_depth = depth_region[
            np.isfinite(depth_region) &
            (depth_region > 0.05) &
            (depth_region < 3.0)
        ]

        if len(valid_depth) < 10:

            self.get_logger().warn(
                f'Cube #{cube_id} ({color}): '
                f'nema dovoljno validnih depth '
                f'podataka: {len(valid_depth)}'
            )

            return

        # =====================================================
        # REMOVE EXTREME OUTLIERS
        # =====================================================

        p05 = np.percentile(
            valid_depth,
            5
        )

        p80 = np.percentile(
            valid_depth,
            80
        )

        filtered_depth = valid_depth[
            (valid_depth >= p05) &
            (valid_depth <= p80)
        ]

        if len(filtered_depth) < 10:

            filtered_depth = valid_depth

        # =====================================================
        # MEDIAN DEPTH
        # =====================================================

        Z = float(
            np.median(
                filtered_depth
            )
        )

        # =====================================================
        # DEPTH STATISTICS
        # =====================================================

        depth_min = float(
            np.min(
                filtered_depth
            )
        )

        depth_max = float(
            np.max(
                filtered_depth
            )
        )

        depth_std = float(
            np.std(
                filtered_depth
            )
        )

        # =====================================================
        # CAMERA INTRINSICS
        # =====================================================

        fx = float(
            self.camera_info.k[0]
        )

        fy = float(
            self.camera_info.k[4]
        )

        camera_cx = float(
            self.camera_info.k[2]
        )

        camera_cy = float(
            self.camera_info.k[5]
        )

        if (
            fx <= 0 or
            fy <= 0
        ):

            self.get_logger().warn(
                f'Cube #{cube_id}: '
                'nevalidni camera intrinsics.'
            )

            return

        # =====================================================
        # CAMERA XYZ
        # =====================================================

        X = (
            (cx - camera_cx)
            * Z
            / fx
        )

        Y = (
            (cy - camera_cy)
            * Z
            / fy
        )

        detection['X'] = X
        detection['Y'] = Y
        detection['Z'] = Z

        # =====================================================
        # POINT STAMPED
        # =====================================================

        point_camera = PointStamped()

        point_camera.header.frame_id = (
            self.camera_frame
        )

        point_camera.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        point_camera.point.x = X
        point_camera.point.y = Y
        point_camera.point.z = Z

        # =====================================================
        # TF2
        # =====================================================

        try:

            if not self.tf_buffer.can_transform(
                self.robot_frame,
                self.camera_frame,
                rclpy.time.Time()
            ):

                self.get_logger().warn(
                    'TF nije trenutno dostupan: '
                    f'{self.robot_frame} <- '
                    f'{self.camera_frame}'
                )

                return

            # =================================================
            # LOOKUP TRANSFORM
            # =================================================

            transform = (
                self.tf_buffer.lookup_transform(
                    self.robot_frame,
                    self.camera_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(
                        seconds=0.2
                    )
                )
            )

            # =================================================
            # TRANSFORM POINT
            # =================================================

            point_base = do_transform_point(
                point_camera,
                transform
            )

            # =================================================
            # BASE XYZ
            # =================================================

            base_X = (
                point_base.point.x
            )

            base_Y = (
                point_base.point.y
            )

            base_Z = (
                point_base.point.z
            )

            detection['base_X'] = base_X
            detection['base_Y'] = base_Y
            detection['base_Z'] = base_Z

        except Exception as e:

            self.get_logger().warn(
                f'TF transformacija nije uspela '
                f'za Cube #{cube_id}: {e}'
            )

            return

        # =====================================================
        # DRAW BOUNDING BOX
        # =====================================================

        cv2.rectangle(
            frame,
            (x, y),
            (x + w, y + h),
            (255, 255, 255),
            2
        )

        # =====================================================
        # DRAW DEPTH REGION
        # =====================================================

        cv2.rectangle(
            frame,
            (inner_x1, inner_y1),
            (inner_x2, inner_y2),
            (0, 255, 255),
            1
        )

        # =====================================================
        # DRAW CENTER
        # =====================================================

        cv2.circle(
            frame,
            (cx, cy),
            5,
            (255, 255, 255),
            -1
        )

        # =====================================================
        # COLOR + ID
        # =====================================================

        cv2.putText(
            frame,
            f'#{cube_id} {color}',
            (
                x,
                max(
                    y - 10,
                    20
                )
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )

        # =====================================================
        # CAMERA XYZ
        # =====================================================

        cv2.putText(
            frame,
            f'Cam: '
            f'{X:.3f}, '
            f'{Y:.3f}, '
            f'{Z:.3f}',
            (
                x,
                y + h + 20
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (255, 255, 255),
            1
        )

        # =====================================================
        # LINK_BASE XYZ
        # =====================================================

        cv2.putText(
            frame,
            f'Base: '
            f'{base_X:.3f}, '
            f'{base_Y:.3f}, '
            f'{base_Z:.3f}',
            (
                x,
                y + h + 40
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (255, 255, 255),
            1
        )

        # =====================================================
        # DEPTH
        # =====================================================

        cv2.putText(
            frame,
            f'Depth: {Z:.3f}m',
            (
                x,
                y + h + 60
            ),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (255, 255, 255),
            1
        )

        # =====================================================
        # TERMINAL OUTPUT
        # =====================================================

        self.get_logger().info(
            '----------------------------------------'
        )

        self.get_logger().info(
            f'Cube #{cube_id} | {color}'
        )

        self.get_logger().info(
            f'RGB center = '
            f'({cx}, {cy})'
        )

        self.get_logger().info(
            f'Bounding box = '
            f'x={x}, '
            f'y={y}, '
            f'w={w}, '
            f'h={h}'
        )

        self.get_logger().info(
            f'Depth region = '
            f'({inner_x1}:{inner_x2}, '
            f'{inner_y1}:{inner_y2})'
        )

        self.get_logger().info(
            f'Depth = '
            f'{Z:.4f} m'
        )

        self.get_logger().info(
            f'Depth range = '
            f'{depth_min:.4f} - '
            f'{depth_max:.4f} m'
        )

        self.get_logger().info(
            f'Depth std = '
            f'{depth_std:.4f} m'
        )

        self.get_logger().info(
            f'Camera XYZ = '
            f'({X:.4f}, '
            f'{Y:.4f}, '
            f'{Z:.4f})'
        )

        self.get_logger().info(
            f'link_base XYZ = '
            f'({base_X:.4f}, '
            f'{base_Y:.4f}, '
            f'{base_Z:.4f})'
        )

        self.get_logger().info(
            '----------------------------------------'
        )

    # =========================================================
    # DESTROY
    # =========================================================

    def destroy_node(self):

        cv2.destroyAllWindows()

        super().destroy_node()


# =============================================================
# MAIN
# =============================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = CubeDetector()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()