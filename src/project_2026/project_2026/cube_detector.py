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

        # Frame depth kamere
        self.camera_frame = 'camera_depth_optical_frame'

        # Koordinatni sistem robota
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

        # -----------------------------------------------------
        # Provera da li imamo depth
        # -----------------------------------------------------

        if self.latest_depth is None:
            return

        # -----------------------------------------------------
        # Provera CameraInfo
        # -----------------------------------------------------

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

            # Ako TF nije uspeo za ovu kockicu
            # nemoj je slati u /detected_cubes

            if (
                'base_X' not in detection or
                'base_Y' not in detection or
                'base_Z' not in detection
            ):
                continue

            cube_msg = CubeDetection()

            cube_msg.id = detection['id']

            cube_msg.color = detection['color']

            # =================================================
            # BITNO:
            #
            # OVE KOORDINATE SU U link_base FRAME-U
            # =================================================

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

        # -----------------------------------------------------
        # Morphological filtering
        # -----------------------------------------------------

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

        # -----------------------------------------------------
        # Contours
        # -----------------------------------------------------

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # -----------------------------------------------------
        # Process contours
        # -----------------------------------------------------

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

            # -------------------------------------------------
            # Centroid
            # -------------------------------------------------

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

            return

        # =====================================================
        # DEPTH WINDOW
        # =====================================================

        window_size = 5

        x1 = max(
            0,
            cx - window_size
        )

        x2 = min(
            depth_width,
            cx + window_size + 1
        )

        y1 = max(
            0,
            cy - window_size
        )

        y2 = min(
            depth_height,
            cy + window_size + 1
        )

        depth_window = self.latest_depth[
            y1:y2,
            x1:x2
        ]

        # =====================================================
        # VALID DEPTH
        # =====================================================

        valid_depth = depth_window[
            np.isfinite(depth_window) &
            (depth_window > 0.0)
        ]

        if len(valid_depth) == 0:

            self.get_logger().warn(
                f'Cube #{cube_id} ({color}): '
                'nema validnog depth podatka.'
            )

            return

        # =====================================================
        # MEDIAN DEPTH
        # =====================================================

        Z = float(
            np.median(valid_depth)
        )

        # =====================================================
        # CAMERA INTRINSICS
        # =====================================================

        fx = self.camera_info.k[0]

        fy = self.camera_info.k[4]

        camera_cx = self.camera_info.k[2]

        camera_cy = self.camera_info.k[5]

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

        # Sacuvaj Camera XYZ

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
            self.get_clock().now().to_msg()
        )

        point_camera.point.x = X

        point_camera.point.y = Y

        point_camera.point.z = Z

        # =====================================================
        # TF2
        # =====================================================

        try:

            # -------------------------------------------------
            # Provera da li transformacija postoji
            # -------------------------------------------------

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

            # -------------------------------------------------
            # Uzmi transformaciju
            # -------------------------------------------------

            transform = self.tf_buffer.lookup_transform(
                self.robot_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(
                    seconds=0.1
                )
            )

            # -------------------------------------------------
            # Transformacija PointStamped
            # -------------------------------------------------

            point_base = do_transform_point(
                point_camera,
                transform
            )

            # -------------------------------------------------
            # BASE XYZ
            # -------------------------------------------------

            base_X = point_base.point.x

            base_Y = point_base.point.y

            base_Z = point_base.point.z

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
        # DRAW RECTANGLE
        # =====================================================

        cv2.rectangle(
            frame,
            (x, y),
            (x + w, y + h),
            (255, 255, 255),
            2
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
            (x, max(y - 10, 20)),
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

            (x, y + h + 20),

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

            (x, y + h + 40),

            cv2.FONT_HERSHEY_SIMPLEX,

            0.42,

            (255, 255, 255),

            1
        )

        # =====================================================
        # TERMINAL
        # =====================================================

        self.get_logger().info(
            f'Cube #{cube_id}: {color} | '

            f'Camera XYZ=('
            f'{X:.3f}, '
            f'{Y:.3f}, '
            f'{Z:.3f}) | '

            f'link_base XYZ=('
            f'{base_X:.3f}, '
            f'{base_Y:.3f}, '
            f'{base_Z:.3f})'
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