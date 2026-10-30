#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from cube_interfaces.srv import DetectSurface

import cv2
import numpy as np
import threading


class SurfaceDetector(Node):

    def __init__(self):
        super().__init__('surface_detector')

        self.bridge = CvBridge()

        # ============================================================
        # POSLEDNJI FRAME KAMERE
        # ============================================================

        self.latest_frame = None
        self.frame_lock = threading.Lock()

        # ============================================================
        # ROS SUBSCRIBER
        # ============================================================

        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        # ============================================================
        # SERVICE
        # ============================================================

        self.service = self.create_service(
            DetectSurface,
            '/detect_surface',
            self.detect_surface_callback
        )

        # ============================================================
        # PARAMETRI DETEKCIJE
        # ============================================================

        self.required_hits = 1

        # Minimalni procenat boje u ROI-u
        self.min_color_ratio = 0.10

        # ROI:
        # centralni deo slike
        self.roi_x_min = 0.25
        self.roi_x_max = 0.75
        self.roi_y_min = 0.25
        self.roi_y_max = 0.85

        # ============================================================
        # GUI
        # ============================================================

        self.window_name = 'Surface Detector V9'

        self.get_logger().info('========================================')
        self.get_logger().info('SURFACE DETECTOR V9')
        self.get_logger().info('DETEKCIJA BOJE NA ZAHTEV')
        self.get_logger().info('========================================')
        self.get_logger().info('Ne koristi depth.')
        self.get_logger().info('Ne koristi TF.')
        self.get_logger().info('Ne koristi 3D poziciju.')
        self.get_logger().info('Ne trazi geometriju povrsine.')
        self.get_logger().info('Ne ceka frame unutar service-a.')
        self.get_logger().info('Service uzima POSLEDNJI dostupni frame.')
        self.get_logger().info('========================================')

    # ================================================================
    # IMAGE CALLBACK
    # ================================================================

    def image_callback(self, msg):

        try:

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            # Sacuvaj poslednji frame
            with self.frame_lock:
                self.latest_frame = frame.copy()

            # Prikaz
            display = frame.copy()

            height, width = display.shape[:2]

            x1 = int(width * self.roi_x_min)
            x2 = int(width * self.roi_x_max)

            y1 = int(height * self.roi_y_min)
            y2 = int(height * self.roi_y_max)

            # ROI okvir
            cv2.rectangle(
                display,
                (x1, y1),
                (x2, y2),
                (255, 255, 255),
                2
            )

            cv2.putText(
                display,
                'SURFACE COLOR ROI',
                (x1 + 5, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2
            )

            cv2.putText(
                display,
                'WAITING FOR /detect_surface',
                (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2
            )

            cv2.imshow(self.window_name, display)

            # Vazno:
            # ne blokira ROS node
            cv2.waitKey(1)

        except Exception as e:

            self.get_logger().error(
                f'Greska u image_callback: {e}'
            )

    # ================================================================
    # DETEKCIJA BOJE
    # ================================================================

    def detect_color(self, frame):

        if frame is None:
            return None, 0.0

        height, width = frame.shape[:2]

        x1 = int(width * self.roi_x_min)
        x2 = int(width * self.roi_x_max)

        y1 = int(height * self.roi_y_min)
        y2 = int(height * self.roi_y_max)

        roi = frame[y1:y2, x1:x2]

        if roi.size == 0:
            return None, 0.0

        # ============================================================
        # BLUR
        # ============================================================

        roi_blur = cv2.GaussianBlur(
            roi,
            (5, 5),
            0
        )

        # ============================================================
        # HSV
        # ============================================================

        hsv = cv2.cvtColor(
            roi_blur,
            cv2.COLOR_BGR2HSV
        )

        # ============================================================
        # MASKS
        # ============================================================

        # RED
        red_low_1 = np.array([0, 100, 80])
        red_high_1 = np.array([10, 255, 255])

        red_low_2 = np.array([170, 100, 80])
        red_high_2 = np.array([180, 255, 255])

        red_mask_1 = cv2.inRange(
            hsv,
            red_low_1,
            red_high_1
        )

        red_mask_2 = cv2.inRange(
            hsv,
            red_low_2,
            red_high_2
        )

        red_mask = cv2.bitwise_or(
            red_mask_1,
            red_mask_2
        )

        # GREEN
        green_low = np.array([35, 70, 70])
        green_high = np.array([85, 255, 255])

        green_mask = cv2.inRange(
            hsv,
            green_low,
            green_high
        )

        # BLUE
        blue_low = np.array([90, 70, 50])
        blue_high = np.array([140, 255, 255])

        blue_mask = cv2.inRange(
            hsv,
            blue_low,
            blue_high
        )

        # ============================================================
        # MORPHOLOGY
        # ============================================================

        kernel = np.ones(
            (5, 5),
            np.uint8
        )

        red_mask = cv2.morphologyEx(
            red_mask,
            cv2.MORPH_OPEN,
            kernel
        )

        red_mask = cv2.morphologyEx(
            red_mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        green_mask = cv2.morphologyEx(
            green_mask,
            cv2.MORPH_OPEN,
            kernel
        )

        green_mask = cv2.morphologyEx(
            green_mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        blue_mask = cv2.morphologyEx(
            blue_mask,
            cv2.MORPH_OPEN,
            kernel
        )

        blue_mask = cv2.morphologyEx(
            blue_mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        # ============================================================
        # COLOR RATIOS
        # ============================================================

        total_pixels = roi.shape[0] * roi.shape[1]

        red_ratio = (
            cv2.countNonZero(red_mask)
            / total_pixels
        )

        green_ratio = (
            cv2.countNonZero(green_mask)
            / total_pixels
        )

        blue_ratio = (
            cv2.countNonZero(blue_mask)
            / total_pixels
        )

        ratios = {
            'RED': red_ratio,
            'GREEN': green_ratio,
            'BLUE': blue_ratio
        }

        # Najveca boja
        best_color = max(
            ratios,
            key=ratios.get
        )

        best_ratio = ratios[best_color]

        # ============================================================
        # PROVERA
        # ============================================================

        if best_ratio < self.min_color_ratio:

            return None, best_ratio

        return best_color, best_ratio

    # ================================================================
    # SERVICE CALLBACK
    # ================================================================

    def detect_surface_callback(
        self,
        request,
        response
    ):

        self.get_logger().info(
            '----------------------------------------'
        )

        self.get_logger().info(
            'NOVI ZAHTEV ZA DETEKCIJU POVRSINE'
        )

        self.get_logger().info(
            'Robot je stigao iznad povrsine.'
        )

        self.get_logger().info(
            'Uzimam poslednji dostupan frame.'
        )

        # ============================================================
        # UZMI POSLEDNJI FRAME
        # ============================================================

        with self.frame_lock:

            if self.latest_frame is None:

                response.success = False
                response.color = ''
                response.confidence = 0.0

                self.get_logger().warn(
                    'NEMA FRAME-A SA KAMERE!'
                )

                return response

            frame = self.latest_frame.copy()

        # ============================================================
        # DETEKCIJA
        # ============================================================

        color, confidence = self.detect_color(
            frame
        )

        # ============================================================
        # REZULTAT
        # ============================================================

        if color is None:

            response.success = False
            response.color = ''
            response.confidence = float(confidence)

            self.get_logger().warn(
                'POVRSINA NIJE DETEKTOVANA'
            )

            self.get_logger().warn(
                f'Najbolji color ratio: {confidence:.3f}'
            )

        else:

            response.success = True
            response.color = color
            response.confidence = float(confidence)

            self.get_logger().info(
                '========================================'
            )

            self.get_logger().info(
                f'SURFACE POTVRDJENA: {color}'
            )

            self.get_logger().info(
                f'Confidence: {confidence:.3f}'
            )

            self.get_logger().info(
                '========================================'
            )

        return response


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    node = SurfaceDetector()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        cv2.destroyAllWindows()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()