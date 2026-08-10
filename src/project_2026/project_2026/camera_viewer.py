import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np


class CameraViewer(Node):

    def __init__(self):
        super().__init__('camera_viewer')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info(
            'Camera Viewer + detekcija boja pokrenut.'
        )

    def image_callback(self, msg):

        try:
            # ROS Image -> OpenCV
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='passthrough'
            )

            # Pretvaranje u BGR format za OpenCV
            if msg.encoding == 'rgb8':
                frame = cv2.cvtColor(
                    frame,
                    cv2.COLOR_RGB2BGR
                )

            elif msg.encoding == 'rgba8':
                frame = cv2.cvtColor(
                    frame,
                    cv2.COLOR_RGBA2BGR
                )

            elif msg.encoding == 'bgra8':
                frame = cv2.cvtColor(
                    frame,
                    cv2.COLOR_BGRA2BGR
                )

            elif len(frame.shape) == 2:
                frame = cv2.cvtColor(
                    frame,
                    cv2.COLOR_GRAY2BGR
                )

            # -----------------------------------------
            # BGR -> HSV
            # -----------------------------------------

            hsv = cv2.cvtColor(
                frame,
                cv2.COLOR_BGR2HSV
            )

            # -----------------------------------------
            # DEFINICIJA BOJA
            # -----------------------------------------

            # CRVENA
            # Crvena je na oba kraja HSV hue opsega.
            red_lower_1 = np.array([0, 100, 80])
            red_upper_1 = np.array([10, 255, 255])

            red_lower_2 = np.array([170, 100, 80])
            red_upper_2 = np.array([179, 255, 255])

            mask_red_1 = cv2.inRange(
                hsv,
                red_lower_1,
                red_upper_1
            )

            mask_red_2 = cv2.inRange(
                hsv,
                red_lower_2,
                red_upper_2
            )

            mask_red = cv2.bitwise_or(
                mask_red_1,
                mask_red_2
            )

            # ZELENA
            green_lower = np.array([35, 80, 70])
            green_upper = np.array([85, 255, 255])

            mask_green = cv2.inRange(
                hsv,
                green_lower,
                green_upper
            )

            # PLAVA
            blue_lower = np.array([90, 80, 50])
            blue_upper = np.array([140, 255, 255])

            mask_blue = cv2.inRange(
                hsv,
                blue_lower,
                blue_upper
            )

            # -----------------------------------------
            # DETEKCIJA
            # -----------------------------------------

            self.detect_color(
                frame,
                mask_red,
                "RED"
            )

            self.detect_color(
                frame,
                mask_green,
                "GREEN"
            )

            self.detect_color(
                frame,
                mask_blue,
                "BLUE"
            )

            # -----------------------------------------
            # PRIKAZ
            # -----------------------------------------

            cv2.imshow(
                'D435i Camera POV - Cube Detection',
                frame
            )

            cv2.waitKey(1)

        except Exception as e:

            self.get_logger().error(
                f'Greska pri obradi slike: {e}'
            )

    def detect_color(self, frame, mask, color_name):

        # Uklanjanje sitnog suma
        kernel = np.ones((5, 5), np.uint8)

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

        # Pronadji konture
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        for contour in contours:

            area = cv2.contourArea(contour)

            # Ignorisemo male objekte
            if area < 300:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            # Centar objekta
            M = cv2.moments(contour)

            if M["m00"] != 0:

                cx = int(
                    M["m10"] / M["m00"]
                )

                cy = int(
                    M["m01"] / M["m00"]
                )

            else:
                cx = x + w // 2
                cy = y + h // 2

            # Bounding box
            cv2.rectangle(
                frame,
                (x, y),
                (x + w, y + h),
                (255, 255, 255),
                2
            )

            # Centar
            cv2.circle(
                frame,
                (cx, cy),
                5,
                (255, 255, 255),
                -1
            )

            # Tekst
            label = f"{color_name}"

            cv2.putText(
                frame,
                label,
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2
            )

            # Centar ispod objekta
            center_text = f"({cx}, {cy})"

            cv2.putText(
                frame,
                center_text,
                (x, y + h + 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1
            )

    def destroy_node(self):

        cv2.destroyAllWindows()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = CameraViewer()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()