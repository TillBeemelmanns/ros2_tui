#!/usr/bin/env python3
"""
Image publisher for testing the image viewer.
Publishes an animated camera image as raw rgb8 and as JPEG, and a depth image
with missing readings, so every decode path shows something recognizable.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

try:
    import cv2
except ImportError:  # JPEG output is skipped without OpenCV
    cv2 = None


class ImagePublisher(Node):
    """Publishes animated test images at camera-like rates"""

    def __init__(self):
        super().__init__('image_publisher')
        self.width, self.height = 640, 480
        self.frame = 0

        self.raw_pub = self.create_publisher(Image, '/test/camera/image_raw', 10)
        # Best effort like most camera drivers, to cover QoS matching.
        self.jpeg_pub = self.create_publisher(
            CompressedImage, '/test/camera/image_raw/compressed', qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, '/test/camera/depth', 10)

        ys, xs = np.mgrid[0:self.height, 0:self.width]
        self.xs, self.ys = xs.astype(np.float32), ys.astype(np.float32)
        dys, dxs = np.mgrid[0:240, 0:320]
        self.dxs, self.dys = dxs.astype(np.float32), dys.astype(np.float32)

        self.create_timer(1.0 / 15, self.publish_camera)
        self.create_timer(1.0 / 10, self.publish_depth)
        self.get_logger().info('Image publisher started')

    def render_scene(self):
        """Sky gradient, scrolling ground grid, color bars and a bouncing ball"""
        t = self.frame / 15.0
        w, h = self.width, self.height
        img = np.zeros((h, w, 3), dtype=np.uint8)

        horizon = h // 2
        sky = np.linspace(0, 1, horizon, dtype=np.float32)[:, None]
        img[:horizon, :, 0] = (40 + 60 * sky).astype(np.uint8)
        img[:horizon, :, 1] = (90 + 80 * sky).astype(np.uint8)
        img[:horizon, :, 2] = (200 - 40 * sky).astype(np.uint8)

        gy, gx = self.ys[horizon:], self.xs[horizon:]
        depth = (gy - horizon + 1)
        lines = ((np.floor(4000 / depth + t * 20) % 2 == 0)
                 ^ (np.floor((gx - w / 2) / depth * 8) % 2 == 0))
        img[horizon:, :, :] = np.where(lines[..., None], [70, 140, 70], [40, 90, 40])

        bars = [(255, 255, 255), (255, 255, 0), (0, 255, 255), (0, 255, 0),
                (255, 0, 255), (255, 0, 0), (0, 0, 255)]
        bar_w = w // len(bars)
        for i, color in enumerate(bars):
            img[10:40, i * bar_w:(i + 1) * bar_w] = color

        cx = w / 2 + (w / 3) * math.sin(t * 1.3)
        cy = horizon - 20 - abs(math.sin(t * 2.6)) * (horizon - 90)
        ball = (self.xs - cx) ** 2 + (self.ys - cy) ** 2 < 30 ** 2
        img[ball] = (230, 60, 40)
        return img

    def publish_camera(self):
        self.frame += 1
        img = self.render_scene()
        stamp = self.get_clock().now().to_msg()

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera_optical_frame'
        msg.height, msg.width = img.shape[:2]
        msg.encoding = 'rgb8'
        msg.step = msg.width * 3
        msg.data = img.tobytes()
        self.raw_pub.publish(msg)

        if cv2 is not None:
            ok, jpeg = cv2.imencode('.jpg', img[:, :, ::-1], [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                compressed = CompressedImage()
                compressed.header = msg.header
                compressed.format = 'rgb8; jpeg compressed bgr8'
                compressed.data = jpeg.tobytes()
                self.jpeg_pub.publish(compressed)

    def publish_depth(self):
        t = self.frame / 15.0
        depth = 2.0 + 1.5 * np.sin(self.dxs * 0.03 + t) * np.cos(self.dys * 0.04 - t * 0.7)
        depth += self.dys * 0.01
        # Missing readings, like the shadow of a real depth sensor.
        depth[(self.dxs > 140) & (self.dxs < 180) & (self.dys > 60)] = np.nan

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'depth_optical_frame'
        msg.height, msg.width = depth.shape
        msg.encoding = '32FC1'
        msg.step = msg.width * 4
        msg.data = depth.astype('<f4').tobytes()
        self.depth_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
