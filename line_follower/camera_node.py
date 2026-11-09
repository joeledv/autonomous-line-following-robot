import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import time
from rclpy.executors import ExternalShutdownException


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('camera_device', 0, ParameterDescriptor(read_only=True))
        self.declare_parameter('video_codec', 'MJPG', ParameterDescriptor(read_only=True))
        self.declare_parameter('frame_width', 640, ParameterDescriptor(read_only=True))
        self.declare_parameter('frame_height', 480, ParameterDescriptor(read_only=True))
        self.declare_parameter('frame_id', 'camera_link', ParameterDescriptor(read_only=True))

        self.camera_device = self.get_parameter('camera_device').value
        self.video_codec = self.get_parameter('video_codec').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.frame_id = self.get_parameter('frame_id').value

        if len(self.video_codec) != 4:
            raise ValueError(f'video_codec ({self.video_codec}) must be 4 characters long')

        self.bridge = CvBridge() # Convert between ROS2 image message and OpenCV image

        self.cap = cv2.VideoCapture(self.camera_device) 

        if not self.cap.isOpened(): 
            self.cap.release()
            self.get_logger().error('Could not open camera')
            raise RuntimeError('Could not open camera')

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.video_codec)) 
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width) 
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height) 

        self.get_logger().info(f'Camera: {int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} @ {self.cap.get(cv2.CAP_PROP_FPS):.0f} FPS')

        # Drop lost frames and keep only the newest. A late frame is useless.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pub = self.create_publisher(Image, '/image_raw', qos) # Publish the image to the /image_raw topic

        self.running = True
        self.thread_ = threading.Thread(target=self.publish_frame, daemon=True)    
        self.thread_.start()

    def publish_frame(self):
        while self.running and rclpy.ok():
            ret, frame = self.cap.read()

            if not ret:
                self.get_logger().warn('Empty frame', throttle_duration_sec=2.5)
                time.sleep(0.01)
                continue

            try: 
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
            except Exception as e:
                self.get_logger().error(f'Error converting frame to image message: {e}', throttle_duration_sec=2.5)
                continue

            try:
                self.pub.publish(msg)
            except Exception:
                if rclpy.ok():
                    raise
                else:
                    break

    def destroy_node(self):
        self.running = False
        self.thread_.join(timeout=1.0) 
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():   
            rclpy.shutdown()


if __name__ == '__main__':
    main()