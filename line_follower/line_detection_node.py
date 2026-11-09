import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, FloatingPointRange, IntegerRange, ParameterType
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from line_follower.utils.line_detection import centroid, error_calculation, draw_debug
from line_follower.utils.stream import MJPEGStream


class LineDetectionNode(Node):

    def __init__(self):
        super().__init__('line_detection_node')

        self.declare_parameter('roi_start', 0.50, ParameterDescriptor(
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=0.75)]))
        self.declare_parameter('width', 320, ParameterDescriptor( 
            integer_range=[IntegerRange(from_value=80, to_value=640, step=1)]))
        self.declare_parameter('kernel_size', 7, ParameterDescriptor( 
            integer_range=[IntegerRange(from_value=5, to_value=9, step=2)]))
        self.declare_parameter('frac_min', 0.01, ParameterDescriptor( 
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0)]))
        self.declare_parameter('frac_max', 0.45, ParameterDescriptor( 
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0)]))
        self.declare_parameter('width_max', 0.50, ParameterDescriptor( 
            floating_point_range=[FloatingPointRange(from_value=0.01, to_value=1.0)]))
        self.declare_parameter('error_smoothing_alpha', 0.6, ParameterDescriptor( 
            floating_point_range=[FloatingPointRange(from_value=0.01, to_value=1.0)]))
        self.declare_parameter('ema_reset_after', 0.1, ParameterDescriptor( # keep equal to serial_node's cmd_timeout: by then the wheels stopped and the old error is stale
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=10.0)]))
        self.declare_parameter('video_debug', False, ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, read_only=True))
        self.declare_parameter('side_timeout', 5.0, ParameterDescriptor(
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=5.0)]))
        self.declare_parameter('recovery_time', 0.3, ParameterDescriptor(
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0)]))
        self.declare_parameter('end_after_stop', 3.0, ParameterDescriptor(
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=10.0)]))

        self.roi_start = self.get_parameter('roi_start').value
        self.width = self.get_parameter('width').value
        self.kernel_size = self.get_parameter('kernel_size').value
        self.frac_min = self.get_parameter('frac_min').value
        self.frac_max = self.get_parameter('frac_max').value
        self.width_max = self.get_parameter('width_max').value
        self.error_smoothing_alpha = self.get_parameter('error_smoothing_alpha').value
        self.ema_reset_after = self.get_parameter('ema_reset_after').value
        self.video_debug = self.get_parameter('video_debug').value
        self.side_timeout = self.get_parameter('side_timeout').value
        self.recovery_time = self.get_parameter('recovery_time').value
        self.end_after_stop = self.get_parameter('end_after_stop').value

        # A range only sees one value, so this rule between two is checked by hand.
        if self.frac_min >= self.frac_max:
            raise ValueError(f'frac_min ({self.frac_min}) must be lower than frac_max ({self.frac_max})')

        self.center = self.width / 2
        self.kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.bridge = CvBridge()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, qos)
        self.detection_subscription = self.create_subscription(String, '/detections', self.detection_callback, 10)

        self.side = None
        self.side_time = 0.0
        self.stop_seen = 0.0

        self.error_publisher = self.create_publisher(Float32, '/error', 10)
        self.filtered_error = None   
        self.last_seen = 0.0
    
        if self.video_debug:
            self.stream = MJPEGStream(port=8080, path='/camera')
            self.get_logger().info(f'Stream ready at {self.stream.url()}')

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'roi_start':
                self.roi_start = param.value
                self.get_logger().info(f'roi_start set to {self.roi_start}')

            elif param.name == 'width':
                self.width = param.value
                self.center = self.width / 2   
                self.get_logger().info(f'width set to {self.width}')

            elif param.name == 'kernel_size':
                self.kernel_size = param.value
                self.kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)   
                self.get_logger().info(f'kernel_size set to {self.kernel_size}')

            elif param.name == 'frac_min':
                if param.value >= self.frac_max:
                    return SetParametersResult(successful=False, reason=f'frac_min must be lower than frac_max ({self.frac_max}), got {param.value}')
                else:
                    self.frac_min = param.value
                    self.get_logger().info(f'frac_min set to {self.frac_min}')

            elif param.name == 'frac_max':
                if param.value <= self.frac_min:
                    return SetParametersResult(successful=False, reason=f'frac_max must be greater than frac_min ({self.frac_min}), got {param.value}')
                else:
                    self.frac_max = param.value
                    self.get_logger().info(f'frac_max set to {self.frac_max}')

            elif param.name == 'width_max':
                self.width_max = param.value
                self.get_logger().info(f'width_max set to {self.width_max}')

            elif param.name == 'error_smoothing_alpha':
                self.error_smoothing_alpha = param.value
                self.get_logger().info(f'error_smoothing_alpha set to {self.error_smoothing_alpha}')

            elif param.name == 'ema_reset_after':
                self.ema_reset_after = param.value
                self.get_logger().info(f'ema_reset_after set to {self.ema_reset_after}')

            elif param.name == 'recovery_time':
                self.recovery_time = param.value
                self.get_logger().info(f'recovery_time set to {self.recovery_time}')

            elif param.name == 'end_after_stop':
                self.end_after_stop = param.value
                self.get_logger().info(f'end_after_stop set to {self.end_after_stop}')

        return SetParametersResult(successful=True)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        now = time.monotonic()

        if now - self.side_time > self.side_timeout:
            self.side = None

        mask, cx = centroid(frame, self.roi_start, self.width, self.kernel,
                            self.frac_min, self.frac_max, self.width_max, self.side)
        error = error_calculation(cx, self.center)

        if error is not None:
            self.last_seen = now
            if self.filtered_error is None:
                self.filtered_error = error   
            else:
                self.filtered_error = (self.error_smoothing_alpha * error
                                        + (1 - self.error_smoothing_alpha) * self.filtered_error)

            self.error_publisher.publish(Float32(data=self.filtered_error))
            self.get_logger().info(f'error: {error:+.3f} (filtered: {self.filtered_error:+.3f})', throttle_duration_sec=1.0)
        else:
            self.get_logger().warn('No line detected', throttle_duration_sec=2.5)
            # A line lost right after a stop sign is the end of the track, so it doesn't search there.
            searching = (now - self.last_seen < self.recovery_time
                         and now - self.stop_seen > self.end_after_stop)
            if searching and self.filtered_error is not None:
                # The same error it had when it lost the line: a sudden change would make the
                # PID's derivative kick.
                self.error_publisher.publish(Float32(data=self.filtered_error))
            elif now - self.last_seen > self.recovery_time + self.ema_reset_after:
                self.filtered_error = None

        if self.video_debug:
            debug_frame = draw_debug(mask, cx, self.filtered_error, self.center)
            self.stream.update(debug_frame)

    def detection_callback(self, msg):
        if msg.data in ['left', 'right']:
            self.side = msg.data
            self.side_time = time.monotonic()
        elif msg.data == 'stop':
            self.stop_seen = time.monotonic()

def main(args=None):
    rclpy.init(args=args)
    node = LineDetectionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.video_debug:
            node.stream.close()
        node.destroy_node()
        if rclpy.ok():   
            rclpy.shutdown()


if __name__ == '__main__':
    main()
