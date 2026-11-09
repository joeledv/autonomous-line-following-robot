import rclpy
from rclpy.node import Node
import supervision as sv
from ultralytics import YOLO
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, FloatingPointRange, IntegerRange, ParameterType
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
import os
from ament_index_python.packages import get_package_share_directory
from line_follower.utils.stream import MJPEGStream
import time


class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')

        self.model = YOLO(os.path.join(get_package_share_directory('line_follower'), 'models', 'best_ncnn_model'), task='detect')
        self.declare_parameter('confidence_threshold', 0.5, ParameterDescriptor(
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=0.95)]))
        self.declare_parameter('nms_threshold', 0.5, ParameterDescriptor(
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=0.95)]))
        self.declare_parameter('class_agnostic', False, ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self.declare_parameter('video_debug', False, ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, read_only=True))
        self.declare_parameter('detection_period', 0.25, ParameterDescriptor(floating_point_range=[FloatingPointRange(from_value=0.0, to_value=2.0)]))
        self.declare_parameter('min_area', 5000, ParameterDescriptor(integer_range=[IntegerRange(from_value=0, to_value=307200, step=1)]))
        
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.nms_threshold = self.get_parameter('nms_threshold').value
        self.class_agnostic = self.get_parameter('class_agnostic').value
        self.video_debug = self.get_parameter('video_debug').value
        self.detection_period = self.get_parameter('detection_period').value
        self.min_area = self.get_parameter('min_area').value

        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.bridge = CvBridge()
        self.last_detection = 0.0
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(Image, '/image_raw', self.detection_callback, qos)

        self.detection_publisher = self.create_publisher(String, '/detections', 10)

        if self.video_debug:
            self.box_annotator = sv.BoxAnnotator()
            self.label_annotator = sv.LabelAnnotator()
            self.stream = MJPEGStream(port=8081, path='/detections', fps=15)
            self.get_logger().info(f'Stream ready at {self.stream.url()}')

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'confidence_threshold':
                self.confidence_threshold = param.value
                self.get_logger().info(f'confidence_threshold set to {self.confidence_threshold}')
            elif param.name == 'nms_threshold':
                self.nms_threshold = param.value
                self.get_logger().info(f'nms_threshold set to {self.nms_threshold}')
            elif param.name == 'class_agnostic':
                self.class_agnostic = param.value
                self.get_logger().info(f'class_agnostic set to {self.class_agnostic}')
            elif param.name == 'detection_period':
                self.detection_period = param.value
                self.get_logger().info(f'detection_period set to {self.detection_period}')
            elif param.name == 'min_area':
                self.min_area = param.value
                self.get_logger().info(f'min_area set to {self.min_area}')
        return SetParametersResult(successful=True)


    def detection_callback(self, msg):
        if time.monotonic() - self.last_detection < self.detection_period:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model(frame, imgsz=320, conf=self.confidence_threshold, iou=self.nms_threshold, agnostic_nms=self.class_agnostic, verbose=False)[0]

        detections = sv.Detections.from_ultralytics(results)
        detections = detections[detections.area > self.min_area]

        if len(detections) > 0:
            best = detections.confidence.argmax()
            sign = detections.data['class_name'][best]
            self.detection_publisher.publish(String(data=str(sign)))

        self.last_detection = time.monotonic()

        if self.video_debug:
            labels = [
                f"{class_name} {confidence:.2f}"
                for class_name, confidence
                in zip(detections['class_name'], detections.confidence)
            ]

            annotated_image = self.box_annotator.annotate(
                scene=frame, detections=detections)
            annotated_image = self.label_annotator.annotate(
                scene=annotated_image, detections=detections, labels=labels)
            self.stream.update(annotated_image)

def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
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