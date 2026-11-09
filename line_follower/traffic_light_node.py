import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from line_follower.utils.traffic_light import traffic_light


class TrafficLightNode(LifecycleNode):
    def __init__(self):
        super().__init__('traffic_light_node')
        self.bridge = CvBridge()
        self.publisher = None
        self.subscription = None

    def on_configure(self, state):
        self.publisher = self.create_lifecycle_publisher(String, '/traffic_light', 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        # Lifecycle only switches publishers on and off; the subscription is created here and
        # destroyed on deactivate, so an inactive node doesn't even receive the images.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, qos)
        self.get_logger().info('Looking for the traffic light')
        return super().on_activate(state)

    def on_deactivate(self, state):
        self.destroy_subscription(self.subscription)
        self.subscription = None
        self.get_logger().info('Traffic light off')
        return super().on_deactivate(state)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        color = traffic_light(frame)
        if color is not None:
            self.publisher.publish(String(data=color))


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightNode()
    node.trigger_configure()   # ready but inactive until the state machine activates it
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