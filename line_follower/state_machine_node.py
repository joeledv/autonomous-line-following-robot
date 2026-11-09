import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, FloatingPointRange
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition


class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')
        self.declare_parameter('stop_time', 3.0, ParameterDescriptor(floating_point_range=[FloatingPointRange(from_value=1.0, to_value=5.0)]))

        self.stop_time = self.get_parameter('stop_time').value

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.subscription_cmd_vel = self.create_subscription(Twist, '/cmd_vel_pid', self.cmd_vel_callback, 10)
        self.subscription_detection = self.create_subscription(String, '/detections', self.detection_callback, 10)
        self.subscription_start_stop = self.create_subscription(String, '/start_stop', self.start_stop_callback, 10)
        self.subscription_traffic_light = self.create_subscription(String, '/traffic_light', self.traffic_light_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_publisher = self.create_publisher(String, '/robot_state', 10)
        self.traffic_light_client = self.create_client(ChangeState, '/traffic_light_node/change_state')
        # Sent every 0.5 s, not only on changes, so a GUI opened late still gets the state.
        self.timer = self.create_timer(0.5, self.state_callback)

        self.mode = 'idle'  
        self.following = True
        self.state_time = 0.0
        self.cmd_received = False

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'stop_time':
                self.stop_time = param.value
                self.get_logger().info(f'stop_time set to {self.stop_time}')
        return SetParametersResult(successful=True)

    
    def start_stop_callback(self, msg):
        if msg.data == 'start' and self.mode == 'idle':
            self.mode = 'waiting_green'
            self.set_traffic_light(Transition.TRANSITION_ACTIVATE)
            self.get_logger().info('START: waiting for green')
        elif msg.data == 'stop' and self.mode != 'idle':
            if self.mode == 'waiting_green':
                self.set_traffic_light(Transition.TRANSITION_DEACTIVATE)
            self.mode = 'idle'
            self.get_logger().info('STOP')

    def traffic_light_callback(self, msg):
        if self.mode == 'waiting_green' and msg.data == 'green':
            self.mode = 'running'
            self.set_traffic_light(Transition.TRANSITION_DEACTIVATE)
            self.get_logger().info('Green: following the line')

    def set_traffic_light(self, transition_id):
        # call_async without waiting for the answer: waiting inside a callback freezes the node.
        if not self.traffic_light_client.service_is_ready():
            self.get_logger().warn('traffic_light_node is not running')
            return
        request = ChangeState.Request()
        request.transition.id = transition_id
        self.traffic_light_client.call_async(request)

    def detection_callback(self, msg):
        if self.following and msg.data in ('stop', 'alert') and time.monotonic() - self.state_time > self.stop_time + 2.0:
            self.following = False
            self.state_time = time.monotonic()

    def decide(self, msg):
        if self.mode != 'running':
            return Twist()
        if not self.following and time.monotonic() - self.state_time > self.stop_time:
            self.following = True
        if self.following:
            return msg
        return Twist()

    def cmd_vel_callback(self, msg):
        self.cmd_received = True
        self.publisher.publish(self.decide(msg))

    def state_callback(self):
        if self.mode == 'idle':
            text = 'Waiting for START'
        elif self.mode == 'waiting_green':
            text = 'Waiting for green'
        elif not self.following:
            text = 'Stopped at sign'
        # The PID only publishes while the line is in view, so no commands means the line is lost.
        elif not self.cmd_received:
            text = 'Line lost'
        else:
            text = 'Following the line'
        self.cmd_received = False
        self.state_publisher.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
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