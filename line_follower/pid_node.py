import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, FloatingPointRange
from simple_pid import PID
from rclpy.executors import ExternalShutdownException


class PidNode(Node):

    def __init__(self):
        super().__init__('pid_node')

        self.declare_parameter('kp', 4.8, ParameterDescriptor(
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=20.0)]))
        self.declare_parameter('ki', 0.0, ParameterDescriptor(
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=5.0)]))
        self.declare_parameter('kd', 1.5, ParameterDescriptor(
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=10.0)]))
        self.declare_parameter('max_angular', 2.1, ParameterDescriptor( # x25 = PWM correction; serial_node clips it at (max_speed - drive_min) / 2
            floating_point_range=[FloatingPointRange(from_value=0.1, to_value=5.0)]))
        self.declare_parameter('linear_speed', 0.2, ParameterDescriptor( # any value > 0 means "drive"; 0.0 stops the robot
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0)]))
        self.declare_parameter('max_integral', 0.4, ParameterDescriptor( # x25 = 10 PWM
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=5.0)]))
        self.declare_parameter('integral_zone', 0.10, ParameterDescriptor( # integrate only while |error| is below this
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0)]))
        self.declare_parameter('tolerance', 0.02, ParameterDescriptor( # camera noise while driving is ~0.01
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=0.5)]))
        self.declare_parameter('restart_gap', 0.1, ParameterDescriptor( # keep equal to serial_node's cmd_timeout: a gap this long means the wheels already stopped
            floating_point_range=[FloatingPointRange(from_value=0.1, to_value=5.0)]))
        self.declare_parameter('min_dt', 1.0 / 60.0, ParameterDescriptor( # half a frame at 30 FPS; errors closer than this came in a burst
            floating_point_range=[FloatingPointRange(from_value=0.001, to_value=0.1)]))

        kp = self.get_parameter('kp').value
        kd = self.get_parameter('kd').value
        self.ki = self.get_parameter('ki').value
        self.max_angular = self.get_parameter('max_angular').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.max_integral = self.get_parameter('max_integral').value
        self.integral_zone = self.get_parameter('integral_zone').value
        self.tolerance = self.get_parameter('tolerance').value
        self.restart_gap = self.get_parameter('restart_gap').value
        self.min_dt = self.get_parameter('min_dt').value

        self.pid = PID(Kp=kp, Ki=0.0, Kd=kd, setpoint=0.0, sample_time=None)
        self.integral = 0.0

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.subscription = self.create_subscription(Float32, '/error', self.error_callback, 1) # depth = 1

        self.publisher = self.create_publisher(Twist, '/cmd_vel_pid', 10)

        self.last_error_received_time = None

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'kp':
                self.pid.Kp = param.value   # the PID was built with the old gain, so it has to change there
                self.get_logger().info(f'kp set to {self.pid.Kp}')

            elif param.name == 'ki':
                self.ki = param.value
                if self.ki == 0.0:
                    self.integral = 0.0   # turning the integral off must also drop what it already accumulated
                self.get_logger().info(f'ki set to {self.ki}')

            elif param.name == 'kd':
                self.pid.Kd = param.value
                self.get_logger().info(f'kd set to {self.pid.Kd}')

            elif param.name == 'max_angular':
                self.max_angular = param.value
                self.get_logger().info(f'max_angular set to {self.max_angular}')

            elif param.name == 'linear_speed':
                self.linear_speed = param.value
                self.get_logger().info(f'linear_speed set to {self.linear_speed}')

            elif param.name == 'max_integral':
                self.max_integral = param.value
                # error_callback only re-clamps inside integral_zone, so a lower cap is applied now.
                self.integral = max(-self.max_integral, min(self.max_integral, self.integral))
                self.get_logger().info(f'max_integral set to {self.max_integral}')

            elif param.name == 'integral_zone':
                self.integral_zone = param.value
                self.get_logger().info(f'integral_zone set to {self.integral_zone}')

            elif param.name == 'tolerance':
                self.tolerance = param.value
                self.get_logger().info(f'tolerance set to {self.tolerance}')

            elif param.name == 'restart_gap':
                self.restart_gap = param.value
                self.get_logger().info(f'restart_gap set to {self.restart_gap}')

            elif param.name == 'min_dt':
                self.min_dt = param.value
                self.get_logger().info(f'min_dt set to {self.min_dt}')

        return SetParametersResult(successful=True)

    def error_callback(self, msg):
        error = msg.data
        now = time.monotonic() # Chronometer 
    
        # Reset the PID if this is the first error or the gap since the last one is too long
        if self.last_error_received_time is None or now - self.last_error_received_time > self.restart_gap:
            self.pid.reset()
            dt = self.min_dt
        else:
            dt = now - self.last_error_received_time
        dt = max(dt, self.min_dt)
        self.last_error_received_time = now

        angular = self.pid(error, dt=dt)
        if abs(error) < self.tolerance:
            angular = 0.0   

        if abs(error) < self.integral_zone:
            self.integral += self.ki * -error * dt
            self.integral = max(-self.max_integral, min(self.max_integral, self.integral))
        angular = max(-self.max_angular, min(self.max_angular, angular + self.integral))

        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = float(angular)
        self.publisher.publish(cmd)
        self.get_logger().info(
            f'error={error:+.3f} -> v={cmd.linear.x:.2f} w={cmd.angular.z:+.2f} '
            f'(i={self.integral:+.2f})', throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = PidNode()
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
