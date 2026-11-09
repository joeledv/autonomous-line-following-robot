import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, FloatingPointRange, IntegerRange


class SerialNode(Node):

    def __init__(self):
        super().__init__('serial_node')

        self.declare_parameter('serial_port', '/dev/ttyUSB0', ParameterDescriptor(read_only=True))
        self.declare_parameter('baud_rate', 9600, ParameterDescriptor(read_only=True)) # must match Serial.begin() in the Arduino sketch
        self.declare_parameter('drive_min', 150, ParameterDescriptor( # minimum speed for the motors; negative brakes the inner side in hard turns
            integer_range=[IntegerRange(from_value=-255, to_value=255, step=1)]))
        self.declare_parameter('max_speed', 255, ParameterDescriptor( # maximum speed for the motors
            integer_range=[IntegerRange(from_value=0, to_value=255, step=1)]))
        self.declare_parameter('base_speed', 180, ParameterDescriptor( # base speed for the motors
            integer_range=[IntegerRange(from_value=0, to_value=255, step=1)]))
        self.declare_parameter('pwm_per_rad', 25.0, ParameterDescriptor( # Hand-calibrated PWM per unit of angular.z
            floating_point_range=[FloatingPointRange(from_value=1.0, to_value=100.0)]))
        self.declare_parameter('cmd_timeout', 0.1, ParameterDescriptor( # /cmd_vel arrives every ~0.04 s; stay under the Arduino's 0.4 s watchdog so this node knows when the wheels stopped
            floating_point_range=[FloatingPointRange(from_value=0.05, to_value=0.3)]))
        self.declare_parameter('start_speed', 115, ParameterDescriptor( # breaking free from rest takes ~100 PWM, rolling only ~70
            integer_range=[IntegerRange(from_value=0, to_value=255, step=1)]))
        self.declare_parameter('start_duration', 0.1, ParameterDescriptor( # how long start_speed is held after a stop; 0.0 turns it off
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=1.0)]))

        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.drive_min = self.get_parameter('drive_min').value
        self.max_speed = self.get_parameter('max_speed').value
        self.base_speed = self.get_parameter('base_speed').value
        self.pwm_per_rad = self.get_parameter('pwm_per_rad').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.start_speed = self.get_parameter('start_speed').value
        self.start_duration = self.get_parameter('start_duration').value

        # A range only sees one value, so the rules between the three speeds are checked by hand.
        if self.drive_min >= self.max_speed:
            raise ValueError(f'drive_min ({self.drive_min}) must be lower than max_speed ({self.max_speed})')
        if not self.drive_min <= self.base_speed <= self.max_speed:
            raise ValueError(f'base_speed ({self.base_speed}) must be between drive_min ({self.drive_min}) and max_speed ({self.max_speed})')

        self.max_correction = (self.max_speed - self.drive_min) / 2 # maximum correction for the motors

        self.add_on_set_parameters_callback(self.parameters_callback)

        try:
            self.serial_conn = serial.Serial(serial_port, baud_rate, write_timeout=0.1)
        except serial.SerialException as e:
            self.get_logger().error(f'Could not open {serial_port}: {e}')
            raise

        time.sleep(2.0)   # Arduino auto-resets when the serial port opens; give it time to boot

        self.last_cmd_time = None
        self.moving_since = None   # None while the wheels are stopped

        # Depth 1: a queued command is already stale; only the newest should drive.
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 1)

        self.timer = self.create_timer(0.02, self.timeout_callback)

        self.get_logger().info(f'Connected to {serial_port} at {baud_rate} baud')

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'drive_min':
                if param.value >= self.max_speed or param.value > self.base_speed:
                    return SetParametersResult(successful=False, reason=f'drive_min must be lower than max_speed ({self.max_speed}) and not above base_speed ({self.base_speed}), got {param.value}')
                else:
                    self.drive_min = param.value
                    self.max_correction = (self.max_speed - self.drive_min) / 2
                    self.get_logger().info(f'drive_min set to {self.drive_min} (max correction {self.max_correction} PWM)')

            elif param.name == 'max_speed':
                if param.value <= self.drive_min or param.value < self.base_speed:
                    return SetParametersResult(successful=False, reason=f'max_speed must be greater than drive_min ({self.drive_min}) and not below base_speed ({self.base_speed}), got {param.value}')
                else:
                    self.max_speed = param.value
                    self.max_correction = (self.max_speed - self.drive_min) / 2
                    self.get_logger().info(f'max_speed set to {self.max_speed} (max correction {self.max_correction} PWM)')

            elif param.name == 'base_speed':
                if not self.drive_min <= param.value <= self.max_speed:
                    return SetParametersResult(successful=False, reason=f'base_speed must be between drive_min ({self.drive_min}) and max_speed ({self.max_speed}), got {param.value}')
                else:
                    self.base_speed = param.value
                    self.get_logger().info(f'base_speed set to {self.base_speed}')

            elif param.name == 'pwm_per_rad':
                self.pwm_per_rad = param.value
                self.get_logger().info(f'pwm_per_rad set to {self.pwm_per_rad}')

            elif param.name == 'cmd_timeout':
                self.cmd_timeout = param.value
                self.get_logger().info(f'cmd_timeout set to {self.cmd_timeout}')

            elif param.name == 'start_speed':
                self.start_speed = param.value
                self.get_logger().info(f'start_speed set to {self.start_speed}')

            elif param.name == 'start_duration':
                self.start_duration = param.value
                self.get_logger().info(f'start_duration set to {self.start_duration}')

        return SetParametersResult(successful=True)

    def cmd_vel_callback(self, msg):
        now = time.monotonic()
        self.last_cmd_time = now

        # A NaN turn would survive the clamp below as a full right turn, so it stops.
        if msg.linear.x > 0.0 and math.isfinite(msg.angular.z):
            if self.moving_since is None:
                self.moving_since = now
                self.get_logger().info(f'Starting: {self.start_speed} PWM for {self.start_duration} s')

            if now - self.moving_since < self.start_duration:
                # base_speed keeps the wheels rolling but can't get them moving from rest.
                left_speed, right_speed = self.start_speed, self.start_speed
            else:
                # REP 103: +angular.z turns left, so the right wheel has to go faster.
                correction = -msg.angular.z * self.pwm_per_rad
                # Any node can publish /cmd_vel, so the driver enforces its own limit:
                # a wider differential would not fit in the band and mix() would break it.
                correction = max(-self.max_correction, min(self.max_correction, correction))
                left_speed, right_speed = mix(self.base_speed, correction,
                                              self.drive_min, self.max_speed)
        else:
            # The chassis can't reverse or pivot in place, so anything that isn't
            # a valid "drive forward" means stop.
            left_speed, right_speed = 0, 0
            self.moving_since = None

        line = f'{left_speed},{right_speed}\n'
        try:
            self.serial_conn.write(line.encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().error(f'Lost serial connection: {e}', throttle_duration_sec=2.5)
        self.get_logger().info(
            f'v={msg.linear.x:.2f} w={msg.angular.z:+.2f} -> {left_speed},{right_speed}', throttle_duration_sec=1)

    def timeout_callback(self):
        # /cmd_vel stops when the line is lost; stop now instead of waiting for the Arduino watchdog.
        if self.last_cmd_time is not None and time.monotonic() - self.last_cmd_time > self.cmd_timeout:
            self.last_cmd_time = None   # one stop is enough; the next /cmd_vel drives again
            self.moving_since = None
            try:
                self.serial_conn.write(b'0,0\n')
            except serial.SerialException as e:
                self.get_logger().error(f'Lost serial connection: {e}', throttle_duration_sec=2.5)
            self.get_logger().warn(f'No /cmd_vel for {self.cmd_timeout} s, stopping', throttle_duration_sec=2.5)

    def destroy_node(self):
        try:
            self.serial_conn.write(b'0,0\n')   # don't leave the last command running
            self.serial_conn.flush()
        except serial.SerialException:
            pass   # port already gone; the Arduino watchdog stops the motors anyway
        self.serial_conn.close()
        super().destroy_node()


def mix(base, correction, drive_min, max_speed):
    left = base + correction
    right = base - correction

    # Slide both wheels together until they fit in the usable band. Shifting keeps
    # the gap between them, which is the whole steering command; clamping each
    # wheel on its own would silently throw that steering away.
    low, high = min(left, right), max(left, right)
    if low < drive_min:
        left += drive_min - low
        right += drive_min - low
    elif high > max_speed:
        left -= high - max_speed
        right -= high - max_speed

    return int(round(left)), int(round(right))


def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
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
