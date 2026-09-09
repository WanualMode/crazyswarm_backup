from collections import deque
import math
from pathlib import Path
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Float64MultiArray, String
import yaml

from ament_index_python.packages import get_package_share_directory
from crazyflie_py import Crazyswarm


HOVER_GRAVITY = 9.81
HOVER_CALIBRATION_WINDOW_SEC = 2.0
HOVER_BUFFER_KEEP_SEC = 3.0
HOVER_MIN_SAMPLES = 20
THRUST_INDEX_RANGE = range(13, 17)
BODY_TORQUE_INDEX_RANGE = range(27, 30)
HOVER_TRIGGER_RETRY_PERIOD_SEC = 1.0
HOVER_TRIGGER_RETRY_COUNT = 3


class SuInterface(Node):
    def __init__(self):
        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.cf = self.swarm.allcfs.crazyflies[0]

        super().__init__('su_interface')

        self.subscription = self.create_subscription(
            String,
            'keyboard_input',
            self.keyboard_callback,
            10
        )
        self.debug_subscription = self.create_subscription(
            Float64MultiArray,
            '/data_logging_msg_debug',
            self.debug_callback,
            50
        )
        self.calibration_publisher = self.create_publisher(
            Float64MultiArray,
            'cf2/hover_calibration',
            10,
        )
        self.param_client = self.create_client(SetParameters, '/crazyflie_server/set_parameters')
        self.hover_samples = deque()
        self.latest_hover_calibration = None
        self.pending_hover_calibration_repeats = 0
        self.disarm_retry_timer = None
        self.su_params_path = Path(get_package_share_directory('crazyflie')) / 'config' / 'su_params.yaml'
        self.current_mass, self.current_com_off_x, self.current_com_off_y = self._load_current_hover_calibration()
        self.hover_calibration_retry_timer = self.create_timer(
            HOVER_TRIGGER_RETRY_PERIOD_SEC,
            self.retry_hover_calibration_trigger,
        )
        self.get_logger().info('su_interface node ready.')

    def keyboard_callback(self, msg):
        if not msg.data:
            return
        input_char = msg.data[0]
        if input_char == 'o':
            self.cf.arm(True)
            self.get_logger().info('ARM command sent.')
        elif input_char == 'p':
            self.request_disarm()
        elif input_char == 'f':
            self.trigger_hover_calibration()

    def debug_callback(self, msg):
        if len(msg.data) <= BODY_TORQUE_INDEX_RANGE.stop - 1:
            return

        motor_thrust = [msg.data[i] for i in THRUST_INDEX_RANGE]
        body_torque = [msg.data[i] for i in BODY_TORQUE_INDEX_RANGE]
        if not all(math.isfinite(value) for value in motor_thrust + body_torque):
            return

        sample = {
            'time': self.get_clock().now().nanoseconds * 1e-9,
            'hover_thrust': sum(motor_thrust),
            'tau_x': body_torque[0],
            'tau_y': body_torque[1],
        }
        self.hover_samples.append(sample)
        self._trim_hover_samples(sample['time'])

    def _trim_hover_samples(self, now_sec):
        cutoff_sec = now_sec - HOVER_BUFFER_KEEP_SEC
        while self.hover_samples and self.hover_samples[0]['time'] < cutoff_sec:
            self.hover_samples.popleft()

    def _load_current_hover_calibration(self):
        try:
            with self.su_params_path.open('r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to read current hover calibration from {self.su_params_path}: {exc}'
            )
            return (float('nan'), 0.0, 0.0)

        su_wrench = (
            data.get('robot_types', {})
            .get('cf21', {})
            .get('firmware_params', {})
            .get('su_wrench', {})
        )
        mass = float(su_wrench.get('mass', float('nan')))
        com_off_x = float(su_wrench.get('comOffX', 0.0))
        com_off_y = float(su_wrench.get('comOffY', 0.0))
        return (mass, com_off_x, com_off_y)

    def trigger_hover_calibration(self):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        self._trim_hover_samples(now_sec)
        window_start_sec = now_sec - HOVER_CALIBRATION_WINDOW_SEC
        samples = [sample for sample in self.hover_samples if sample['time'] >= window_start_sec]

        if len(samples) < HOVER_MIN_SAMPLES:
            self.get_logger().warning(
                'HOVER CALIBRATION skipped: only %d samples in the last %.1f s on /data_logging_msg_debug',
                len(samples),
                HOVER_CALIBRATION_WINDOW_SEC,
            )
            return

        hover_thrust = sum(sample['hover_thrust'] for sample in samples) / len(samples)
        tau_x = sum(sample['tau_x'] for sample in samples) / len(samples)
        tau_y = sum(sample['tau_y'] for sample in samples) / len(samples)

        if not math.isfinite(hover_thrust) or hover_thrust <= 1e-6:
            self.get_logger().warning(
                'HOVER CALIBRATION skipped: invalid hover thrust %.6f N',
                hover_thrust,
            )
            return

        self.current_mass, self.current_com_off_x, self.current_com_off_y = self._load_current_hover_calibration()
        current_mass = self.current_mass
        current_com_off_x = self.current_com_off_x
        current_com_off_y = self.current_com_off_y

        measured_mass = hover_thrust / HOVER_GRAVITY
        # Hover calibration updates only the CoM offsets. Keep the mass loaded
        # from su_params.yaml instead of replacing it with the hover estimate.
        mass = current_mass
        # tau_input already includes the currently configured CoM compensation term.
        # In hover, the remaining body-torque mismatch corresponds to the error
        # between the current CoM estimate and the true CoM offset.
        delta_com_x = -tau_y / hover_thrust
        delta_com_y = tau_x / hover_thrust
        com_off_x = current_com_off_x - delta_com_x
        com_off_y = current_com_off_y - delta_com_y

        if not all(math.isfinite(value) for value in (mass, com_off_x, com_off_y)):
            self.get_logger().warning(
                'HOVER CALIBRATION skipped: invalid estimate mass=%.6f, comOffX=%.6f, comOffY=%.6f',
                mass,
                com_off_x,
                com_off_y,
            )
            return

        calibration = {
            'mass': mass,
            'com_off_x': com_off_x,
            'com_off_y': com_off_y,
            'hover_thrust': hover_thrust,
            'tau_x': tau_x,
            'tau_y': tau_y,
            'delta_com_x': delta_com_x,
            'delta_com_y': delta_com_y,
            'sample_count': len(samples),
        }
        self.latest_hover_calibration = calibration
        self.current_mass = mass
        self.current_com_off_x = com_off_x
        self.current_com_off_y = com_off_y
        self.pending_hover_calibration_repeats = max(0, HOVER_TRIGGER_RETRY_COUNT - 1)

        self.send_hover_calibration_trigger(calibration, log_request=True)
        self.get_logger().info(
            'HOVER CALIBRATION local result: samples=%d, thrust=%.4f N, tau_input=(%.5f, %.5f) N*m, '
            'configured mass=%.4f kg, hover-estimated mass=%.4f kg, '
            'current comOffXY=(%.5f, %.5f) m, delta comOffXY=(%.5f, %.5f) m, '
            'new comOffXY=(%.5f, %.5f) m',
            calibration['sample_count'],
            calibration['hover_thrust'],
            calibration['tau_x'],
            calibration['tau_y'],
            calibration['mass'],
            measured_mass,
            current_com_off_x,
            current_com_off_y,
            calibration['delta_com_x'],
            calibration['delta_com_y'],
            calibration['com_off_x'],
            calibration['com_off_y'],
        )

    def send_hover_calibration_trigger(self, calibration, *, log_request):
        msg = Float64MultiArray()
        msg.data = [
            calibration['mass'],
            calibration['com_off_x'],
            calibration['com_off_y'],
        ]
        self.calibration_publisher.publish(msg)
        if log_request:
            self.get_logger().info(
                'HOVER CALIBRATION trigger published to cf2/hover_calibration: samples=%d, '
                'mass=%.4f kg, comOffXY=(%.5f, %.5f) m',
                calibration['sample_count'],
                calibration['mass'],
                calibration['com_off_x'],
                calibration['com_off_y'],
            )

    def retry_hover_calibration_trigger(self):
        calibration = self.latest_hover_calibration
        if calibration is None or self.pending_hover_calibration_repeats <= 0:
            return

        self.send_hover_calibration_trigger(calibration, log_request=False)
        self.pending_hover_calibration_repeats -= 1

    def request_disarm(self):
        self.cf.notifySetpointsStop(remainValidMillisecs=0)
        self.cf.arm(False)
        self.get_logger().info('DISARM command sent.')

        if self.disarm_retry_timer is not None:
            self.disarm_retry_timer.cancel()

        self.disarm_retry_timer = self.create_timer(0.12, self._retry_disarm_once)

    def _retry_disarm_once(self):
        self.cf.notifySetpointsStop(remainValidMillisecs=0)
        self.cf.arm(False)
        self.get_logger().info('DISARM retry sent.')
        if self.disarm_retry_timer is not None:
            self.disarm_retry_timer.cancel()
            self.disarm_retry_timer = None

    def shutdown(self):
        self.cf.land(targetHeight=0.04, duration=2.5)
        self.timeHelper.sleep(3.0)



def main(args=None):
    node = SuInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
