import rospy
import math

from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

STEERING_CONTROLLER_KP = 0.8
STEERING_CONTROLLER_KI = .0
STEERING_CONTROLLER_KD = .16

THROTTLE_CONTROLLER_KP = .3
THROTTLE_CONTROLLER_KI = .0
THROTTLE_CONTROLLER_KD = .01

COMPLETE_STOP_TARGET_SPEED_VALUE = 0.35

class Controller(object):
    def __init__(self, *args, **kwargs):
        self.yaw_controller = YawController(kwargs['wheel_base'],
                                            kwargs['steer_ratio'],
                                            kwargs['min_speed'],
                                            kwargs['max_lat_accel'],
                                            kwargs['max_steer_angle'])

        self.accel_limit = kwargs['accel_limit']
        self.decel_limit = kwargs['decel_limit']
        self.brake_deadband = kwargs['brake_deadband']
        self.wheel_radius = kwargs['wheel_radius']
        self.vehicle_mass = kwargs['vehicle_mass']

        self.low_pass_filter_steering = LowPassFilter(0.2, 0.1)
        self.low_pass_filter_lin_vel_target = LowPassFilter(10.0, 1.0)
        self.low_pass_filter_ang_vel_target = LowPassFilter(10.0, 1.0)

        self.throttle_pid_controller = PID(kp=THROTTLE_CONTROLLER_KP,
                                           ki=THROTTLE_CONTROLLER_KI,
                                           kd=THROTTLE_CONTROLLER_KD,
                                           mn=self.decel_limit,
                                           mx=self.accel_limit)
        self.last_steering_angle = 0.0
        self.steering_pid_controller = PID(kp=STEERING_CONTROLLER_KP,
                                           ki=STEERING_CONTROLLER_KI,
                                           kd=STEERING_CONTROLLER_KD,
                                           mn=-kwargs['max_steer_angle'],
                                           mx=kwargs['max_steer_angle'])

        # Set to None to disable dbw log
        self.log_dbw = 'dbw.log'
        self.log_init = rospy.get_time()
        if self.log_dbw:
            with open(self.log_dbw,'w') as f:
                f.write("Time, Throttle, Brake, Steering, LX_taget, LX_current, AZ_target, AZ_current\n")


    def get_steering_angle(self, lx_target, az_target, lx_current, az_current,
                           time_elapsed, current_steering_angle):
        target_steering_angle = self.yaw_controller.get_steering(lx_target, az_target, lx_current)
        steering_error = target_steering_angle - current_steering_angle
        steer = self.steering_pid_controller.step(steering_error, time_elapsed)

        return steer

    def get_throttle_and_brakes(self, lx_target, lx_current, time_elapsed):
        error = lx_target - lx_current
        throttle = self.throttle_pid_controller.step(error, time_elapsed)
        # print("SPEED: Carget: {}, Current: {},  Error: {}, Control: {}".format(lx_target, lx_current, error, throttle))

        if lx_target < COMPLETE_STOP_TARGET_SPEED_VALUE:
            return 0.0, self.throttle_to_brake_torque(-2.0)

        if throttle > 0.0:
            return min(throttle, self.accel_limit), 0.0
        elif throttle >= -self.brake_deadband:
            return 0.0, 0.0
        else:
            return 0.0, self.throttle_to_brake_torque(throttle)

    def throttle_to_brake_torque(self, throttle):
        return self.wheel_radius * self.vehicle_mass * math.fabs(throttle)

    def control(self, *args, **kwargs):

        lx_target = kwargs['lx_target']
        az_target = kwargs['az_target']
        lx_current = kwargs['lx_current']
        az_current = kwargs['az_current']

        # print(lx_target)

        dbw_enabled = kwargs['dbw_enabled']
        time_elapsed = kwargs['time_elapsed']
        current_steering_angle = kwargs['current_steering_angle']

        lx_target = self.low_pass_filter_lin_vel_target.filt(lx_target)
        az_target = self.low_pass_filter_ang_vel_target.filt(az_target)

        MIN_SPEED_WHEN_STEERING_MAKES_SENSE = 4.0

        if dbw_enabled:
            if lx_current > MIN_SPEED_WHEN_STEERING_MAKES_SENSE:
                steering_angle = self.get_steering_angle(lx_target, az_target, lx_current, az_current, time_elapsed,
                                                         current_steering_angle)
            else:
                steering_angle = self.last_steering_angle
            self.last_steering_angle = steering_angle

            throttle, brakes = self.get_throttle_and_brakes(lx_target, lx_current, time_elapsed)

            if self.log_dbw:
                with open(self.log_dbw,'a') as f:
                     current_time = rospy.get_time() - self.log_init
                     f.write("{}, {}, {}, {}, {}, {}, {}, {}\n".format(current_time, throttle, brakes, steering_angle, lx_target, lx_current, az_target, az_current))

            return throttle, brakes, steering_angle
        else:
            self.throttle_pid_controller.reset()
            self.steering_pid_controller.reset()

            return 0.0, 0.0, 0.0
