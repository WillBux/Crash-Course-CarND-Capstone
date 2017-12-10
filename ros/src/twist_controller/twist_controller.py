import rospy
from yaw_controller import YawController


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # Let's start with basic YawController
        self.YC = YawController(kwargs['wheel_base'],
                                kwargs['steer_ratio'],
                                kwargs['min_speed'],
                                kwargs['max_lat_accel'],
                                kwargs['max_steer_angle'])

        self.last_throttle = 0.0
        self.last_brake = 0.0
        self.last_steer = 0.0

        self.max_throttle = 1.0
        self.max_brake = 100.0

        pass

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        lx_target  = kwargs['lx_target']
        az_target  = kwargs['az_target']
        lx_current = kwargs['lx_current']
        az_current  = kwargs['az_current']

        # ========================================
        # Steering controller
        # ========================================
        if lx_target > 0:
            steer = self.YC.get_steering(lx_target, az_target, lx_current)
        else:
            steer = self.last_steer

        # ========================================
        # Throttle/Brake controller
        # ========================================
        throttle = 1.0
        brake = 0.0
        # Speed up if we are slow
        if lx_current < lx_target:
            brake = 0
            throttle = self.max_throttle
            #rospy.loginfo("Speeding: {} {}".format(throttle,brake))

        # keep up the last value if we are on target
        elif lx_current == lx_target:
            brake = 0
            throttle = self.last_throttle
            rospy.loginfo("Coasting: {} {}".format(throttle,brake))

        # Slow down if we are speeding up
        else:
            throttle = 0
            brake = min(self.last_brake + 25, self.max_brake)
            #rospy.loginfo("Slowing: {} {}".format(throttle,brake))



        #throttle = 1.0
        #brake = 0.0
        self.last_throttle = throttle
        self.last_brake = brake
        self.last_steer = steer

        return throttle, brake, steer
