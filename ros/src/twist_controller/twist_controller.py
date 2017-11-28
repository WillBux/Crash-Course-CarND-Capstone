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
        self.last_steering = 0.0
        
        pass

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        lx_target  = kwargs['lx_target']
        az_target  = kwargs['az_target']
        lx_current = kwargs['lx_current']
        az_current  = kwargs['az_current']        

        max_steer = 0.15
        
        if lx_target > 0:
            steer = self.YC.get_steering(lx_target, az_target, lx_current)
        else:
            steer = self.last_steer

        # Override for now with constant values
        throttle = 0.75
        brake = 0.0

        # Slow down when turning
        if abs(steer) > 0.1 and lx_current > 5.0:
            rospy.loginfo("Slowing.. {} {}".format(steer,lx_current))
            throttle = 0.5

        # Stop accelerating if too fast on a curve
        if abs(steer) > 0.20 and lx_current > 5.0:
            rospy.loginfo("Off Pedal.. {} {}".format(steer,lx_current))
            throttle = 0.0

        # Slow down further if too fast
        if abs(steer) > 0.25 and lx_current > 10.0:
            rospy.loginfo("Braking!! {} {}".format(steer,lx_current))
            brake = 1.0


        self.last_throttle = throttle
        self.last_brake = brake
        self.last_steer = steer
        
        return throttle, brake, steer
