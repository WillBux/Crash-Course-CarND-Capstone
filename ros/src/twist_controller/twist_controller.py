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
        
        pass

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        steer = self.YC.get_steering(kwargs['lx_target'],
                                     kwargs['az_target'],
                                     kwargs['lx_current'])
        throttle = 1.0
        brake = 0.0
        
        rospy.loginfo("THROTTLE = {}; BRAKE = {};  STEER = {}".format(throttle,brake,steer))
        
        return throttle, brake, steer
