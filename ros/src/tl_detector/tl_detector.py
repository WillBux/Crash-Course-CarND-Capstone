#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
from math import sqrt

# Since the traffic light classifier takes time to process, we need to
# lower the threshold a bit
STATE_COUNT_THRESHOLD = 2

DEBUG = False

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.sight_distance = 300    # Let's say we can only see some distance ahead
        self.stop_waypoints = []

        self.save_images = False
        self.image_number = 0
        self.last_red_time = None
        self.last_stop_wp = -1
        self.red_elapsed = 0.0
        self.last_light = TrafficLight.UNKNOWN

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    # ========================================================================
    def waypoints_cb(self, wp):
        # Store all the waypoints in class
        self.waypoints = wp.waypoints

        # Also let's associate all the light/stops with waypoints
        stop_line_positions = self.config['stop_line_positions']

        # Get distances from stop lines
        for xi, yi in stop_line_positions:

            idx = -1
            max_dist = None

            for wp in self.waypoints:
                xj = wp.pose.pose.position.x
                yj = wp.pose.pose.position.y

                idx += 1

                dij = sqrt((xj-xi)**2 + (yj-yi)**2)

                if max_dist is None or dij < max_dist:
                    max_dist = dij
                    closest = idx

            if DEBUG:
                rospy.loginfo("ASSOCIATED WP = {} at {}".format(closest, max_dist))
            self.stop_waypoints.append(closest)

    # ========================================================================
    def traffic_cb(self, msg):
        self.lights = msg.lights

    # ========================================================================
    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state

        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))

            if DEBUG and state < 3:
                tl_colors = ['RED', 'YELLOW', 'GREEN', 'UNKNOWN', 'UNKNOWN']
                rospy.loginfo("TL DETECTED: {} COLOR = {}".format(state, tl_colors[state]))

        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    # ========================================================================
    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        # Find the closest waypoint from current pose
        if self.waypoints is None:
            return 0

        max_dist = None
        closest_wp = None
        xi = pose.position.x
        yi = pose.position.y
        zi = pose.position.z

        for idx, wp in enumerate(self.waypoints):
            xj = wp.pose.pose.position.x
            yj = wp.pose.pose.position.y
            zj = wp.pose.pose.position.z

            dij = sqrt( (xi-xj)**2 + (yi-yj)**2 + (zi-zj)**2 )

            if max_dist is None or dij < max_dist:
                max_dist = dij
                closest_wp = idx

        #rospy.loginfo("Closest waypoint = {} at {} m".format(closest_wp, max_dist))
        return closest_wp

    # ========================================================================
    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        #rospy.loginfo("CAMERA IMAGE = {}".format(cv_image.shape))

        if self.save_images:
            filnam = "cam_img-{:04d}.png".format(self.image_number)
            cv2.imwrite(filnam, cv_image)
            self.image_number += 1

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    # ========================================================================
    def get_closest(self, pose, landmarks):
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_distance = None
        stop_idx = None

        if self.waypoints is None:
            return stop_idx, stop_distance

        # Car position
        xi = self.waypoints[pose].pose.pose.position.x
        yi = self.waypoints[pose].pose.pose.position.y

        # Get distances from stop lines
        idx = -1
        for xj, yj in landmarks:
            dij = sqrt((xj-xi)**2 + (yj-yi)**2)
            idx += 1

            if stop_distance is None or dij < stop_distance:
                stop_distance = dij
                stop_idx = idx

        #rospy.loginfo("STOP IDX = {} at {} m".format(stop_idx, stop_distance))
        return stop_idx, stop_distance

    # ========================================================================
    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
        else:
            return -1, TrafficLight.UNKNOWN

        # Find nearest stopline ahead of car_position
        light_positions = []
        for light in self.lights:
            xl = light.pose.pose.position.x
            yl = light.pose.pose.position.y
            light_positions.append((xl, yl))

        light_idx, light_distance = self.get_closest(car_position, light_positions)

        if self.stop_waypoints is None or light_idx is None or light_distance > self.sight_distance:
            return -1, TrafficLight.UNKNOWN

        # Waypoints associated with stop lines and lights
        stop_wp = self.stop_waypoints[light_idx]
        light_wp = stop_wp + 15

        # Do not classify if we have passed the light
        if light_wp < car_position:
            return -1, TrafficLight.UNKNOWN

        state = self.get_light_state(self.lights[light_idx])

        ### ========================================================================
        ### Update last red time if we are at a new light
        ##if stop_wp != self.last_stop_wp:
        ##    state = TrafficLight.RED
        ##    self.last_red_time = rospy.get_time()
        ##    new_signal = True
        ##else:
        ##    state = self.last_light
        ##
        ### Check if we are continuing from red or just started
        ##if self.last_light == TrafficLight.RED:
        ##    # Continuing from red
        ##    self.red_elapsed = rospy.get_time() - self.last_red_time
        ##
        ##    # Turn green if we have stayed on red long enough
        ##    if self.red_elapsed > 15.0:
        ##        state = TrafficLight.GREEN
        ##
        ##else:
        ##    # Just started red
        ##    self.last_red_time = rospy.get_time()

        self.last_light = state
        self.last_stop_wp = stop_wp

        # ========================================================================


        return stop_wp, state

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
