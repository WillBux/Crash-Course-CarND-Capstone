#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Class storage
        self.base_wp = None        # Let's store the base waypoints here
        self.current_position = None        # Current Position (full)
        self.next_wp_index = None
        self.next_wp_distance = 0.0
        self.wp_dist      = None        # List of (dist, waypoint) tuples
        self.wp_vels      = None        # List of velocity targets for each wp
        self.wp_last      = None        # List of waypoints publishes last time (to limit search)
        self.wp_num       = 0           # Number of base waypoints
        self.max_vel      = 17.8        # Maximum speed on the track [meters/sec] (40 Mph)
        self.stop_dist    = 100         # Stopping distance (from max_vel to 0)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Enabling traffic/obstacles later on
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint',Obstacle, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_next_wps()
            rate.sleep()

    def pose_cb(self, msg):
        # Store the current coordinates in the class object
        self.current_position = msg.pose.position

        if not self.base_wp:
            return

        self.calc_next_wp_index()

    def waypoints_cb(self, wp):
        # Callback for base waypoints. Let's just store in class
        self.base_wp = wp.waypoints
        self.wp_num = len(wp.waypoints)
        self.set_max_vel()
        pass

    # ============================================================
    def set_max_vel(self):
        # Assign maximum velocity to all waypoints
        self.wp_vels = [self.max_vel for i in range(self.wp_num)]

        # Stop at the last waypoint
        self.set_stop_vel(self.wp_num-1)

        pass

    # ============================================================
    def set_stop_vel(self, stop):
        # Stop at the stopping point
        self.wp_vels[stop] = 0
        xi = self.base_wp[stop].pose.pose.position.x
        yi = self.base_wp[stop].pose.pose.position.y
        zi = self.base_wp[stop].pose.pose.position.z

        ramp_distance = True
        n = 0
        while ramp_distance:
            n += 1
            xj = self.base_wp[stop-n].pose.pose.position.x
            yj = self.base_wp[stop-n].pose.pose.position.y
            zj = self.base_wp[stop-n].pose.pose.position.z

            dij = math.sqrt( (xi-xj)**2 + (yi-yj)**2 + (zi-zj)**2 )

            ramp_vel = self.max_vel*dij/self.stop_dist

            self.wp_vels[stop-n] = min(self.max_vel, ramp_vel)

            # If far enough away, stop ramping up
            if dij > self.stop_dist:
                ramp_distance = False

    # ============================================================
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        stop_wp = msg.data
        if stop_wp < 0:
            # Reset all speeds to maximum velocity
            self.set_max_vel()
            return

        # Reduce the velocity of waypoints leading up to stop_wp
        self.set_stop_vel(stop_wp)

        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass


    def dist(self, a, b):
        return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)

    def calc_next_wp_index(self):
        if not self.next_wp_index:
            self.next_wp_distance, self.next_wp_index = min([(self.dist(self.current_position, wp.pose.pose.position), i)
                                                             for i, wp in enumerate(self.base_wp)])
        else:
            self.next_wp_distance = self.dist(self.current_position, self.base_wp[self.next_wp_index].pose.pose.position)

            # search for next best wp in next LOOKAHEAD_WPS points
            # distance = 0.0
            for i in range(self.next_wp_index + 1, (self.next_wp_index + LOOKAHEAD_WPS)):
                index = i % len(self.base_wp)
                wp = self.base_wp[index]
                distance = self.dist(self.current_position, wp.pose.pose.position)
                if distance < self.next_wp_distance:
                    self.next_wp_index = index
                    self.next_wp_distance = distance
                    break

    def next_wps(self):
        rospy.loginfo("NEXT POINT: {} {}".format(self.next_wp_index, self.next_wp_distance))

        next_wps = [self.base_wp[self.next_wp_index]]

        for i in range(self.next_wp_index + 1, (self.next_wp_index + LOOKAHEAD_WPS)):
            index = i % len(self.base_wp)
            next_wps.append(self.base_wp[index])

        return next_wps

    def publish_next_wps(self):
        if not self.base_wp or not self.next_wp_index:
            return

        next_wps = self.next_wps()

        rospy.loginfo("PUBLISHING NEXT WPS: {} {} {}".format(self.next_wp_index, self.next_wp_distance, len(next_wps)))

        # Construct a lane message
        msg = Lane()
        msg.header.frame_id = '/world'
        msg.header.stamp = rospy.Time.now()

        for wp in next_wps:
            wp.twist.twist.linear.x = MAX_VELOCITY
            msg.waypoints.append(wp)

        self.final_waypoints_pub.publish(msg)

        pass


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
