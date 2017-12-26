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
MAX_DECEL = 1.0

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Class storage
        self.base_wp      = None        # Let's store the base waypoints here
        self.current_pose = None        # Current Position (full)
        self.current_x    = None        # Current X Position
        self.current_y    = None        # Current Y Position
        self.current_z    = None        # Current Z Position
        self.wp_dist      = None        # List of (dist, waypoint) tuples
        self.wp_vels      = None        # List of velocity targets for each wp
        self.wp_last      = None        # List of waypoints publishes last time (to limit search)
        self.wp_num       = 0           # Number of base waypoints
        self.stop_dist    = 100.0       # Stopping distance (from max_vel to 0)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Enabling traffic/obstacles later on
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint',Obstacle, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.finalize_wp()
            rate.sleep()

    # ============================================================
    def pose_cb(self, msg):
        # Store the current coordinates in the class object
        self.current_pose = msg
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z

        return

    # ============================================================
    def waypoints_cb(self, wp):
        # Callback for base waypoints. Let's just store in class

        # Set the maximum velocity for track based on receommended
        # speed from waypoint loader. Will automatically reset to
        # 10 mph for chuchlots site
        self.max_vel = wp.waypoints[0].twist.twist.linear.x * 1.60934
        rospy.loginfo("Maximum track speed set to {} mps".format(self.max_vel))

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
            if stop-n < 0:
                break
            xj = self.base_wp[stop-n].pose.pose.position.x
            yj = self.base_wp[stop-n].pose.pose.position.y
            zj = self.base_wp[stop-n].pose.pose.position.z

            dij = math.sqrt( (xi-xj)**2 + (yi-yj)**2 + (zi-zj)**2 )

            ramp_vel = self.max_vel * min(1.0, dij/self.stop_dist)
            if ramp_vel < 1.0:
                ramp_vel = 0.0

            # Set velocity behind the stop line to ramp velocity
            self.wp_vels[stop - n] = ramp_vel

            # Set velocity ahead of stop line to 0 (in case car creeps forward)
            if stop + n - 1 < self.wp_num - 1:
                self.wp_vels[stop + n - 1] = 0.0

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


    # ============================================================
    def wp_distances(self):
        # Compute the distances to all base waypoints and store in class
        try:
            wp_dist = []
            xi = self.current_x
            yi = self.current_y
            zi = self.current_z

            # Let's limit the search in the last published range (if available)
            if self.wp_last is None:
                search_range = [i for i in range(self.wp_num)]
            else:
                search_range = self.wp_last

            for i in search_range:
                wp = self.base_wp[i]
                xj = wp.pose.pose.position.x
                yj = wp.pose.pose.position.y
                zj = wp.pose.pose.position.z

                dij = math.sqrt( (xi-xj)**2 + (yi-yj)**2 + (zi-zj)**2 )

                wp_dist.append((dij, wp, i))

            self.wp_dist = sorted(wp_dist)

            if len(self.wp_dist) > 0:
                return True
            else:
                return False

        except Exception as e:
            #rospy.logwarn(e.message)
            return False

    # ============================================================
    def finalize_wp(self):
        # Build 'final waypoints' message and publish

        success = self.wp_distances()
        if not success:
            return

        # Construct a lane message
        msg = Lane()
        msg.header.frame_id = '/world'
        msg.header.stamp = rospy.Time.now()

        # Select a subset of waypoints and add to message
        self.wp_last = []

        # Add waypoints starting with the closest and incrementing thereon
        i = 0
        n = self.wp_dist[0][2]
        while i < LOOKAHEAD_WPS and n < self.wp_num:
            wpn = self.base_wp[n]
            wpn.twist.twist.linear.x = self.wp_vels[n]
            msg.waypoints.append(wpn)
            self.wp_last.append(n)
            i += 1
            n += 1

        self.final_waypoints_pub.publish(msg)

        pass


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
