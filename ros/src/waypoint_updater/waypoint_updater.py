#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


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

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Enabling traffic/obstacles later on
        # rospy.Subscriber('/traffic_waypoint', Traffic, self.traffic_cb)
        # rospy.Subscriber('/obstacle_waypoint',Obstacle, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        rate = rospy.rate(10)
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
    def waypoints_cb(self, waypoints):
        # Callback for base waypoints. Let's just store in class
        self.base_wp = waypoints

    # ============================================================
    def wp_distances(self):
        # Compute the distances to all base waypoints and store in class
        wp_dist = []
        xi = self.current_x
        yi = self.current_y
        zi = self.current_z
        for wp in self.waypoints:
            xj = wp.pose.pose.position.x
            yj = wp.pose.pose.position.y
            zj = wp.pose.pose.position.z

            dij = math.sqrt( (xi-xj)**2 + (yi-yj)**2 + (zi-zj)**2 )

            wp_dist.append((dij, wp))

        self.wp_dist = sorted(wp_dist)

        pass

    # ============================================================
    def finalize_wp(self):
        # Build 'final waypoints' message and publish
        self.wp_distances()

        # Construct a lane message
        msg = Lane()
        msg.header.frame_id = '/world'
        msg.header.stamp = rospy.Time.now()

        for i in range(LOOKAHEAD_WPS):
            wpi = self.wp_disp[i][1]
            msg.waypoints.append(wpi)

        self.final_waypoints_pub.publist(msg)

        pass

    # ============================================================
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
