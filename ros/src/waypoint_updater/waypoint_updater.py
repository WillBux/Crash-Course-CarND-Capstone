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
        self.base_wp      = None        # Let's store the base waypoints here
        self.current_pose = None        # Current Position (full)
        self.current_x    = None        # Current X Position
        self.current_y    = None        # Current Y Position
        self.current_z    = None        # Current Z Position
        self.wp_dist      = None        # List of (dist, waypoint) tuples
        self.wp_vels      = None        # List of velocity targets for each wp
        self.max_vel      = 17.8        # Maximum speed on the track [meters/sec] (40 Mph)
        self.stop_dist    = 50          # Stopping distance (from max_vel to 0)

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
        self.base_wp = wp.waypoints
        self.set_max_vel()
        pass

    # ============================================================
    def set_max_vel(self):
        # Assign maximum velocity to all waypoints
        num_wp = len(self.base_wp)
        self.wp_vels = [self.max_vel for i in range(num_wp)]

        # Stop at the last waypoint
        self.set_stop_vel(num_wp-1)

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
            #rospy.loginfo("RESUMING FROM STOP")
            return

        # Reduce the velocity of waypoints leading up to stop_wp
        self.set_stop_vel(stop_wp)

        #rospy.loginfo("STOPPING AT = {}".format(stop_wp))
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
            count = 0
            max_dist = 1.0e8
            for i,wp in enumerate(self.base_wp):
                xj = wp.pose.pose.position.x
                yj = wp.pose.pose.position.y
                zj = wp.pose.pose.position.z

                dij = math.sqrt( (xi-xj)**2 + (yi-yj)**2 + (zi-zj)**2 )

                if dij < max_dist:
                    max_dist = dij
                    next_wp = i

                wp_dist.append((dij, wp, count))
                count += 1

            self.wp_dist = sorted(wp_dist)
            self.next_wp = next_wp
            return True

        except Exception as e:
            #rospy.logwarn(e.message)
            return False

    # ============================================================
    def finalize_wp(self):
        # Build 'final waypoints' message and publish

        success = self.wp_distances()
        if not success:
            return

        max_vel = 11.0

        # Construct a lane message
        msg = Lane()
        msg.header.frame_id = '/world'
        msg.header.stamp = rospy.Time.now()

        n = 0
        i = 0

        # Select a subset of waypoints and add to message
        while n < LOOKAHEAD_WPS:
            i += 1
            # Select waypoints ahead of car position [closest wp]
            if self.wp_dist[i][2] >= self.wp_dist[0][2]:
                wpi = self.wp_dist[i][1]

                # Assign maximum speed
                wpi.twist.twist.linear.x = self.wp_vels[self.wp_dist[i][2]]
                msg.waypoints.append(wpi)
                n += 1

        self.final_waypoints_pub.publish(msg)

        pass

    #def get_waypoint_velocity(self, waypoint):
    #    return waypoint.twist.twist.linear.x
    #
    #def set_waypoint_velocity(self, waypoints, waypoint, velocity):
    #    waypoints[waypoint].twist.twist.linear.x = velocity
    #
    #def distance(self, waypoints, wp1, wp2):
    #    dist = 0
    #    dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
    #    for i in range(wp1, wp2+1):
    #        dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
    #        wp1 = i
    #    return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
