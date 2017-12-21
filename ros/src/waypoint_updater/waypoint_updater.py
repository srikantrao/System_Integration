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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    """
    Subscribers -
    Publishers -
    Call Back methods
        1.
    """
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # TODO: Add other member variables you need below
        # Additional class variables used in the callback functions
        # Used in pose_cb
        self.pose = None
        self.position = None
        self.orientation = None
        self.waypoints = []
        ## Will keep this to get the future waypoints that need to be calculated
        self.final_waypoints_pub = []

        # PoseStamped - Consists of Header and Pose( Point, Quarternion)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        # Lane - Header and List of Waypoints(PoseStamped, TwistedStamped)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        ## The message type is based on rosmsg output
        rospy.Subscriber('/traffic_waypoint', Int32 , self.traffic_cb)
        rospy.Subscriber('/obstable_waypoint', Lane, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Publish the final waypoints
        self.publishFinalWayPoints()


        # Used in waypoints_cb


        rospy.spin()

    def pose_cb(self, msg):

        # Record the current pose
        self.pose = msg.pose
        self.position = msg.pose.position
        self.orientation = msg.pose.orientation
        pass

    def waypoints_cb(self, waypoints):
        # Append the waypoints
        for waypoint in waypoints.waypoints:
            self.waypoints.append(waypoint)
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def publishFinalWayPoints(self):
        rate = rospy.Rate(50)
        # Build a dummy Lane msg
        finalWayPoints = Lane()
        finalWayPoints.header.stamp = rospy.Time().now().to_sec()
        finalWayPoints.header.frame_id ='Dummy'
        new_waypoints = []
        for waypoint in self.waypoints:
            new_waypoint = waypoint
            new_waypoint.twist.twist.linear.x = waypoint.twist.twist.linear.x + 5
            new_waypoints.append(new_waypoint)
        ## This obviously needs to be modified
        finalWayPoints.waypoints = new_waypoints

        ## Publish the temporary final waypoints for now
        self.final_waypoints_pub.publish(finalWayPoints)




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
