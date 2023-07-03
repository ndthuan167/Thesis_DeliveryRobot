#!/usr/bin/env python3

import glob
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from utils.msg import goal_indoor


def Create_path():
    rospy.init_node('line_pub_example', anonymous=True)
    rospy.loginfo('Publishing example line')
    rospy.Subscriber("/goal_indoor", goal_indoor, CallbackGoalindoor)
    rospy.spin()

def CallbackGoalindoor(goal_in):

#    global waypoint_0, waypoint_1, waypoint_2
    waypoint_x_0 = goal_in.way_x_0
    waypoint_x_1 = goal_in.way_x_1
    waypoint_x_2 = goal_in.way_x_2

    waypoint_y_0 = -goal_in.way_y_0
    waypoint_y_1 = -goal_in.way_y_1
    waypoint_y_2 = -goal_in.way_y_2

    while not rospy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD

        # marker scale
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # marker color
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # marker orientaiton
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # marker position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        # marker line points
        marker.points = []
        # first point
        first_line_point = Point()
        first_line_point.x = waypoint_x_0
        first_line_point.y = waypoint_y_0
        # first_line_point.z = 0.0
        marker.points.append(first_line_point)
            # second point
        second_line_point = Point()
        second_line_point.x = waypoint_x_1
        second_line_point.y = waypoint_y_1
            # second_line_point.z = 0.0
        marker.points.append(second_line_point)

        third_line_point = Point()
        third_line_point.x = waypoint_x_2
        third_line_point.y = waypoint_y_2
            # third_line_point.z = 0.0
        marker.points.append(third_line_point)

            # four_line_point = Point()
            # four_line_point.x = waypoint_4[0]
            # four_line_point.y = waypoint_4[1]
            # # third_line_point.z = 0.0
            # marker.points.append(four_line_point)
            # Publish the Marker
        pub_line_min_dist.publish(marker)

        rospy.sleep(0.5)

if __name__ == '__main__':
    pub_line_min_dist = rospy.Publisher('path_indoor', Marker, queue_size=1)
    Create_path()
