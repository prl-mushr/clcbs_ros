#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Quaternion,PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
import time
import math

def angle_to_quaternion(angle):
    """Convert an angle in radians into a quaternion _message_."""
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))


if __name__ == "__main__":
    rospy.init_node("init_deadlock")

    num_agent = 4
    pubs = []
    pose_pubs = []
    waypoint_pubs = []
    marker_pubs = []
    # this is basically initializing all the subscribers for counting the
    # number of cars and publishers for initializing pose and goal points.
    for i in range(num_agent):
        name = rospy.get_param("/init_deadlock/car" + str(i + 1) + "/name")
        print(name)
        publisher = rospy.Publisher(
            name + "/init_pose", PoseStamped, queue_size=5)
        pubs.append(publisher)
        pose_publisher = rospy.Publisher(
            name + "/initialpose", PoseWithCovarianceStamped, queue_size=5)
        pose_pubs.append(pose_publisher)
        waypoint_pub = rospy.Publisher(name + "/waypoints", PoseArray, queue_size=5)
        waypoint_pubs.append(waypoint_pub)
        marker_pub = rospy.Publisher(name + "/marker", Marker, queue_size=5)
        marker_pubs.append(marker_pub)

    rospy.sleep(1)

    start_poses = [[0.0, 0.0, math.pi / 4], [2.12132, 0.0, 3 * math.pi / 4], [0.0, 2.12132, -math.pi / 4], [2.12132, 2.12132, -3 * math.pi / 4]]
    goal_poses = [[2.12132, 2.12132, math.pi / 4], [0.0, 2.12132, 3 * math.pi / 4], [2.12132, 0.0, -math.pi / 4], [0.0, 0.0, -3 * math.pi / 4]]
    num_waypoint = 5

    for i in range(num_agent):
        now = rospy.Time.now()
        carmsg = PoseStamped()
        carmsg.header.frame_id = "/map"
        carmsg.header.stamp = now

        carmsg.pose.position.x = start_poses[i][0]
        carmsg.pose.position.y = start_poses[i][1]
        carmsg.pose.position.z = 0.0
        carmsg.pose.orientation = angle_to_quaternion(start_poses[i][2])

        cur_pose = PoseWithCovarianceStamped()
        cur_pose.header.frame_id = "/map"
        cur_pose.header.stamp = now
        cur_pose.pose.pose = carmsg.pose
        print(carmsg)
        pubs[i].publish(carmsg)
        pose_pubs[i].publish(cur_pose)

        marker = Marker()
        marker_size = 0.25
        marker.pose.position.x = goal_poses[i][0]
        marker.pose.position.y = goal_poses[i][1]
        marker.pose.position.z = 0

        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1

        marker_hex = int(rospy.get_param("/init_deadlock/car" + str(i + 1) + "/color"), 16)
        marker.color.r = marker_hex >> 16 & 0xff;
        marker.color.g = marker_hex >> 8 & 0xff;
        marker.color.b = marker_hex & 0xff;
        marker.color.a = 1.0;

        marker.scale.x = marker_size;
        marker.scale.y = marker_size;
        marker.scale.z = marker_size;

        marker.header.frame_id = "/map"
        marker.header.stamp = now
        marker.id = i

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker_pubs[i].publish(marker)

        goalmsg = PoseArray()
        goalmsg.header.stamp = now
        goalmsg.header.frame_id = "/map"
        for j in range(num_waypoint):
            goal = Pose()
            goal.position.x = j * (goal_poses[i][0] - start_poses[i][0]) / num_waypoint + start_poses[i][0]
            goal.position.y = j * (goal_poses[i][1] - start_poses[i][1]) / num_waypoint + start_poses[i][1]
            goal.position.z = 0.001
            goal.orientation = angle_to_quaternion(j * (goal_poses[i][2] - start_poses[i][2]) / num_waypoint + start_poses[i][2])
            goalmsg.poses.append(goal)
        waypoint_pubs[i].publish(goalmsg)
