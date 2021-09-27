#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Quaternion,PoseWithCovarianceStamped
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
    # this is basically initializing all the subscribers for counting the
    # number of cars and publishers for initializing pose and goal points.
    for i in range(num_agent):
        name = rospy.get_param("/init_deadlock/car" + str(i+1) + "/name")
        print(name)
        publisher = rospy.Publisher(
            name + "/init_pose", PoseStamped, queue_size=5)
        pubs.append(publisher)
        pose_publisher = rospy.Publisher(
            name + "/initialpose", PoseWithCovarianceStamped, queue_size=5)
        pose_pubs.append(pose_publisher)
        waypoint_pub = rospy.Publisher(name + "/waypoints", PoseArray, queue_size=5)
        waypoint_pubs.append(waypoint_pub)

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

    for i in range(num_agent):
        goalmsg = PoseArray()
        now = rospy.Time.now()
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
