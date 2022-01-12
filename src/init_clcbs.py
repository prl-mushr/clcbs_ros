#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Quaternion, PoseWithCovarianceStamped
import random
from clcbs_ros.msg import GoalPoseArray
from copy import deepcopy


testing_standalone = False  # set to false if testing the whole system as one unit. When testing as standalone, don't launch clcbs_node

use_mocap_start = False  # set to true to use the mocap position as the start position, false to use the testcase start position

INVALID_POSE_Z = -100.0
mocap_poses = []

def mocap_callback(p, i):
    mocap_poses[i] = p

def angle_to_quaternion(angle):
    """Convert an angle in radians into a quaternion _message_."""
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))


if __name__ == "__main__":
    rospy.init_node("init_clcbs")
    rospy.sleep(1)

    num_agent = rospy.get_param("/init_clcbs/num_agent")
    num_waypoint = rospy.get_param("/init_clcbs/num_waypoint")
    randomness = rospy.get_param("/init_clcbs/randomness")
    x_min = rospy.get_param("/init_clcbs/minx")
    x_max = rospy.get_param("/init_clcbs/maxx")
    y_min = rospy.get_param("/init_clcbs/miny")
    y_max = rospy.get_param("/init_clcbs/maxy")
    pubs = []
    pose_pubs = []
    target_pub = []
    mocap_subs = []
    # this is basically initializing all the subscribers for counting the
    # number of cars and publishers for initializing pose and goal points.
    for i in range(num_agent):
        name = rospy.get_param("/init_clcbs/car" + str(i+1) + "/name")
        print(name)
        publisher = rospy.Publisher(
            name + "/init_pose", PoseStamped, queue_size=5)
        pubs.append(publisher)
        pose_publisher = rospy.Publisher(
            name + "/initialpose", PoseWithCovarianceStamped, queue_size=5)
        pose_pubs.append(pose_publisher)
        # target point publishing in case we want to test nhttc standalone
        target = rospy.Publisher(
            name + "/waypoints", PoseArray, queue_size=5)
        target_pub.append(target)
        if use_mocap_start:
            mocap_poses.append(PoseStamped())
            mocap_poses[i].pose.position.z = INVALID_POSE_Z
            mocap_sub = rospy.Subscriber("/" + name + "/car_pose", PoseStamped, callback=mocap_callback, callback_args=i)
            mocap_subs.append(mocap_sub)

    goal_pub = rospy.Publisher(
        "/clcbs_ros/goals", GoalPoseArray, queue_size=5)

    obs_pub = rospy.Publisher(
        "/clcbs_ros/obstacles", PoseArray, queue_size=5)
    rospy.sleep(1)

    for i in range(num_agent):
        now = rospy.Time.now()
        carmsg = PoseStamped()
        carmsg.header.frame_id = "/map"
        carmsg.header.stamp = now

        if use_mocap_start and mocap_poses[i].pose.position.z != INVALID_POSE_Z:  # make sure mocap has published for this agent
            carmsg.pose = deepcopy(mocap_poses[i].pose)
            carmsg.pose.position.z = 0.0
        else:
            start_pose = rospy.get_param("/init_clcbs/car" + str(i + 1) + "/start")
            carmsg.pose.position.x = min(x_max, max(x_min, start_pose[0] + random.uniform(-randomness[0], randomness[0])))
            carmsg.pose.position.y = min(y_max, max(y_min, start_pose[1] + random.uniform(-randomness[1], randomness[1])))
            carmsg.pose.position.z = 0.0
            carmsg.pose.orientation = angle_to_quaternion(start_pose[2] + random.uniform(-randomness[2], randomness[2]))

        cur_pose = PoseWithCovarianceStamped()
        cur_pose.header.frame_id = "/map"
        cur_pose.header.stamp = now
        cur_pose.pose.pose = carmsg.pose
        print(carmsg)
        rospy.sleep(1)
        pubs[i].publish(carmsg)
        pose_pubs[i].publish(cur_pose)


    now = rospy.Time.now()
    obsmsg = PoseArray()
    obsmsg.header.frame_id = "/map"
    obsmsg.header.stamp = now
    obs_pub.publish(obsmsg)
    goalmsg = GoalPoseArray()
    goalmsg.header.frame_id = "/map"
    goalmsg.header.stamp = now
    goalmsg.num_agent = num_agent
    goalmsg.num_waypoint = num_waypoint
    goalmsg.scale = rospy.get_param("/init_clcbs/scale")
    goalmsg.minx = x_min
    goalmsg.miny = y_min
    goalmsg.maxx = x_max
    goalmsg.maxy = y_max
    for i in range(num_agent):
        goalmsg.goals.append(PoseArray())
        waypoints = rospy.get_param("/init_clcbs/car" + str(i + 1) + "/waypoints")
        for j in range(num_waypoint):
            goal = Pose()
            goal.position.x = waypoints[j][0]
            goal.position.y = waypoints[j][1]
            goal.position.z = 0.0
            goal.orientation = angle_to_quaternion(waypoints[j][2])
            goalmsg.goals[i].poses.append(goal)
    goal_pub.publish(goalmsg)
    if(testing_standalone):
        for i in range(2):
            goalmsg.goals[i].header.frame_id = "/map"
            target_pub[i].publish(goalmsg.goals[i])  # use when testing local planner as a standalone system
