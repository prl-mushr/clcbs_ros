#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Quaternion,PoseWithCovarianceStamped
import time
import math
from clcbs_ros.msg import GoalPoseArray
from std_msgs.msg import String


testing_standalone = False  # set to false if testing the whole system as one unit. When testing as standalone, don't launch clcbs_node

def angle_to_quaternion(angle):
    """Convert an angle in radians into a quaternion _message_."""
    return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))


if __name__ == "__main__":
    rospy.init_node("init_planner")
    rospy.sleep(1)

    num_agent = rospy.get_param("/init_planner/num_agent")
    num_waypoint = rospy.get_param("/init_planner/num_waypoint")
    pubs = []
    pose_pubs = []
    target_pub = []
    # this is basically initializing all the subscribers for counting the
    # number of cars and publishers for initializing pose and goal points.
    for i in range(num_agent):
        name = rospy.get_param("/init_planner/car" + str(i+1) + "/name")
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

        start_pose = rospy.get_param("/init_planner/car" + str(i + 1) + "/start")
        carmsg.pose.position.x = start_pose[0]
        carmsg.pose.position.y = start_pose[1]
        carmsg.pose.position.z = 0.0
        carmsg.pose.orientation = angle_to_quaternion(start_pose[2])

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
    goalmsg.scale = rospy.get_param("/init_planner/scale")
    goalmsg.minx = rospy.get_param("/init_planner/minx")
    goalmsg.miny = rospy.get_param("/init_planner/miny")
    goalmsg.maxx = rospy.get_param("/init_planner/maxx")
    goalmsg.maxy = rospy.get_param("/init_planner/maxy")
    for i in range(num_agent):
        goalmsg.goals.append(PoseArray())
        waypoints = rospy.get_param("/init_planner/car" + str(i + 1) + "/waypoints")
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
