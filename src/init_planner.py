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

    count = rospy.get_param("init_planner/num_agent")
    goal_count = 2
    pubs = []
    pose_pubs = []
    target_pub = []
    # this is basically initializing all the subscribers for counting the
    # number of cars and publishers for initiailizing pose and goal points.
    for i in range(count):
        name = rospy.get_param("init_planner/car" + str(i+1) + "/name")
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

    car_pose = [[0, 5, -math.pi/2], [0, 0, math.pi/2]]
    goal_pose = [[[1, 3, -math.pi/2], [3, 3, math.pi]], [[1, 2, math.pi/2], [2, 4, 0]]]
    
    # car_pose = [[3, 0, math.pi/2], [0, 0, math.pi/2]]
    # goal_pose = [[[1, 5, math.pi/2], [0, 0, math.pi/2]], [[2, 5, math.pi/2], [3, 0, math.pi/2]]]

    for i in range(count):
        now = rospy.Time.now()
        carmsg = PoseStamped()
        carmsg.header.frame_id = "/map"
        carmsg.header.stamp = now

        carmsg.pose.position.x = car_pose[i][0]
        carmsg.pose.position.y = car_pose[i][1]
        carmsg.pose.position.z = 0.0
        carmsg.pose.orientation = angle_to_quaternion(car_pose[i][2])

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
    goalmsg.scale = 2
    goalmsg.minx = 0
    goalmsg.miny = 0
    goalmsg.maxx = 3
    goalmsg.maxy = 5
    for i in range(goal_count):
        goalmsg.goals.append(PoseArray())
        for j in range(2):
            goal = Pose()
            goal.position.x = goal_pose[i][j][0]
            goal.position.y = goal_pose[i][j][1]
            goal.position.z = 0.0
            goal.orientation = angle_to_quaternion(goal_pose[i][j][2])
            goalmsg.goals[i].poses.append(goal)
    goal_pub.publish(goalmsg)
    if(testing_standalone):
        for i in range(2):
            goalmsg.goals[i].header.frame_id = "/map"    
            target_pub[i].publish(goalmsg.goals[i])  # use when testing local planner as a standalone system
    # rospy.spin()
    
