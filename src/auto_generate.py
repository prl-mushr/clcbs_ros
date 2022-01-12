#!/usr/bin/env python
import roslaunch
import rospy
from geometry_msgs.msg import PoseArray
import os
import subprocess
import numpy as np
import time
from bs4 import BeautifulSoup as bs
import matplotlib.pyplot as plt

rospy.init_node('autogenerate_clcbs', anonymous=True)

plan_pub = False
def pose_callback(msg):
    global plan_pub
    plan_pub = True  # plan has been published

## this helps us track whether the launch file has been completely shut down yet or not
class ProcessListener(roslaunch.pmon.ProcessListener):
    def process_died(self, name, exit_code):
        rospy.logwarn("%s died with code %s", name, exit_code)

def adjust_launch_file(filename, i):
    with open(filename, 'r') as f:
        data = f.read()
    bs_data = bs(data, 'xml')
    rec_name = bs_data.find("arg", {"name":"record_name"})
    rec_name["default"] = "clcbs_data_"+str(i+1)

    output = bs_data.prettify()  # prettify doesn't actually make it prettier.
    with open(filename, 'w') as f:
        f.write(output)

subscriber_ = rospy.Subscriber("/car1/waypoints", PoseArray, pose_callback)  # subscribe to car-pose to check collisions for first car (plans pubbed simultaneously)

N = 100
timeout = 30 # 30 second time out

clcbs_launchfile = "/home/stark/catkin_mushr/src/clcbs_ros/launch/clcbs_ros.launch"
init_launchfile = "/home/stark/catkin_mushr/src/clcbs_ros/launch/init_clcbs_record.launch"
bagdir = "/home/stark/catkin_mushr/src/clcbs_ros/bags"  # the cases where the bag files are saved. This is also present in the launch file i think.

failed = 0

for i in range(N):
    print("iteration " + str(i) + " starting")
    iter_succ = False
    while(not iter_succ):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch_clcbs = roslaunch.parent.ROSLaunchParent(uuid,
                                                  [clcbs_launchfile],
                                                  process_listeners=[ProcessListener()])
        launch_clcbs.start()
        time.sleep(4)


        adjust_launch_file(init_launchfile, i)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid,
                                                  [init_launchfile],
                                                  process_listeners=[ProcessListener()])
        plan_pub = False  # reset plan published flag before starting new cycle
        launch.start()  # launch the damn thing
        start_time = time.time()
        while( time.time() - start_time < timeout and not plan_pub):
            rospy.sleep(1)
        launch.shutdown()
        launch_clcbs.shutdown()
        print("waiting 10 seconds for clean exit")
        time.sleep(10)
        if plan_pub:
            iter_succ = True  # if plan wasn't published we shall try again until it is.
        else:
            ## if the planner failed: find the latest bag file and delete it. Also count the number of times the planner failed
            files = os.listdir(bagdir)
            files.sort(key = lambda x: os.path.getmtime(bagdir+'/'+x))
            os.remove(bagdir + '/' + files[-1])  # delete last created file
            failed += 1  # up the counter on failed cases

## rename all bags
files = os.listdir(bagdir)
for i in range(N):
    source = bagdir + '/' + files[i]
    dest = bagdir + '/' + 'clcbs_data_' + str(i) + '.bag'
    os.rename(source, dest)


print("failed cases:", failed)
print("failure rate: ", 100 * failed / (failed + N))

'''
('failed cases:', 321)
('failure rate: ', 76)
'''
