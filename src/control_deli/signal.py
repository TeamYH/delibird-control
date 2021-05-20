#!/usr/bin/env python

import rospy
import roslaunch
from std_msgs.msg import String
import os
import sys
from multiprocessing import Process
#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

map_name = "deli_map"

clean_launch_file = "/home/hj/catkin_ws/src/Clean-robot-turtlebot3/clean_robot/launch/clean_work.launch"
slam_launch_file = "/home/hj/catkin_ws/src/Clean-robot-turtlebot3/clean_robot/launch/auto_slam.launch"
nav_launch_file = "/home/hj/catkin_ws/src/Clean-robot-turtlebot3/clean_robot/launch/navigation.launch"
serving_launch_file = "/home/hj/catkin_ws/src/Clean-robot-turtlebot3/clean_robot/launch/serving.launch"
explore_launch_file = "/home/hj/catkin_ws/src/Clean-robot-turtlebot3/clean_robot/launch/exploration.launch"

class ProcessListener(roslaunch.pmon.ProcessListener):
    def processs_died(self, name, exit_code):
        global process_generate_running
        process_generate_running = False
        rospy.logwarn("%s died with code %s", name, exit_code)

        
def init_launch(launchfile, process_listener):
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(
        uuid,
        [launchfile],
        process_listeners=[process_listener],
    )

    return launch


if __name__ == "__main__":
    rospy.init_node("control_deli")
    r = rospy.Rate(1)

    rospy.loginfo("start")

    clean_launch = init_launch(clean_launch_file, ProcessListener())
    explore_launch = init_launch(explore_launch_file, ProcessListener())
    slam_launch = init_launch(slam_launch_file, ProcessListener())
    nav_launch = init_launch(nav_launch_file, ProcessListener())
    serving_launch = init_launch(serving_launch_file, ProcessListener())

    while True:
        #rospy.spin()
        data = rospy.wait_for_message("/web_signal", String)

        rospy.loginfo("data: %s", data)

        if data.data == "mapstart":
            rospy.loginfo("received signal: make a map")
            slam_launch.start()
            explore_launch.start()
        elif data.data == "mapsave":
            #os.system("rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: ""map""}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'")

            #rospy.sleep(60.)
            rospy.loginfo("received signal: map save")
            os.system("rosrun map_server map_saver -f /home/hj/test3_deli_map")
            explore_launch.shutdown()

        elif data.data == "mapstop":
            rospy.loginfo("received signal: map stop")
            slam_launch.shutdown()

        elif data.data == "cleanstart":
            rospy.loginfo("received signal: clean mode start")
            clean_launch.start()

        elif data.data == "cleanstop":
            rospy.loginfo("received signal: clean stop")
            clean_launch.shutdown()

        elif data.data == "opentable":
            rospy.loginfo("received signal: open table")
            nav_launch.start()

        elif data.data == "closetable":
            rospy.loginfo("received signal: close table")
            nav_launch.shutdown()
        
        elif data.data == "servingstart":
            rospy.loginfo("received signal: serving start")
            serving_launch.start()

        elif data.data == "servingstop":
            rospy.loginfo("received signal: serving stop")
            serving_launch.shutdown()


        r.sleep()


