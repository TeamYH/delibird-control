#!/usr/bin/env python

import rospy
import roslaunch
from std_msgs.msg import String
import os
import sys
from geometry_msgs.msg import PoseStamped
import time

map_name = "deli_map"

clean_launch_file = "/home/hj/catkin_ws/src/Clean-robot-turtlebot3/clean_robot/launch/clean__gazebo.launch"
#slam_launch_file = "/home/hj/catkin_ws/src/turtlebot3/turtlebot3_slam/launch/turtlebot3_slam.launch"
slam_launch_file = "/home/hj/catkin_ws/src/Clean-robot-turtlebot3/clean_robot/launch/auto_slam.launch"
nav_launch_file = "/home/hj/catkin_ws/src/Clean-robot-turtlebot3/clean_robot/launch/table.launch"
serving_launch_file = "/home/hj/catkin_ws/src/Clean-robot-turtlebot3/clean_robot/launch/serving.launch"
explore_launch_file = "/home/hj/catkin_ws/src/Clean-robot-turtlebot3/clean_robot/launch/exploration.launch"

clean_status = 0
slam_status = 0
nav_status = 0
serve_status = 0
explore_status = 0


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
            if slam_status == 0:
                slam_launch.start()
                slam_status = 1
            if explore_status == 0:
                explore_launch.start()
                explore_status = 1
        elif data.data == "mapsave":
            rospy.loginfo("received signal: map save")
            os.system("rosrun map_server map_saver -f /home/hj/auto_test_map")

        elif data.data == "mapstop":
            rospy.loginfo("received signal: map stop")
            slam_launch.shutdown()
            explore_launch.shutdown()
            slam_launch = init_launch(slam_launch_file, ProcessListener())
            explore_launch = init_launch(explore_launch_file, ProcessListener())
            slam_status = 0
            explore_status = 0

        elif data.data == "cleanstart":
            rospy.loginfo("received signal: clean mode start")
            if clean_status == 0:
                clean_launch.start()
                clean_status = 1

        elif data.data == "cleanstop":
            rospy.loginfo("received signal: clean stop")
            clean_launch.shutdown()
            clean_launch = init_launch(clean_launch_file, ProcessListener())
            clean_status = 0

        elif data.data == "opentable":
            rospy.loginfo("received signal: open table")
            if nav_status == 0:
                nav_launch.start()
                nav_status = 1

        elif data.data == "closetable":
            rospy.loginfo("received signal: close table")
            nav_launch.shutdown()
            nav_launch = init_launch(nav_launch_file, ProcessListener())
            nav_status = 0
        
        elif data.data == "servestart":
            rospy.loginfo("received signal: serving start")
            if serve_status == 0:
                serving_launch.start()
                serve_status = 1

        elif data.data == "servestop":
            rospy.loginfo("received signal: serving stop")
            serving_launch.shutdown()
            serving_launch = init_launch(serving_launch_file, ProcessListener())
            serve_status = 0

        #rendering map in clean mode


        #r.sleep()


