#!/usr/bin/env python

import rospy
import roslaunch
from std_msgs.msg import String
import os
import sys
from geometry_msgs.msg import PoseStamped


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
    rospy.init_node("control_deli_2")
    r = rospy.Rate(1)

    rospy.loginfo("start")
    
    while True:
        #rospy.spin()
        data = rospy.wait_for_message("/goal_signal", PoseStamped)

        rospy.loginfo("data: %s", data)
        
        
        r.sleep()


