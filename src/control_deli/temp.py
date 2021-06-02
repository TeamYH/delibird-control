#! /usr/bin/env python
#coding=utf-8
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped

def PoseCallBack(msg):
    data=""
    #Subscribed coordinate information
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    #Subscribed quaternion information, used to indicate the direction
    orien_z = msg.pose.pose.orientation.z
    orien_w = msg.pose.pose.orientation.w

    pub = rospy.Publisher("/amcl_pose", PoseWithCovarianceStamped, queue_size = 10)

    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = "map"
    pose.pose.pose.position.x = x
    pose.pose.pose.position.y = y
    pose.pose.pose.position.z = z

    pose.pose.pose.orientation.w = orien_w
    pose.pose.pose.orientation.z = orien_z

    pub.publish(pose)

    data = str(x) + "," + str(y)+ "," + str(orien_z)+ "," + str(orien_w)
    rospy.loginfo(data)

def PoseSub():
    rospy.init_node('pose_sub',anonymous=False)
    #Monitor topics and handle them in the callback function
    rospy.Subscriber('/initialpose',PoseWithCovarianceStamped,PoseCallBack)
    rospy.spin()
if __name__=='__main__':
    try:
        PoseSub()
    except:
        rospy.loginfo("Error, exiting...")
