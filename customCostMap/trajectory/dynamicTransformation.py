#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from  tf2_msgs.msg import TFMessage
import tf
transform = TransformStamped()
transform.header.frame_id = 'base_link'  # Parent frame (static)
transform.child_frame_id = 'dynamicTrajectoryMap'  # Child frame (dynamic)

tf_broadcaster = tf2_ros.TransformBroadcaster()
def odomCB(msg):
    global transform
    transform.header.stamp = rospy.Time.now()
    # transform.transform.translation.x = msg.pose.pose.position.x
    # transform.transform.translation.y = msg.pose.pose.position.y
    # transform.transform.translation.z = msg.pose.pose.position.z
    # transform.transform.rotation.x    = msg.pose.pose.orientation.x
    # transform.transform.rotation.y    = msg.pose.pose.orientation.y
    # transform.transform.rotation.z    = msg.pose.pose.orientation.z
    # transform.transform.rotation.w    = msg.pose.pose.orientation.w
    # tf_broadcaster.sendTransform(transform)
    # print(transform)

rospy.init_node('dynamic_transform_publisher')

rospy.Subscriber("odom", Odometry, odomCB)








listener = tf.TransformListener()





rate=rospy.Rate(1)
while not rospy.is_shutdown():
    listener.waitForTransform( "odom","base_link", rospy.Time(), rospy.Duration(4.0))

    (trans, rot) = listener.lookupTransform( "odom","base_link", rospy.Time(0))
    transform.transform.translation.x = trans[0]
    transform.transform.translation.y = trans[1]
    transform.transform.translation.z = trans[2]
    transform.transform.rotation.x    = rot[0]
    transform.transform.rotation.y    = rot[1]
    transform.transform.rotation.z    = rot[2]
    transform.transform.rotation.w    = rot[3]
    tf_broadcaster.sendTransform(transform)
    print(trans,rot)
    rate.sleep()

