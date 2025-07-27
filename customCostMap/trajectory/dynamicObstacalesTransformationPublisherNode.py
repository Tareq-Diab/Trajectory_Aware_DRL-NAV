#!/usr/bin/env python3
"""
This script is used to publish Transformations for all dynamic obstacles in the environemnt 
it prodcast the transformations relative to the odom frame of the robot
"""
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf
from gazebo_msgs.msg import ModelStates
from  tqdm import tqdm
import time

rospy.init_node('DynamicObstacleTransformationNode')
print("CreatingDynamicObstacle")
msg=rospy.wait_for_message("/gazebo/model_states",ModelStates)
dynamic_obstacle_dict={}
dynamic_obstacle_dict["actor_plugin"]=TransformStamped() 
dynamic_obstacle_dict["actor_plugin"].header.frame_id = 'odom'
dynamic_obstacle_dict["actor_plugin"].child_frame_id="actor_plugin"

for n in tqdm(msg.name):
    if n[:12]=="animated_box":
        dynamic_obstacle_dict[n]=TransformStamped() 
        dynamic_obstacle_dict[n].header.frame_id = 'odom'
        dynamic_obstacle_dict[n].child_frame_id=n
    if n[:15]=="humanoid_actor_":
        dynamic_obstacle_dict[n]=TransformStamped() 
        dynamic_obstacle_dict[n].header.frame_id = 'odom'
        dynamic_obstacle_dict[n].child_frame_id=n


tf_broadcaster = tf2_ros.TransformBroadcaster()
last_t=rospy.Time.now()
def odomCB(msg):

    global dynamic_obstacle_dict , last_t

    
    for n in msg.name: 

        if n[:12]=="animated_box": 
        
            t=rospy.Time.now()
            dynamic_obstacle_dict[n].header.stamp = t if t>last_t else last_t+ rospy.Duration(nsecs=1) #BUG causes a redundant time stamp issue
            last_t=dynamic_obstacle_dict[n].header.stamp
            dynamic_obstacle_dict[n].transform.translation.x = msg.pose[msg.name.index(n)].position.x
            dynamic_obstacle_dict[n].transform.translation.y = msg.pose[msg.name.index(n)].position.y
            dynamic_obstacle_dict[n].transform.translation.z = msg.pose[msg.name.index(n)].position.z
            dynamic_obstacle_dict[n].transform.rotation.x    = msg.pose[msg.name.index(n)].orientation.x
            dynamic_obstacle_dict[n].transform.rotation.y    = msg.pose[msg.name.index(n)].orientation.y
            dynamic_obstacle_dict[n].transform.rotation.z    = msg.pose[msg.name.index(n)].orientation.z
            dynamic_obstacle_dict[n].transform.rotation.w    = msg.pose[msg.name.index(n)].orientation.w
            tf_broadcaster.sendTransform(dynamic_obstacle_dict[n])

        if  n[:15]=="humanoid_actor_":
            print(n)

            t=rospy.Time.now()
            dynamic_obstacle_dict[n].header.stamp = t if t>last_t else last_t+ rospy.Duration(nsecs=1) #BUG causes a redundant time stamp issue
            last_t=dynamic_obstacle_dict[n].header.stamp
            dynamic_obstacle_dict[n].transform.translation.x = msg.pose[msg.name.index(n)].position.x
            dynamic_obstacle_dict[n].transform.translation.y = msg.pose[msg.name.index(n)].position.y
            dynamic_obstacle_dict[n].transform.translation.z = msg.pose[msg.name.index(n)].position.z
            dynamic_obstacle_dict[n].transform.rotation.x    = msg.pose[msg.name.index(n)].orientation.x
            dynamic_obstacle_dict[n].transform.rotation.y    = msg.pose[msg.name.index(n)].orientation.y
            dynamic_obstacle_dict[n].transform.rotation.z    = msg.pose[msg.name.index(n)].orientation.z
            dynamic_obstacle_dict[n].transform.rotation.w    = msg.pose[msg.name.index(n)].orientation.w
            tf_broadcaster.sendTransform(dynamic_obstacle_dict[n])

sub=rospy.Subscriber("/gazebo/model_states", ModelStates, odomCB)
rate=rospy.Rate(1)
while not rospy.is_shutdown():
    rate.sleep()