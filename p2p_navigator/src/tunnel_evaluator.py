#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import LaserScan
import time
from nav_msgs.msg import Odometry
import random
import numpy as np 
from tf.transformations import quaternion_from_euler , euler_from_quaternion
from nav_msgs.msg import OccupancyGrid 
from map_msgs.msg import OccupancyGridUpdate
from gym.spaces import Box
from copy import copy
from stable_baselines3 import TD3 ,SAC
from trajectory.DOTrajectoryPredectionNode import TrajectoryCalculator , get_tf
from Environment.goal_randomizer import GoalRandomizer ,publish_marker
from scipy.ndimage import rotate
from geometry_msgs.msg import PoseStamped
from threading import Thread
from gazebo_msgs.msg import ModelState
from tunnel_navigator import Navigator
import os
import tqdm
from std_srvs.srv import Empty
from math import sqrt
from std_srvs.srv import Trigger, TriggerResponse

unPauseWorld = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
PauseWorld = rospy.ServiceProxy('/gazebo/pause_physics', Empty)


class DistanceCalculator:
    def __init__(self):

        self.prev_x = None
        self.prev_y = None
        self.total_distance = 0.0
        self.recording = False
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        

        
    def start_recording(self):
        self.recording = True
        self.total_distance = 0.0  
        self.prev_x = None
        self.prev_y = None
        return True
    
    def stop_recording(self):
        self.recording = False
        return self.total_distance
    
    def odom_callback(self, msg):
        if not self.recording:
            return
        
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        
        # If this is the first position, just store it
        if self.prev_x is None or self.prev_y is None:
            self.prev_x = current_x
            self.prev_y = current_y
            return
        
        # Calculate Euclidean distance between previous and current position
        distance = sqrt((current_x - self.prev_x) ** 2 + (current_y - self.prev_y) ** 2)
        
        # Update the total distance
        self.total_distance += distance
        
        # Update the previous position
        self.prev_x = current_x
        self.prev_y = current_y
        

    def run(self):
        rospy.spin()


class RobotHome():
    def __init__(self):
        self.x=0
        self.y=0
        self.z=0
        self.yaw=0
        self.qz=0
        self.qw=0
        self.qx=0
        self.qy=0


        
    def getQFromYaw(self,yaw,roll=0.0,pitch=0.0):
        """_summary_

        Args:
            yaw (_type_): _description_
            roll (float, optional): _description_. Defaults to 0.0.
            pitch (float, optional): _description_. Defaults to 0.0.
        """
        self.qx,self.qy,self.qz,self.qw= quaternion_from_euler(roll, pitch, yaw)
class evalenv:
    def __init__(self):
        pass
        self.modelStateMessage=ModelState()
        self.modelStatePublisher = rospy.Publisher("/gazebo/set_model_state",ModelState , queue_size=1000)
        self.robothome = RobotHome()
        self.robothome.x=rospy.get_param("/x_pos",default=0)
        self.robothome.y=rospy.get_param("/y_pos",default=0)
        self.robothome.z=rospy.get_param("/z_pos",default=0)
        self.robothome.getQFromYaw(rospy.get_param("/Y_pos",default=0))
    def respwnRobotAt(self,x,y,z,w):

        self.modelStateMessage.model_name="bot"
        self.modelStateMessage.pose.position.x=x
        self.modelStateMessage.pose.position.y=y
        self.modelStateMessage.pose.position.z=0.1
        self.modelStateMessage.pose.orientation.x = 0
        self.modelStateMessage.pose.orientation.y = 0
        self.modelStateMessage.pose.orientation.z = z
        self.modelStateMessage.pose.orientation.w = w
        self.modelStatePublisher.publish(self.modelStateMessage)

    def resetRobot(self,randomness:dict=None):
        if randomness is None: 
            self.respwnRobotAt(self.robothome.x,self.robothome.y,self.robothome.qz,self.robothome.qw)
        else : 
            x_noise=randomness["x"]*np.random.uniform(-1,1)
            y_noise=randomness["y"]*np.random.uniform(-1,1)
            _,_,qz_noise,qw_noise=quaternion_from_euler(0, 0, 1.57+np.random.uniform(-randomness["yaw"],randomness["yaw"]))
            self.respwnRobotAt(self.robothome.x+x_noise,self.robothome.y+y_noise,qz_noise,qw_noise)



def evaluate(model_name="",model_type="",stacked_states=None):
    goal_publisher=rospy.Publisher("/move_base_simple/goal",PoseStamped)
    msg=PoseStamped()
    msg.pose.position.x=-9
    msg.pose.position.y=0
    env=evalenv()
    robot = Navigator(model_name,model_type=model_type,stacked_states=stacked_states)
    distance_calculator = DistanceCalculator()
    complete=[]
    distances=[]
    steps_all=[]
    for i in tqdm.tqdm(range(100)): 
        robot.step((0,0))
        unPauseWorld()
        env.resetRobot()
        time.sleep(2)
        
        PauseWorld()
        goal_publisher.publish(msg)
        time.sleep(1)
        if robot.goalFlag:
            distance_calculator.start_recording()
            robot.goalFlag=False
            while not robot.done:
                finished, colided ,steps =robot.pursuitGoal()
                # print("reached")
                break
            complete.append(not (finished and colided))
            distances.append(distance_calculator.stop_recording())
            steps_all.append(steps)


    dist = [f for b, f in zip(complete, distances) if b]
    st=[f for b, f in zip(complete, steps_all) if b]
    print(f"out of 100 : {complete.count(True)} reached , avg distance traveled {np.mean(dist)} , avg steps / time = {np.mean(st)}steps /{np.mean(st)*0.05}s ")
    os.makedirs(model_type, exist_ok=True) if not stacked_states else os.makedirs(model_type+f"_{stacked_states}", exist_ok=True)
    if not stacked_states:
        np.save(f"{model_type}/complete.npy",np.array(complete))
        np.save(f"{model_type}/distances.npy",np.array(distances))
        np.save(f"{model_type}/steps_all.npy",np.array(steps_all))
    else :
        np.save(f"{model_type}_{stacked_states}/complete.npy",np.array(complete))
        np.save(f"{model_type}_{stacked_states}/distances.npy",np.array(distances))
        np.save(f"{model_type}_{stacked_states}/steps_all.npy",np.array(steps_all))



unPauseWorld()
time.sleep(2)
evaluate("sac_diffrobot_sac-2024-07-21 04:09:12.923888",model_type="M-COST")
unPauseWorld()
time.sleep(2)
evaluate("best_tunnel/sac-2024-06-30 21:08:25.506315/sac_diffrobot_sac-2024-06-30 21:08:25.506315",model_type="M-COST")
unPauseWorld()
time.sleep(2)
evaluate("best_tunnel/sac_mono_channel-2024-07-04 08:43:52.408907/td3_diffrobot_sac_mono_channel-2024-07-04 08:43:52.408907",model_type="mono")
unPauseWorld()
time.sleep(2)
evaluate("best_tunnel/sac_stacked-states-2024-07-06 07:09:52.162889/td3_diffrobot_sac_stacked-states-2024-07-06 07:09:52.162889",model_type="mono",stacked_states=4)
unPauseWorld()
time.sleep(2)
evaluate("best_tunnel/sac-v2-raw-lidar-dynamic-tunnel-2024-09-26 19:45:39.959087/sac_diffrobot_sac-v2-raw-lidar-dynamic-tunnel-2024-09-26 19:45:39.959087",model_type="lidar")
evaluate("td3_diffrobot_sac_mono_channel-2024-07-04 08:43:52.408907",model_type="mono")
evaluate("td3_diffrobot_sac_stacked-states-2024-07-02 17:39:16.771158",model_type="mono",stacked_states=4)

