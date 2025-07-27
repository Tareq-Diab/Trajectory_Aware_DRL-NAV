#! /usr/bin/env python3
"""_summary_
This the point to point navigator handler for the rl agent. 
Used in deployemenused responisble for handeling state and action formulation between the environemnt and the agent ,
 On both simulation and Hardware. 
"""
import rospy
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import LaserScan
import time
import math
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
from std_msgs.msg import Bool , Int32
from collections import deque


class Navigator():
    def __init__(self,path,model_type,stacked_states=None):
        
        rospy.init_node("Navigator")
        time.sleep(1)
        self.type=model_type
        self.minLaserRangeBeforCollision=0
        self.laserNormalizationValue=1
        self.deviationNormalizationValue=1.7
        self.reachGoalThreshold=0.5
        self.x_twist_normalization=1
        self.z_twist_normalization=1
        self.lase_res=360
        self.opservation= np.zeros(5)
        self.image=np.zeros([1,50,50])
        self.opservation_return={"image": Box(low=0,high=1,shape=(2,50,50)), "scalers": Box(low=-1,high=1,shape=(5,))}
        self.opservation_return["image"]=self.image
        self.opservation_return["scalers"]=self.opservation
        self.maxstep=2000 

        self.goalFlag=False

        self.odom=Odometry()
        self.action=Twist()

        self.n_steps=0

        self.robotSpeedPublisher=rospy.Publisher("cmd_vel",Twist,queue_size=1)

        self.trajectory_costmap_calculator=TrajectoryCalculator()
        self.trajectory_costmap_calculator.calculateAndPublish().reshape(50,50)
        rospy.Subscriber("/costmap_node/costmap/costmap_updates",OccupancyGridUpdate,callback=self.costmapCB) 
        rospy.Subscriber("/odom",Odometry,self.odometry_cb,tcp_nodelay=True,queue_size=1)
        rospy.Subscriber("/costmap_node/costmap/costmap",OccupancyGrid,callback=self.costmapCBf)
        rospy.Subscriber('/scan',LaserScan,callback=self.scanCB)
        rospy.Subscriber("/move_base_simple/goal",PoseStamped,callback=self.goalCB)
        self.model=self.load_model(path)
        self.stepper = rospy.Publisher("/clock_control/stepper", Int32, queue_size=1)
        rospy.Subscriber("/clock_control/Stepped", Bool, self.callback,tcp_nodelay=True)

        self.steppsmsg=Int32()
        self.controlFrequency=20

        self.steppsmsg.data=int((1/self.controlFrequency)/0.01) 
        self.stepped =True
        self.stacked_states=stacked_states
        if stacked_states :
            self.temporalStates=deque([np.zeros((1,50,50)).astype(np.uint8) for i in range(stacked_states) ],maxlen=stacked_states)

        if self.type == "lidar":
            self.opservation= np.zeros(365)

    def callback(self,msg):
        self.stepped =True
    def load_model(self,path):
        try:
            model = TD3.load(path)
        except:
            model = SAC.load(path)
        return model

    def goalCB(self,msg):
        """_summary_

        Returns:
            _type_: _description_
        """
        self.n_steps=0
        self.done=False
        self.colided=False

        self.goal={"x":msg.pose.position.x,"y":msg.pose.position.y}
        self.distance_max=((self.goal["x"]- self.odom.pose.pose.position.x)**2+(self.goal["y"]- self.odom.pose.pose.position.y)**2)**0.5
        self.isp={"x": self.odom.pose.pose.position.x,"y":self.odom.pose.pose.position.y}
        publish_marker(1,self.goal["x"],self.goal["y"],0,1,0,0)
        publish_marker(2,self.isp["x"],self.isp["y"],0,0,1,0)

        self.old_distance_to_goal=(((self.odom.pose.pose.position.x-self.goal["x"])**2)+((self.odom.pose.pose.position.y-self.goal["y"])**2) )**0.5
        self.goalFlag=True


    def pursuitGoal(self):
        steps=0
        observation=self.__getObservation()
        while not self.done:
            steps+=1
            action,_=self.model.predict(observation)
            self.step(action)
            observation=self.__getObservation()
        self.step((0,0))
        return True ,self.colided ,steps

    def step(self,action):
        self.n_steps+=1
        linear,angular=action
        self.__setSpeed(linear,angular*2)
        self.stepper.publish(self.steppsmsg)
        while (not self.stepped):
            pass
        self.stepped=False

    def lidar_observation(self):


        self.opservation[:self.lase_res]=self.laser_data/30
        x=self.odom.pose.pose.position.x
        y=self.odom.pose.pose.position.y
        d=abs(((self.goal["y"]-self.isp["y"])*x) - ((self.goal["x"]-self.isp["x"])*y) + (( self.goal["x"]*self.isp["y"] )-( self.goal["y"]*self.isp["x"] )) ) / np.sqrt((self.goal["y"]-self.isp["y"])**2+(self.goal["x"]-self.isp["x"])**2)
        self.opservation[0]=d/self.deviationNormalizationValue 

        if   self.n_steps>self.maxstep: 
            self.done=True

        if self.reachedGoal(self.goal["x"],self.goal["y"],self.reachGoalThreshold) or self.colided:
            self.done=True

        self.opservation[self.lase_res+1]=self.odom.twist.twist.linear.x/self.x_twist_normalization
        self.opservation[self.lase_res+2]=self.odom.twist.twist.angular.z/self.z_twist_normalization
        self.opservation[self.lase_res+3]=self.__getThetaRobotGoal()

        self.distance_to_goal=(((self.odom.pose.pose.position.x-self.goal["x"])**2)+((self.odom.pose.pose.position.y-self.goal["y"])**2) )**0.5
        distance_reward = 1 - (self.distance_to_goal / self.distance_max)**0.5
        self.opservation[self.lase_res+4]=distance_reward
        return self.opservation
    
    def __getObservation(self): 
        if self.type== "lidar":
            return self.lidar_observation()
        x=self.odom.pose.pose.position.x
        y=self.odom.pose.pose.position.y
        d=abs(((self.goal["y"]-self.isp["y"])*x) - ((self.goal["x"]-self.isp["x"])*y) + (( self.goal["x"]*self.isp["y"] )-( self.goal["y"]*self.isp["x"] )) ) / np.sqrt((self.goal["y"]-self.isp["y"])**2+(self.goal["x"]-self.isp["x"])**2)
        self.opservation[0]=d/self.deviationNormalizationValue 
        
        if   self.n_steps>self.maxstep: 
            self.done=True

        if self.reachedGoal(self.goal["x"],self.goal["y"],self.reachGoalThreshold) or self.colided:
            self.done=True

        self.opservation[1]=self.odom.twist.twist.linear.x/self.x_twist_normalization
        self.opservation[2]=self.odom.twist.twist.angular.z/self.z_twist_normalization

        self.opservation[3]=self.__getThetaRobotGoal()

        self.distance_to_goal=(((self.odom.pose.pose.position.x-self.goal["x"])**2)+((self.odom.pose.pose.position.y-self.goal["y"])**2) )**0.5 
        distance_reward = 1 - (self.distance_to_goal / self.distance_max)**0.5 
        self.opservation[4]=distance_reward
        trajectory_costmap=self.trajectory_costmap_calculator.calculateAndPublish().reshape(1,50,50)
        static_costmap=self.image
        yaw_robot=euler_from_quaternion([self.odom.pose.pose.orientation.x,self.odom.pose.pose.orientation.y,self.odom.pose.pose.orientation.z,self.odom.pose.pose.orientation.w])[2]
        result =np.concatenate([self.rotate_image(trajectory_costmap,yaw_robot*(180/np.pi) ) ,self.rotate_image(static_costmap,yaw_robot*(180/np.pi))],axis=0)
        m_cost=np.reshape((result*2.55),(2,50,50)).astype(np.uint8)
        static_costmap=self.rotate_image(static_costmap,yaw_robot*(180/np.pi))
        self.opservation_return["image"]=m_cost  if self.type=="M-COST" else (static_costmap if  self.type=="mono" else None)
        self.opservation_return["scalers"]=self.opservation
        if not self.stacked_states:
            return self.opservation_return
        else :
            return self.stack_states(self.opservation_return) 
    
    def stack_states(self,observation):
        self.temporalStates.append(np.reshape(observation["image"]*2.55,(1,50,50)).astype(np.uint8))
        observation["image"]=np.concatenate(self.temporalStates,axis=0)
        observation["scalers"]=observation["scalers"]
        return observation
    
    def __getThetaRobotGoal(self): #ANCHOR - here
        t,r=self.trajectory_costmap_calculator.transformation_manager.listener.lookupTransform("odom","base_link", rospy.Time(0))
        TFM=get_tf(t,r)
        goal_to_robot=np.dot(TFM,np.array([self.goal["x"] ,self.goal["y"], 0,1] )) 
        x,y=goal_to_robot[:2]
        theta=(np.arctan2(x,y)/np.pi)
        #where /np.pi normalizes between 0 and 1 
        return  theta 

    def rotate_image(self,image, angle):
        """
        Rotate the given image by the specified angle.

        Parameters:
        image (numpy.ndarray): Input image as a 2D or 3D array.
        angle (float): Angle by which to rotate the image in degrees.

        Returns:
        numpy.ndarray: Rotated image.
        """
        if image.ndim == 3 and image.shape[0] == 1:
            # If the image is 3D with a single channel, rotate the 2D slice
            rotated_image = rotate(image[0], angle, reshape=False)
            # Add the channel dimension back
            rotated_image = np.expand_dims(rotated_image, axis=0)
        elif image.ndim == 2:
            # If the image is 2D, rotate it directly
            rotated_image = rotate(image, angle, reshape=False)
        else:
            raise ValueError("Input image must be either a 2D array or a 3D array with shape (1, height, width).")
        
        return rotated_image

    def __setSpeed(self,linear,angular):
        self.action.linear.x=linear
        self.action.angular.z=angular
        self.robotSpeedPublisher.publish(self.action)
    
    def odometry_cb(self,msg):
        self.odom.pose.pose.position.x=msg.pose.pose.position.x
        self.odom.pose.pose.position.y=msg.pose.pose.position.y
        self.odom.pose.pose.orientation=msg.pose.pose.orientation
        self.odom.twist.twist.linear=msg.twist.twist.linear
        self.odom.twist.twist.angular=msg.twist.twist.angular
        
    def costmapCB(self,msg):
        image=np.array(msg.data)
        self.image=image.reshape(1,50,50)
        
    def costmapCBf(self,msg):
        image=np.array(msg.data)
        self.image=image.reshape(1,50,50)
        # print("got_image")

    def scanCB(self,data):
        """_summary_

        Args:
            data (_type_): _description_
        """
        if np.min(data.ranges) < self.minLaserRangeBeforCollision:
            self.collision=True 
        self.laser_data=np.array(data.ranges)
        self.laser_data[self.laser_data==np.inf]=30
        if np.min(self.laser_data)<0.25 :
            self.colided=True
        else :
            self.colided=False

    def reachedGoal(self,goal_x,goal_y,toleranceRaduis): 
        actualRaduis=math.sqrt((self.odom.pose.pose.position.x-goal_x)**2+(self.odom.pose.pose.position.y-goal_y)**2)
        if actualRaduis>toleranceRaduis:
            return False
        else :
            return True

    def threader(self):
        """_summary_
        MUST BE CALLED IN THE MAIN THREAD
        responsiple for pursuing goals
        """
        while True:
            if  self.goalFlag:
                self.goalFlag =False
                while  not self.goalFlag:
                    self.pursuitGoal()
                    print("done")
                    break

if __name__ == "__main__":
    import argp
    #parse model and type

    nav=Navigator("best_p2p/sac-p2p-env_v2_p2p-2024-09-21 17:52:21.524231/sac_p2p_mcost.zip","M-COST")
    while True:

        nav.threader()
