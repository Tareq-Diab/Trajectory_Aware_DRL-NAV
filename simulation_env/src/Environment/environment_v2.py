#! /usr/bin/env python3
"""_summary_
continious evironemnt that implement a simulation based stepping mechanism that accuretly control the solving steps.
"""
import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import LaserScan
import time
from std_srvs.srv import Empty
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Bool , Int32
from gazebo_msgs.srv import SetModelState
import math
from nav_msgs.msg import Odometry
import random
import numpy as np 
from tf.transformations import quaternion_from_euler , euler_from_quaternion
import yaml
from rospkg import RosPack
from rosgraph_msgs.msg import Clock
import os
from trajectory.DOTrajectoryPredectionNode import TrajectoryCalculator , get_tf

packagepath=RosPack()
try : 
    path=packagepath.get_path("simulation_env")
    print(path) 
except : 
    path="Environment"

class RobotHome():
    def __init__(self,robotSpawnRandomness:str="/robotSpawn.Yaml"):
        self.x=0
        self.y=0
        self.z=0
        self.yaw=0
        self.qz=0
        self.qw=0
        self.qx=0
        self.qy=0
        self.__spawnInfoFile=open(path+robotSpawnRandomness,"r")
        self.spawnInfo=yaml.full_load(self.__spawnInfoFile)

        
    def getQFromYaw(self,yaw,roll=0.0,pitch=0.0):
        """_summary_

        Args:
            yaw (_type_): _description_
            roll (float, optional): _description_. Defaults to 0.0.
            pitch (float, optional): _description_. Defaults to 0.0.
        """
        self.qx,self.qy,self.qz,self.qw= quaternion_from_euler(roll, pitch, yaw)
class EnvironmentElements:
    def __init__(self,elemntsFile:str="/dynamicElements.Yaml"):
        self.__elemntsfile=open(elemntsFile,"r")
        self.elemntsInfo=yaml.full_load(self.__elemntsfile)

class Info :
    def __init__(self):
        pass
class Reward:
    def __init__(self):
        pass
class space:
    def __init__(self,shape,low,high):
        self.shape=[shape,0]
        self.low=low
        self.high=high
class Environment ():
    """_summary_
    """
    def __init__(self,elemntsFile:str="/dynamicElements.Yaml",controlFrequency=20, **kwargs):

        rospy.init_node("environment")

        self.minLaserRangeBeforCollision=0
        #-------------------------------------------variables----------------------------------------
        self.seed = kwargs.pop('seed', '') if "seed" in kwargs else 42
        self.controlFrequency=controlFrequency
        self.robothome = RobotHome()
        self.robothome.x=rospy.get_param("/x_pos",default=0)
        self.robothome.y=rospy.get_param("/y_pos",default=0)
        self.robothome.z=rospy.get_param("/z_pos",default=0)
        self.robothome.getQFromYaw(rospy.get_param("/Y_pos",default=0))
        
        self.randomiseEnvironement=rospy.get_param("/randomize_env",default=True)
        self.randomiseRobotSpawning=rospy.get_param("/randomiseRobotSpawning",default=True)


        self.resetWorld = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.pauseWorld = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unPauseWorld = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.unPauseWorld()
        
        self.modelStatePublisher = rospy.Publisher("/gazebo/set_model_state",ModelState , queue_size=1000)
        self.collision_pub =rospy.Publisher("environment/colided",Bool,queue_size=1)
        self.robotSpeedPublisher=rospy.Publisher("cmd_vel",Twist,queue_size=1)

        rospy.Subscriber("/odom",Odometry,self.odometry_cb,tcp_nodelay=True,queue_size=1)
        
        scan_for_dangerous_distances=rospy.Subscriber('/scan',LaserScan,callback=self.scanCB)
        self.trajectory_costmap_calculator=TrajectoryCalculator()
        self.modelStateMessage=ModelState()
        self.odom=Odometry()

        self.minLaserRangeBeforCollision=0

        self.lase_res=360
        self.done=False
        self.info={"is_terminal":False,"is_failure":False}
        self.elements=EnvironmentElements(path+elemntsFile)
        self.opservation=np.zeros(self.lase_res+5)
        self.laserNormalizationValue=1
        self.deviationNormalizationValue=1.7
        self.colided=False
        self.n_steps=0
        self.maxstep=2000 
        self.x_twist_normalization=1
        self.z_twist_normalization=1
        self.steppingTime=0.02
        self.action=Twist()
        self.envOffsets={"x":0,"y":0,"yaw":0}
        self.goal={"x":-9,"y":0}
        self.reachGoalThreshold=0.5
        #-------------------------------------------variables----------------------------------------
        self.distance_max=abs(self.goal["x"]) 
        self.nS,self.nA=(self.lase_res+5),2
        self.observation_space=space((self.lase_res+5),-1,1)
        self.action_space=space(2,np.array([-0.1,-1.0],dtype=np.float32),np.array([1.0,1.0],dtype=np.float32))
        self.lastStepTimeStamp=0
        self.reward_debug={"distance":0,"proximity_punishment":0,"orientation":0,"speed":0,"colision_reward":0,"reachreward":0}
        self.__setSeed(self.seed)
        print("environment intiated")
        self.lastcolided=time.time()
        self.colisioncooldown=0
        
        time.sleep(5)
        self.stepped=True

        rospy.Subscriber("/clock_control/Stepped", Bool, self.callback,tcp_nodelay=True)
        self.stepper = rospy.Publisher("/clock_control/stepper", Int32, queue_size=1)
        self.steppsmsg=Int32()
        self.steppsmsg.data=int((1/self.controlFrequency)/0.01)
    #-------------------------------------------------  Environment specific methods ---------------------------------
    def callback(self,msg):
        self.stepped =True
    def reset(self):
        """_summary_

        Returns:
            _type_: _description_
        """
        self.unPauseWorld()
        self.info={"is_terminal":False,"is_failure":False}
        self.n_steps=0
        if self.randomiseEnvironement:
            self.__randomizeEnvironment(self.elements.elemntsInfo)
        else:self.__randomizeEnvironment()

        if self.randomiseRobotSpawning:
            self.__resetRobot(self.robothome.spawnInfo)
        else:self.__resetRobot()
        time.sleep(0.5*(1/8)) 
        self.done=False
        self.colided=False
        self.goal={"x":-9,"y":0} 
        self.distance_max=((self.goal["x"]- self.odom.pose.pose.position.x)**2+(self.goal["y"]- self.odom.pose.pose.position.y)**2)**0.5
        self.isp={"x": self.odom.pose.pose.position.x,"y":self.odom.pose.pose.position.y}

        try:
            self.odometry_cb(rospy.wait_for_message('/odom', Odometry, timeout=1))
        except : 
            pass

        self.pauseWorld()
        self.reward_debug={"distance":0,"proximity_punishment":0,"orientation":0,"speed":0,"colision_reward":0,"reachreward":0}
        self.old_distance_to_goal=(((self.odom.pose.pose.position.x-self.goal["x"])**2)+((self.odom.pose.pose.position.y-self.goal["y"])**2) )**0.5
    
        return self.__getObservation() 
    def step(self,action):
        self.n_steps+=1
        linear,angular=action

        self.__setSpeed(linear,angular)
        self.stepper.publish(self.steppsmsg)
        while (not self.stepped):
            pass
        self.stepped=False
        return  self.__getObservation(),self.__rewardCalculation(), self.__isDone(),self.info
    
    def __getObservation(self): 


        self.opservation[:self.lase_res]=self.laser_data/30

        x=self.odom.pose.pose.position.x
        y=self.odom.pose.pose.position.y
        d=abs(((self.goal["y"]-self.isp["y"])*x) - ((self.goal["x"]-self.isp["x"])*y) + (( self.goal["x"]*self.isp["y"] )-( self.goal["y"]*self.isp["x"] )) ) / np.sqrt((self.goal["y"]-self.isp["y"])**2+(self.goal["x"]-self.isp["x"])**2)
        self.opservation[0]=d/self.deviationNormalizationValue 

        if   self.n_steps>self.maxstep: 
            self.done=True
            self.info["is_failure"]=True
            self.info["is_terminal"]=False
        if self.reachedGoal(self.goal["x"],self.goal["y"],self.reachGoalThreshold) or self.colided:
            self.done=True
            self.info["is_terminal"]=True
            self.info["is_failure"]=False


        #speed            normalized
        self.opservation[self.lase_res+1]=self.odom.twist.twist.linear.x/self.x_twist_normalization
        self.opservation[self.lase_res+2]=self.odom.twist.twist.angular.z/self.z_twist_normalization
        self.opservation[self.lase_res+3]=self.__getThetaRobotGoal()

        self.distance_to_goal=(((self.odom.pose.pose.position.x-self.goal["x"])**2)+((self.odom.pose.pose.position.y-self.goal["y"])**2) )**0.5
        distance_reward = 1 - (self.distance_to_goal / self.distance_max)**0.5
        self.opservation[self.lase_res+4]=distance_reward
        return self.opservation
     
                                                                           

    def __getThetaRobotGoal(self): #ANCHOR - here
        t,r=self.trajectory_costmap_calculator.transformation_manager.listener.lookupTransform("odom","base_link", rospy.Time(0))
        TFM=get_tf(t,r)
        goal_to_robot=np.dot(TFM,np.array([self.goal["x"] ,self.goal["y"], 0,1] )) 
        x,y=goal_to_robot[:2]
        theta=(np.arctan2(x,y)/np.pi)
        return  theta 
    def __rewardCalculation(self):
        ####################################################### Robot Behaviour ###############################################
        #----------------------------------------------------------Robot progression---------------------------------------------
        self.distance_to_goal=(((self.odom.pose.pose.position.x-self.goal["x"])**2)+((self.odom.pose.pose.position.y-self.goal["y"])**2) )**0.5
        progressionReward=-(self.distance_to_goal-self.old_distance_to_goal)/(1.0/self.controlFrequency)
        reward=progressionReward
        self.old_distance_to_goal=self.distance_to_goal
        self.reward_debug["distance"]=progressionReward
        #----------------------------------------------------------Robot Speed---------------------------------------------

        if self.action.linear.x < 0.1:
            reward+=-abs(self.action.linear.x) if (np.min(self.laser_data)>0.3) else abs(self.action.linear.x)
            self.reward_debug["speed"]+=-abs(self.action.linear.x) if (np.min(self.laser_data)>0.3) else abs(self.action.linear.x)
        else :
            reward+=self.action.linear.x*0.8 if (np.min(self.laser_data)>0.3) else -self.action.linear.x*0.8
            self.reward_debug["speed"]+=self.action.linear.x*0.8 if (np.min(self.laser_data)>0.3) else abs(self.action.linear.x)
        #-----------------------------------------------------------Robot Orientation  ------------------------------------------
        yaw=self.__getThetaRobotGoal()

        #if the robot is near an obstacle change the punishing factor for going in the wrong direction
        dist=np.min(self.laser_data) 
        factor=1 if dist>1 else 2 

        #calculate the reward based on angle all reversed orientation takes -1.1/factor and all forward orientation takes reward relative to the angle of the robot whis is 
        # -0/factor when robot heading is towards the goal and -1/factor when facing +-90 degree from the goal
        reward+=(-1.1/factor) if abs(yaw)>0.5 else (-abs(yaw)/0.5)/factor
        self.reward_debug["orientation"]=(-1.1/factor) if abs(yaw)>0.5 else (-abs(yaw)/0.5)/factor
        #------------------------------------------------------------proximity punishment---------------------------------------
        if np.min(self.laser_data) < 1.0:
            reward+= -((1-np.min(self.laser_data))*1.8)**np.e
            
            self.reward_debug["proximity_punishment"]+=-((1-np.min(self.laser_data))*1.8)**np.e
        else :
            reward+= 0.1
            self.reward_debug["proximity_punishment"]=0.1    
        ####################################################### TERMINAL STATES ###############################################
        #-----------------------------------------------------------Collision  -----------------------------------------------
        if self.colided:
            self.reward_debug["colision_reward"]=-100
            # print("updated collision")
            reward = -100

        #-----------------------------------------------------------reaching Goal --------------------------------------------
        if self.reachedGoal(self.goal["x"],self.goal["y"],self.reachGoalThreshold):
            reward=100+   min( (100*((1000-self.n_steps)/1000)),40)
            self.reward_debug["reachreward"]=100+ min( (100*((1000-self.n_steps)/1000)),40)
            # print("updated reach")

        #========================================================================================================================
        debug_message="{} and reward is : {} \r".format(self.reward_debug,reward)
        debug_message="distance: {:05.4f}, speed: {:05.4f},orientation:, {:05.4f}, proximity: {:05.4f},colision_reward: {:05.4f}, reachreward: {:05.4f},total: {:05.4f}\r".format(self.reward_debug["distance"],
                                                                                                                                        self.reward_debug["speed"],
                                                                                                                                        self.reward_debug["orientation"],
                                                                                                                                        self.reward_debug["proximity_punishment"],
                                                                                                                                        self.reward_debug["colision_reward"],
                                                                                                                                        self.reward_debug["reachreward"],
                                                                                                                             reward
                                                                                                                                        )

        return reward
    





    def __isDone(self):
        return self.done
    
    def __setSpeed(self,linear,angular):
        self.action.linear.x=linear
        self.action.angular.z=angular
        self.robotSpeedPublisher.publish(self.action)
    
    def __setSeed(self,seed:int=42):
        np.random.seed(seed)
    
    def __randomizeEnvironment(self,dynamicElements:dict=None): 
        """_summary_

        Args:
            dynamicElements (dict, optional): _description_. Defaults to None.
        """
        if dynamicElements is None : 
            return
        else :
            for key,v in dynamicElements.items():
                if v["randomisation"]["method"]=="axis":
                    axis=v["randomisation"]["type"]
                    value=v["randomisation"]["value"]
                    center=v["center"]

                    self.spawn_obj_randomly_in_range_along_axis(key,center,axis,value)
                if v["randomisation"]["method"]=="raduis":
                    value=v["randomisation"]["value"]
                    center=v["center"]
                    self.spawn_obj_randomly_in_raduis(key,center,value)
                
                
        
    def collision_cb(self,msg):
        pass




    def odometry_cb(self,msg):
        self.odom.pose.pose.position.x=msg.pose.pose.position.x
        self.odom.pose.pose.position.y=msg.pose.pose.position.y
        self.odom.pose.pose.orientation=msg.pose.pose.orientation
        self.odom.twist.twist.linear=msg.twist.twist.linear
        self.odom.twist.twist.angular=msg.twist.twist.angular
        
        

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



    def resetEnvironment(self):
        self.resetWorld()

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
    
    def __resetRobot(self,randomness:dict=None):
        self.__setSpeed(0,0)
        if randomness is None: 
            self.respwnRobotAt(self.robothome.x,self.robothome.y,self.robothome.qz,self.robothome.qw)
        else : 
            x_noise=randomness["x"]*np.random.uniform(-1,1)
            y_noise=randomness["y"]*np.random.uniform(-1,1)
            _,_,qz_noise,qw_noise=quaternion_from_euler(0, 0, 1.57+np.random.uniform(-randomness["yaw"],randomness["yaw"]))
            self.respwnRobotAt(self.robothome.x+x_noise,self.robothome.y+y_noise,qz_noise,qw_noise)

    def moveObstacaleTo(self,obstacleName,new_x,new_y):
        self.modelStateMessage.model_name=obstacleName
        self.modelStateMessage.pose.position.x=new_x
        self.modelStateMessage.pose.position.y=new_y
        self.modelStateMessage.pose.position.z=0.45
        self.modelStateMessage.pose.orientation.x = 0
        self.modelStateMessage.pose.orientation.y = 0
        self.modelStateMessage.pose.orientation.z = 1
        self.modelStateMessage.pose.orientation.w = 0
        self.modelStatePublisher.publish(self.modelStateMessage)

    def reachedGoal(self,goal_x,goal_y,toleranceRaduis): 
        actualRaduis=math.sqrt((self.odom.pose.pose.position.x-goal_x)**2+(self.odom.pose.pose.position.y-goal_y)**2)
        if actualRaduis>toleranceRaduis:
            return False
        else :
            return True

    
    def spawn_obj_randomly_in_raduis(self,obstacleName,centerListXY,raduis):
        random_ofset_x=random.random()
        random_ofset_y=random.random()
        signx=random.random()
        signy=random.random()
        x_ofset=random_ofset_x*raduis if (signx >0.5) else random_ofset_x*raduis*-1
        y_ofset=random_ofset_y*raduis if (signy >0.5) else random_ofset_y*raduis*-1
        """
            generate a random ofset that can take a positive or negative value based on random 
            variable of the othere axis random ofset =D 
        """
        self.modelStateMessage.model_name=obstacleName
        self.modelStateMessage.pose.position.x=centerListXY[0]+x_ofset
        self.modelStateMessage.pose.position.y=centerListXY[1]+y_ofset
        self.modelStateMessage.pose.position.z=0.45
        self.modelStateMessage.pose.orientation.x = 0
        self.modelStateMessage.pose.orientation.y = 0
        self.modelStateMessage.pose.orientation.z = 1
        self.modelStateMessage.pose.orientation.w = 0
        self.modelStatePublisher.publish(self.modelStateMessage)
    
    def spawn_obj_randomly_in_range_along_axis(self,obstacleName,posListXY,axis,range):
        if axis == "x":
            random_ofset_x=random.random()
            sign=random.random()
            x_ofset=random_ofset_x*range if (sign >0.5) else random_ofset_x*range*-1
            self.modelStateMessage.pose.position.x=posListXY[0]+x_ofset
            self.modelStateMessage.pose.position.y=posListXY[1]
        else :
            random_ofset_y=random.random()
            sign=random.random()
            y_ofset=random_ofset_y*range if (sign >0.5) else random_ofset_y*range*-1
            self.modelStateMessage.pose.position.x=posListXY[0]
            self.modelStateMessage.pose.position.y=posListXY[1]+y_ofset
        """
            generate a random ofset that an take a positive or negative value based on random 
            variable of the othere axis random ofset =D 
        """
        self.modelStateMessage.model_name=obstacleName

        self.modelStateMessage.pose.position.z=0.45
        self.modelStateMessage.pose.orientation.x = 0
        self.modelStateMessage.pose.orientation.y = 0
        self.modelStateMessage.pose.orientation.z = 1
        self.modelStateMessage.pose.orientation.w = 0
        self.modelStatePublisher.publish(self.modelStateMessage)
