#!/usr/bin/env python3
"""
This node is responsiple of 
    - calculate dynamic trajectories from their transformations
    - calculting the required transformations for publishing the trajectory costmap
    - publishing the costmap

"""
import numpy as np 
import rospy 
import time
from trajectory.trajectoryCostmap import TrajectoryCostmap
import tf
from tf.transformations import quaternion_from_euler ,euler_from_quaternion
from scipy.spatial.transform import Rotation
from trajectory.dynamicObjectPredictor import DynamicObstaclePredictor


def get_transformationMatrix(trans,rot):
    translation = np.array(trans)
    # Quaternion rotation
    rotation_quaternion = np.array(rot)
    # Convert quaternion to rotation matrix
    rotation_matrix = Rotation.from_quat(rotation_quaternion).as_matrix()
    # Calculate the inverse of the rotation matrix
    rotation_matrix = np.linalg.inv(rotation_matrix)
    # Create transformation matrix
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix
    p=np.array([translation[0],translation[1],0,1])
    translation=np.dot(transformation_matrix,p)
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, 3] = translation[:-1]

    return transformation_matrix

def get_tf(trans,rot):
    translation = np.array(trans)
    rotation_quaternion = np.array(rot)
    rotation_matrix = Rotation.from_quat(rotation_quaternion).as_matrix()
    

    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix
    transformation_matrix[:3, 3] = translation

    return np.linalg.inv(transformation_matrix)

def get_all_dynamic_obtacles_frames(filter_by=["animated_box","humanoid_actor"]):
    frames=[]
    listener = tf.TransformListener()
    time.sleep(1)
    for sen in listener.allFramesAsString().split('.'):
        if len(sen)  > 10:
            name=sen.split(" ")[1]
            for f in filter_by : 
                if name[:len(f)]==f:
                    frames.append(name)
    return frames

class DynamicTrajectoryTransformationManger:
    def __init__(self,frames,parentFrame="odom",robotFrame="base_link"):
        self.frames=frames
        self.parentFrame=parentFrame
        self.robotFrame=robotFrame
        self.listener = tf.TransformListener()
        self.dynamic_tf={frame:None for frame in self.frames}

    def get_dynamic_obstacle_transformations(self):
        """_summary_

        Returns:
            dictionary of the dynamic obstacles transformations
        """
        self.listener.waitForTransform(self.parentFrame,self.frames[0], rospy.Time(), rospy.Duration(1.0))
        for frame in self.frames:
            self.dynamic_tf[frame]= self.listener.lookupTransform( self.parentFrame,frame, rospy.Time(0))[0]
        return self.dynamic_tf

    def get_costmap_transformation(self):
        return self.listener.lookupTransform( self.parentFrame,self.robotFrame, rospy.Time(0))[0]
    
    def get_odom_robot_tf_matrix(self):
        t,r=self.listener.lookupTransform( self.robotFrame,self.parentFrame, rospy.Time(0))
        return get_transformationMatrix(t,r)

def gaussian_2d(x, y, mu, sigma):
    # Calculate the Gaussian PDF
    exponent = -0.5 * ((x - mu[0])**2 / sigma[0]**2 + (y - mu[1])**2 / sigma[1]**2)
    return np.exp(exponent) / (2 * np.pi * sigma[0] * sigma[1])



class TrajectoryCalculator:
    def __init__(self):
        # identifying all available dynamic obstacle frames in the environment
        self.frames = get_all_dynamic_obtacles_frames(filter_by=["animated_box","humanoid_actor"])

        # creating a transformation manager to manage tf calculations for dynamic obstacles
        self.transformation_manager = DynamicTrajectoryTransformationManger(
            frames=self.frames,
            parentFrame="odom",
            robotFrame="base_link"
        )

        # creating a predictor for every dynamic obstacle
        self.predictors = {}
        for frame in self.frames:
            self.predictors[frame] = DynamicObstaclePredictor(100)

        # costmap setup
        self.x, self.y = np.meshgrid(np.linspace(0, 50, 50), np.linspace(0, 50, 50))
        self.pos = np.dstack((self.x, self.y))

        # initialize costmap
        self.roscostmap = TrajectoryCostmap("Customcostmap2", 50, 50, Xposition=-2.5, Yposition=-2.5)

        # tf listener setup
        self.listener = tf.TransformListener()

        # initialize predictor
        self.predictor = DynamicObstaclePredictor(100)

    def calculate(self):
        dynamic_obstacle_tfs=self.transformation_manager.get_dynamic_obstacle_transformations()

        z=np.zeros([50,50])
        for obstacle_tf in dynamic_obstacle_tfs.keys():
            trans=dynamic_obstacle_tfs[obstacle_tf]
            future_points,future_covarainces=self.predictors[obstacle_tf].Predict(trans[0],trans[1])
            tfmatrix=self.transformation_manager.get_odom_robot_tf_matrix()
            for i,(p ,c) in enumerate(zip(future_points,future_covarainces)):
                point= np.array( [ p[0], p[1], 0, 1])
                kfx_transformed=np.dot(tfmatrix,point)
                if abs(kfx_transformed[0])<2.5 and abs(kfx_transformed[1])<2.5 :
                    z=gaussian_2d(self.x,self.y,(kfx_transformed+2.5)*10,np.diag(c))/(i+1)+z

        z=((z - np.min(z)) / np.ptp(z) * 100).astype(np.uint8).reshape(-1)
        return z 




    def calculateAndPublish(self):
        dynamic_obstacle_tfs=self.transformation_manager.get_dynamic_obstacle_transformations()

        z=np.zeros([50,50])
        for obstacle_tf in dynamic_obstacle_tfs.keys():
            trans=dynamic_obstacle_tfs[obstacle_tf]
            future_points,future_covarainces=self.predictors[obstacle_tf].Predict(trans[0],trans[1])
            tfmatrix=self.transformation_manager.get_odom_robot_tf_matrix()
            for i,(p ,c) in enumerate(zip(future_points,future_covarainces)):
                point= np.array( [ p[0], p[1], 0, 1])
                kfx_transformed=np.dot(tfmatrix,point)
                if abs(kfx_transformed[0])<2.5 and abs(kfx_transformed[1])<2.5 :
                    z=gaussian_2d(self.x,self.y,(kfx_transformed+2.5)*10,np.diag(c))/(i+1)+z



        costmap_transformation=self.transformation_manager.get_costmap_transformation()
        self.roscostmap .publish(z,costmap_transformation[0]-2.5,costmap_transformation[1]-2.5)
        z=((z - np.min(z)) / np.ptp(z) * 100).astype(np.uint8).reshape(-1)
        return z 






if __name__ == "__main__":
    rospy.init_node("DynamicObstacletrajectorypredection_Node")
    time.sleep(1)

    #identifying all avilable dynamic abstacle frames in thbe environemnt
    frames=get_all_dynamic_obtacles_frames()
    #creating a transformation manger to mange tf calculations for dynamic obstacles 
    transformationManger=DynamicTrajectoryTransformationManger(frames=frames,parentFrame="odom",robotFrame="base_link")
    # creating a predictor for every dynamic obstacle
    predictors={}
    for frame in frames : 
        predictors[frame]=DynamicObstaclePredictor(100)




    x, y = np.meshgrid(np.linspace(0, 50, 50), np.linspace(0, 50, 50))
    pos = np.dstack((x, y))
    roscostmap =TrajectoryCostmap("Customcostmap2",50,50,Xposition=-2.5,Yposition=-2.5)
    listener = tf.TransformListener()
    predictor=DynamicObstaclePredictor(100)





    rospy.sleep(1)
    while not rospy.is_shutdown():
        dynamic_obstacle_tfs=transformationManger.get_dynamic_obstacle_transformations()

        z=np.zeros([50,50])
        for obstacle_tf in dynamic_obstacle_tfs.keys():
            trans=dynamic_obstacle_tfs[obstacle_tf]
            future_points,future_covarainces=predictors[obstacle_tf].Predict(trans[0],trans[1])
            tfmatrix=transformationManger.get_odom_robot_tf_matrix()
            for i,(p ,c) in enumerate(zip(future_points,future_covarainces)):
                point= np.array( [ p[0], p[1], 0, 1])
                kfx_transformed=np.dot(tfmatrix,point)
                if abs(kfx_transformed[0])<2.5 and abs(kfx_transformed[1])<2.5 :
                    z=gaussian_2d(x,y,(kfx_transformed+2.5)*10,np.diag(c))/(i+1)+z
        

        costmap_transformation=transformationManger.get_costmap_transformation()
        roscostmap.publish(z,costmap_transformation[0]-2.5,costmap_transformation[1]-2.5)
        time.sleep(0.02)

