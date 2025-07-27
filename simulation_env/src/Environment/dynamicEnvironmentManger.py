import numpy as np
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetModelStateResponse
from gazebo_msgs.srv import GetWorldProperties
import numpy as np 
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import time
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import quaternion_from_euler
import rospy
from Environment.goal_randomizer import GoalRandomizer ,publish_marker
from Environment.dynamicEnvironmentUtils import *
from copy import copy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
import matplotlib.pyplot as plt 
import yaml
class HumanoidActorsManger():
    def __init__(self,actor_number=10,goals_number=5,max_vel=0.7,forbiddenArea=None,pathsyaml=None):
        self.actors_number=actor_number
        self.goals_number=goals_number
        self.actor_publishers=[rospy.Publisher(f'/cmd_path_actor_{i}', Path, queue_size=1, latch=True) for i in range(self.actors_number)]
        self.actor_vel_publishers=[rospy.Publisher(f'/cmd_vel_actor_{i}', Twist, queue_size=1, latch=True) for i in range(self.actors_number)]
        self.forbiddenArea=forbiddenArea
        time.sleep(2)
        self.goalrandomizer=GoalRandomizer(-9,9,1,forbiddenArea==self.forbiddenArea)
        self.modelStatePublisher = rospy.Publisher("/gazebo/set_model_state",ModelState , queue_size=1000)
        self.modelStateMessage=ModelState()
        self.pathsyaml=pathsyaml
        if self.pathsyaml:
            self.create_pathes_from_yaml()
    def publish_path_points(self,pub,points):
        # Create the Path message
        path_msg = Path()
        # Set the header for the Path message
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "odom"
        for p in points:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "odom"
            x,y=grid_to_world(p[0],p[1])
            pose.pose.position = Point(x=x, y=y, z=0)
            path_msg.poses.append(pose)
        pub.publish(path_msg)



    def get_all_model_positions(self):

        rospy.wait_for_service('/gazebo/get_world_properties')
        rospy.wait_for_service('/gazebo/get_model_state')
        try : 
            # Create a service proxy for getting world properties
            world_properties_service = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
            world_properties = world_properties_service()
    
            # Create a service proxy for getting model state
            model_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            model_positions = {}
            poses=[]
            # Loop through all the model names in the world
            for model_name in world_properties.model_names:
                if model_name[:14] == "humanoid_actor":
                    continue
                # Get the state of each model
                model_state_request = GetModelStateRequest()
                model_state_request.model_name = model_name
                model_state_response = model_state_service(model_state_request)
    
                if model_state_response.success:
                    # Store the position of the model
                    poses.append( [model_state_response.pose.position.x,model_state_response.pose.position.y])
            return poses
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return []
    def spawn_actor_to(self,obstacleName,centerListXY):

        self.modelStateMessage.model_name=obstacleName
        self.modelStateMessage.pose.position.x=centerListXY[0]
        self.modelStateMessage.pose.position.y=centerListXY[1]
        self.modelStateMessage.pose.position.z=1
        self.modelStateMessage.pose.orientation.x = 0
        self.modelStateMessage.pose.orientation.y = 0
        self.modelStateMessage.pose.orientation.z = 1
        self.modelStateMessage.pose.orientation.w = 0
        self.modelStatePublisher.publish(self.modelStateMessage)

    def create_randomized_pathes(self):
        if self.pathsyaml :
            return None
        obstacle_poses=self.get_all_model_positions()
        obstacle_grid = create_obstacle_grid(obstacle_poses)

        #ensure paths arent created near the robot safe place 
        if self.forbiddenArea:
            coordinates=world_to_grid(self.forbiddenArea[0],self.forbiddenArea[1])
            print(coordinates)
            dist_in_cells=self.forbiddenArea[2]*10
            print(dist_in_cells)
            obstacle_grid[int(coordinates[1]-dist_in_cells):int(coordinates[1]+dist_in_cells) , int(coordinates[0]-dist_in_cells):int(coordinates[0]+dist_in_cells)]=1

        path_points={}
        for actor in range(self.actors_number):
            goals=[]
            path_points[actor]=[]
            for i in range (self.goals_number):
                p=copy(self.goalrandomizer.getRandomGoal())
                # print(p)
                goals.append(p)
            # print(goals)
            for i in range(self.goals_number) :
                if i != self.goals_number -1:

                    start = world_to_grid(goals[i]["x"]  ,goals[i]["y"])
                    end   = world_to_grid(goals[i+1]["x"],goals[i+1]["y"])
                else :

                    start = world_to_grid(goals[i]["x"]  ,goals[i]["y"])
                    end   = world_to_grid(goals[0]["x"],goals[0]["y"])
                path = a_star(obstacle_grid, start, end)
                if path :
                    path_points[actor].extend(path)
            # print(f"actor {actor} : {path_points[actor]}")

            self.publish_path_points(self.actor_publishers[actor],path_points[actor])
            try :
                self.spawn_actor_to(f"humanoid_actor_{actor}",grid_to_world(path_points[actor][0][0],path_points[actor][0][1])) #move actor to first point in path
                print(f"actor soawned to {actor}")
            except:
                self.spawn_actor_to(f"humanoid_actor_{actor}",(10,10))
                print(f"spawning actor {actor} to 10,10 as error in its path occured")

            sampled_vel = np.random.uniform(0.1, 0.7)
            twist = Twist()
            # Set the linear velocity in x direction (1 m/s)
            twist.linear.x = sampled_vel
            # Increase angular velocity in z to create a tighter turn
            twist.angular.z = 2.0  
            self.actor_vel_publishers[actor].publish(twist)


    def create_pathes_from_yaml(self):
        obstacle_poses=self.get_all_model_positions()
        obstacle_grid = create_obstacle_grid(obstacle_poses)
        with open(self.pathsyaml, 'r') as yaml_file:

            data = yaml.safe_load(yaml_file)

        #ensure paths arent created near the robot safe place 
        path_points={}
        for actor in range(self.actors_number):
            goals=data[f"actor{actor}"]
            path_points[actor]=[]

            for i in range(len(goals)-1) :
                if i != self.goals_number -1:

                    start = world_to_grid(goals[i][0]  ,goals[i][1])
                    end   = world_to_grid(goals[i+1][0],goals[i+1][1])
                else :

                    start = world_to_grid(goals[i][0]  ,goals[i][1])
                    end   = world_to_grid(goals[0][0],goals[0][1])
                path = a_star(obstacle_grid, start, end)
                if path :
                    path_points[actor].extend(path)


            self.publish_path_points(self.actor_publishers[actor],path_points[actor])
            try :
                self.spawn_actor_to(f"humanoid_actor_{actor}",grid_to_world(path_points[actor][0][0],path_points[actor][0][1])) #move actor to first point in path
                print(f"actor soawned to {actor}")
            except:
                self.spawn_actor_to(f"humanoid_actor_{actor}",(10,10))
                print(f"spawning actor {actor} to 10,10 as error in its path occured")


            twist = Twist()
            # Set the linear velocity in x direction (1 m/s)
            twist.linear.x = 1
            # Increase angular velocity in z to create a tighter turn
            twist.angular.z = 2.0  
            self.actor_vel_publishers[actor].publish(twist)

