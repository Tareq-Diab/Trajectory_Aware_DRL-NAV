
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetModelStateResponse
from gazebo_msgs.srv import GetWorldProperties
import numpy as np 
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import time
import yaml

class GoalRandomizer:
    def __init__(self,min_range,max_range , obstacle_proximity,filter="unit",forbiddenArea=None,obstacle_yaml=None):
        """_summary_

        Args:
            min_range (_type_): _description_
            max_range (_type_): _description_
            obstacle_proximity (_type_): _description_
            filter (str, optional): _description_. Defaults to "unit".
            forbiddenArea : list of ceneter and raduis
        """
        self.min_range =min_range
        self.max_range =max_range 
        self.obstacle_proximity =obstacle_proximity
        self.filter =filter
        self.goal={"x":0,"y":0}
        if obstacle_yaml : 
            with open(obstacle_yaml, 'r') as yaml_file:
                self.data = yaml.safe_load(yaml_file)
                print(f"goal randomizer intiated with obstacle yaml  {obstacle_yaml} which had {self.data}")
        else :
            self.data=None
        self.forbiddenArea=forbiddenArea

    def get_all_model_positions(self):

        rospy.wait_for_service('/gazebo/get_world_properties')
        rospy.wait_for_service('/gazebo/get_model_state')

        try:
            world_properties_service = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
            world_properties = world_properties_service()
            model_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            model_positions = {}

            for model_name in world_properties.model_names:
                model_state_request = GetModelStateRequest()
                model_state_request.model_name = model_name
                model_state_response = model_state_service(model_state_request)

                if model_state_response.success:
                    model_positions[model_name] = model_state_response.pose.position

            return model_positions

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return {}
    

    def get_dist(self,ooi,goal,threshold=1):
        """recursive function exit condition

        Args:
            ooi (_type_): _description_
            goal (_type_): _description_
            threshold (int, optional): _description_. Defaults to 1.

        Returns:
            bool: true if goal is valid
        """
        for coor in ooi :
            if ((coor[0]-goal["x"])**2+(coor[1]-goal["y"])**2)**0.5 <threshold:
                return False
        #ensure goals are not set near forbiddenArea 
        if self.forbiddenArea:
            if ((self.forbiddenArea[0]-goal["x"])**2+(self.forbiddenArea[1]-goal["y"])**2)**0.5 <self.forbiddenArea[2]:
                return False
        return True
    
    def get_obstacle_data(self):
        return self.data

    def get_obstacle_rectangles(self, obstacle_data):
        obstacle_rectangles = []
        for obs_name, corners in obstacle_data.items():
            x_coords = [coord[0] for coord in corners]
            y_coords = [coord[1] for coord in corners]
            x_min = min(x_coords)
            x_max = max(x_coords)
            y_min = min(y_coords)
            y_max = max(y_coords)
            obstacle_rectangles.append({
                'x_min': x_min,
                'x_max': x_max,
                'y_min': y_min,
                'y_max': y_max
            })
        return obstacle_rectangles

    def is_point_in_obstacles(self, point, obstacle_rectangles):
        if  not np.any (obstacle_rectangles ):
            return False
        x = point["x"]
        y = point["y"]
        for rect in obstacle_rectangles:
            buffer = self.obstacle_proximity
            x_min = rect['x_min'] - buffer
            x_max = rect['x_max'] + buffer
            y_min = rect['y_min'] - buffer
            y_max = rect['y_max'] + buffer
            
            if x_min <= x <= x_max and y_min <= y <= y_max:
                return True  # Point is inside this obstacle or too close
        return False  # Point is safe



    def __getrandom_goal(self,ooi,obstacle_rectangles):
        """recursive function that get a random goal and test it against all static iobstacles in the environemnt

        Args:
            ooi list of all static obstacle positions

        Returns:
           goal dict
        """
        self.goal["x"],self.goal["y"]=np.random.uniform(self.min_range,self.max_range,2)
        if self.get_dist(ooi,self.goal,self.obstacle_proximity) and not self.is_point_in_obstacles(self.goal, obstacle_rectangles):
            return self.goal
        else:
            return self.__getrandom_goal(ooi,obstacle_rectangles)
        
    def getRandomGoal(self):
            positions = self.get_all_model_positions()
            obstacle_data = self.get_obstacle_data()
            if np.any(obstacle_data):
                obstacle_rectangles = self.get_obstacle_rectangles(obstacle_data)
            else :
                obstacle_rectangles=None
            ooi=[]
            for model_name, position in positions.items():
                if model_name[:4] == self.filter:
                    ooi.append([position.x,position.y])
            return self.__getrandom_goal(ooi,obstacle_rectangles)
            
def publish_marker(marker_id, x, y, z, r, g, b):

    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    #marker configurations 
    marker = Marker()
    marker.header.frame_id = "odom"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "basic_shapes"
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.MODIFY

    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.4
    marker.scale.y = 0.4
    marker.scale.z = 0.4

    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration()
    marker_pub.publish(marker)

if __name__ == "__main__":
    rospy.init_node("randomizerTest")
    time.sleep(1)
    while True:
        goalcreator=GoalRandomizer(-9,9,1)
        goal=goalcreator.getRandomGoal()
        print(goal)
        publish_marker(1,goal["x"],goal["y"],0,1,0,0)
        time.sleep(1)
