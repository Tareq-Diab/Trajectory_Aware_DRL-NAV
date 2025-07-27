#!/usr/bin/env python

import rospy
import random
import numpy as np
from nav_msgs.msg import OccupancyGrid


class TrajectoryCostmap():
    """_summary_
    """
    def __init__(self,topic:str,width:int,hight:int,resolution:float=0.1,Xposition:float=0,Yposition:float=0) -> None:
        """_summary_

        Args:
            topic (str): ros topic for the custom map
            width (int):                   Change to the desired width of the costmap.
            hight (int):                   Change to the desired height of the costmap.
            resolution (float, optional):  Change to the desired resolution of the costmap. Defaults to 0.1.
            Xposition (float, optional):   Change to the desired origin of the costmap. Defaults to 0.
            Yposition (float, optional):   Change to the desired origin of the costmap. Defaults to 0.

        """
        self.pub                            = rospy.Publisher(topic, OccupancyGrid, queue_size=1)
        # Create a new random costmap
        self.costmap                        = OccupancyGrid()
        self.costmap.header.frame_id        = 'odom'
        self.costmap.info.width             = width      
        self.costmap.info.height            = hight      
        self.costmap.info.resolution        = resolution 
        self.costmap.info.origin.position.x = Xposition  
        self.costmap.info.origin.position.y = Yposition  
    
    def publish(self,map,ox,oy)->None:
        map = ((map - np.min(map)) / np.ptp(map) * 100).astype(np.uint8).reshape(-1)
        self.costmap.info.origin.position.x = ox  
        self.costmap.info.origin.position.y = oy      
        self.costmap.data=map.reshape(-1)
        self.costmap.header.stamp= rospy.Time.now()
        self.pub.publish(self.costmap)