

from filterpy.kalman import KalmanFilter
import numpy as np
import copy 


class DynamicObstaclePredictor:
    def __init__(self,n_future_steps=50) :
        self.n_future_steps=n_future_steps
        # Initialize the Kalman filter
        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        # Define the state transition matrix
        self.kf.F = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]])
        # Define the measurement function
        self.kf.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        # Define the state covariance matrix
        self.kf.P = np.eye(4) * 1000
        # Define the measurement noise matrix
        self.kf.R = np.array([[5, 0], [0, 5]])
        # Define the process noise matrix
        self.kf.Q = np.eye(4) * 0.01
        # Initialize the state estimate
        self.kf.x = np.array([0, 0, 0, 0])
        self.__copy()
    def __copy(self):
        self.kf_before=copy.deepcopy(self.kf)

    def __restor(self):
        self.kf =self.kf_before

    def Predict(self,x,y):
        """_summary_

        Args:
            x (float): currun x of the obstacle
            y (float):  currun y of the obstacle

        Returns:
            future_points (list): future predicted points  
            future_covarainces(list): future covariances for the predicted future points kf.P[:2,:2] 
        """
        self.kf.predict()
        self.kf.update([ x,y ])
        self.__copy()
        future_points=[]
        future_covarainces=[]
        for i in range(self.n_future_steps):
            self.kf.predict() #predicting a step into future
            self.kf.update(self.kf.x[:2])#updating a step into future
            future_points.append(self.kf.x[:2])
            future_covarainces.append(self.kf.P[:2,:2])
        self.__restor() #getting the old kf back after future prediction changed its internal state
        return future_points ,future_covarainces