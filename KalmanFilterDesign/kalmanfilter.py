    
import numpy as np
import math
from constants import *

zero_matrix = np.zeros((1, 2))

class KalmanFilter:
  def __init__(self):
    # state vector
    self.x = np.zeros((DIM_OF_SYSTEM,1))

    # TODO: update initialisation
    # mean
    self.x_mean = np.zeros((7,1))
    print("x\n"+str(self.x))
    print("x_mean\n"+str(self.x_mean))
    
    #covariance matrix
    self.P = (self.x - self.x_mean) @ (self.x - self.x_mean).T
    print("P\n"+str(self.P))
    
    #process noise
    self.w = np.zeros((7,1))
    
    #measurement noise (d = measurements amount)
    # 1x Sun Sensor
    # 1x Gyro
    # 1x Magnetometer
    self.v = np.zeros((12,1))

    print(self.v)

    self.x_aug = np.vstack((self.x, self.w, self.v))
    print(self.x_aug)

    self.sigma_points = self.compute_sigma_points()
    # print("sigma points:")
    # print(self.sigma_points)
    
    
    
  # TODO: we need 2n + 1 sigma points -> n = 4 becaue state vector has dimension 4
  def compute_sigma_points(self):
    sigma_points = np.zeros((9,1))
    sigma_points[0] = 0 #mean of the gaussian
    for i in range(1,9):
      point = 0
      if i % 2:
        point = 0 + (math.sqrt((DIM_OF_SYSTEM + 1) * 2)) #0 = mean of the gaussian; 1 = scaling factor; 2 = covariance
        
      else:
        point = 0 - (math.sqrt((DIM_OF_SYSTEM + 1) * 2)) #0 = mean of the gaussian; 1 = scaling factor; 2 = covariance
        
      sigma_points[i] = point
    return sigma_points
  
  def prediction(self):
    pass

  def execute_kalman_filter(self):
    print("Executing Kalman Filter...")

