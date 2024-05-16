    
import numpy as np
import math
from constants import *
from scipy.linalg import sqrtm

zero_matrix = np.zeros((1, 2))

class KalmanFilter:
  def __init__(self):
    self.alpha = 10**(-3) # https://liu.diva-portal.org/smash/get/diva2:1641373/FULLTEXT01.pdf
    self.beta = 2
    self.kappa = 0
    
    self.scaling_factor = (self.alpha ** 2) * (SYSTEM_DIM + self.kappa) - SYSTEM_DIM
    print(f'scaling_factor {self.scaling_factor}')
    
    # state vector
    self.x = np.zeros((SYSTEM_DIM,1))

    # covariance matrix
    self.covariance_matrix = np.zeros((SYSTEM_DIM,SYSTEM_DIM))

    # mean
    self.x_mean = 0
    
    # self.x_mean = np.zeros((SYSTEM_DIM,1))
    # print("x\n"+str(self.x))
    # print("x_mean\n"+str(self.x_mean))
    
    # #covariance matrix
    # self.P = (self.x - self.x_mean) @ (self.x - self.x_mean).T
    # print("P\n"+str(self.P))
    
    # #process noise
    # self.w = np.zeros((7,1))
    
    # #measurement noise (d = measurements amount)
    # # 1x Sun Sensor
    # # 1x Gyro
    # # 1x Magnetometer
    # self.v = np.zeros((12,1))

    # print(self.v)

    # self.x_aug = np.vstack((self.x, self.w, self.v))
    # print(self.x_aug)

    self.sigma_points = self.compute_sigma_points()
    for i in range(len(self.sigma_points)):
      print(self.sigma_points[i])
      
    self.weights_m = self.compute_weights_m()
    print(f'sum weights_m: {sum(self.weights_m)}')
    self.weights_c = self.compute_weights_c()
    print(f'sum weights_c: {sum(self.weights_c)}')
    # print("sigma points:")
    # print(self.sigma_points)
    
    
    
  def compute_sigma_points(self):
    print("computing sigma points...")
        
    sigma_points = []
    sigma_points.append(self.x_mean)  
    point_matrix = (sqrtm((SYSTEM_DIM + self.scaling_factor) * self.covariance_matrix))
    print(point_matrix)
    for i in range(SYSTEM_DIM):
      sigma_points.append(self.x_mean + point_matrix[i])
      sigma_points.append(self.x_mean - point_matrix[i])
      
    return sigma_points
  
  def compute_weights(self,weights):
    for i in range(2*SYSTEM_DIM):
      weights.append(1 / ( 2 * (SYSTEM_DIM + self.scaling_factor)))
    return weights
  
  
  def compute_weights_m(self):
    print("computingg weights m...")
    weights = []
    weights.append(self.scaling_factor / (SYSTEM_DIM + self.scaling_factor))
    print(self.scaling_factor)
    return self.compute_weights(weights)
  
  
  def compute_weights_c(self):
    print("computingg weights c...")
    weights = []
    weights.append((self.scaling_factor / (SYSTEM_DIM + self.scaling_factor)) + (1 - self.alpha**2 + self.beta))
    return self.compute_weights(weights)
  
  
  def prediction(self):
    pass

  def execute_kalman_filter(self):
    print("Executing Kalman Filter...")

