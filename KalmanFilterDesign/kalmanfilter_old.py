    
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
    
    # process noise
    values = [0,0,0,0]
    self.process_noise_vector = np.array(values).reshape(len(values),1)
    
    # measurement noise
    values = [0,0,0] # how many measurements -> sun sensors, gyro, magnetometer
    self.measurement_noise_vector = np.array(values).reshape(len(values),1)
    
    # state vector
    values = [0,0,0,0]
    self.state_vector = np.array(values).reshape(SYSTEM_DIM,1)
    values = [*self.state_vector,*self.process_noise_vector,*self.measurement_noise_vector]
    self.state_vector_aug = np.array(values).reshape(len(values),1)
    print("state vector aug:")
    print(self.state_vector_aug)
    # self.state_vector_aug_mean = sum(self.state_vector_aug) / len(self.state_vector_aug)
    # print(f'x_aug_mean {self.state_vector_aug_mean} = {sum(self.state_vector_aug)} / {len(self.state_vector_aug)}')
    
    # previous x
    self.state_vector_prev = self.state_vector * 0
    self.state_vector_aug_prev = self.state_vector_aug * 0

    # covariance matrix
    self.covariance_matrix = (self.state_vector_prev - self.state_vector) @ (self.state_vector_prev - self.state_vector).T
    print(self.covariance_matrix)
    self.covariance_matrix_aug = (self.state_vector_aug_prev - self.state_vector_aug) @ (self.state_vector_aug_prev - self.state_vector_aug).T
    print(self.covariance_matrix_aug)
    

    self.sigma_points = self.compute_sigma_points()
    for i in range(len(self.sigma_points)):
      print("---")
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
    sigma_points.append(self.state_vector_aug)  
    point_matrix = (sqrtm((SYSTEM_DIM + self.scaling_factor) * self.covariance_matrix_aug))
    print(point_matrix)
    for i in range(SYSTEM_DIM):
      sigma_points.append(self.state_vector_aug + point_matrix[:,i].reshape(len(point_matrix[:,i]),1))
      sigma_points.append(self.state_vector_aug - point_matrix[:,i].reshape(len(point_matrix[:,i]),1))
      
    return sigma_points
  
  def compute_weights(self,weights):
    for i in range(2*SYSTEM_DIM):
      weights.append(1 / ( 2 * (SYSTEM_DIM + self.scaling_factor)))
    return weights
  
  
  def compute_weights_m(self):
    print("computing weights m...")
    weights = []
    weights.append(self.scaling_factor / (SYSTEM_DIM + self.scaling_factor))
    print(self.scaling_factor)
    return self.compute_weights(weights)
  
  
  def compute_weights_c(self):
    print("computing weights c...")
    weights = []
    weights.append((self.scaling_factor / (SYSTEM_DIM + self.scaling_factor)) + (1 - self.alpha**2 + self.beta))
    return self.compute_weights(weights)
  
  
  def time_update(self):
    print("time update...")
    pass
    
  
  def prediction(self):
    print("prediction...")
    pass

  def execute_kalman_filter(self):
    print("Executing Kalman Filter...")

