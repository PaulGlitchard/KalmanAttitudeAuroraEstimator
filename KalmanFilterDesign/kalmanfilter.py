    
import numpy as np
import math
from constants import *
from scipy.linalg import sqrtm

zero_matrix = np.zeros((1, 2))

class KalmanFilter:
  def __init__(self):
    self.state_vector = np.zeros((STATE_VEC_SIZE,1))
    self.process_noise_matrix = np.zeros((STATE_VEC_SIZE,STATE_VEC_SIZE))
    self.covariance_matrix = np.zeros((STATE_VEC_SIZE,STATE_VEC_SIZE))
    
    self.compute_sigma_points()
    
  def compute_sigma_points(self):
    sigma_points = []
    Q = np.zeros((STATE_VEC_SIZE,STATE_VEC_SIZE)) # TODO: wat tis
    W = sqrtm(2*STATE_VEC_SIZE * (self.covariance_matrix + Q))
    for i in range(len(W[:,0])):
      sigma_points.append(+ W[:,i].reshape(len(W[:,i]),1))
      sigma_points.append(- W[:,i].reshape(len(W[:,i]),1))
      
    for i in range(len(sigma_points)):
      print(sigma_points[i])
    
    print(len(sigma_points))
    
    
    
    
  def execute_kalman_filter(self):
    print("Executing Kalman Filter...")

