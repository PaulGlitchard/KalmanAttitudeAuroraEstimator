    
import numpy as np
import math
from constants import *
from scipy.linalg import sqrtm

zero_matrix = np.zeros((1, 2))

def quaternion_mult(q1,q2):

  w1, x1, y1, z1 = q1
  w2, x2, y2, z2 = q2

  w = w1*w2 - x1*x2 - y1*y2 - z1*z2
  x = w1*x2 + x1*w2 - y1*z2 + z1*y2
  y = w1*y2 + x1*z2 + y1*w2 - z1*x2
  z = w1*z2 - x1*y2 + y1*x2 + z1*w2

  quaternion_new = np.array([w, x, y, z])
  return quaternion_new

class KalmanFilter:
  def __init__(self):
    self.state_vector = np.zeros((STATE_VEC_SIZE,1))
    self.process_noise_matrix = np.zeros((STATE_VEC_SIZE,STATE_VEC_SIZE))
    self.covariance_matrix = np.zeros((STATE_VEC_SIZE,STATE_VEC_SIZE))
  
    sigma_points = self.compute_sigma_points()
    self.new_state_vector = self.process_model(sigma_points, 0)
    
  def compute_sigma_points(self):
    sigma_points = []
    Q = np.zeros((STATE_VEC_SIZE,STATE_VEC_SIZE)) # TODO: wat is covariance Q
    W = sqrtm(2*STATE_VEC_SIZE * (self.covariance_matrix + Q))
    for i in range(len(W[:,0])):
      sigma_points.append(self.state_vector + W[:,i].reshape(len(W[:,i]),1))
      sigma_points.append(self.state_vector - W[:,i].reshape(len(W[:,i]),1))
      
    for i in range(len(sigma_points)):
      print(sigma_points[i])
    
    print(len(sigma_points))
    return sigma_points
  
  def process_model(state_vector,process_noise, time_diff):
    angle = np.linalg.norm(process_noise) * time_diff
    axis  = process_noise / np.linalg.norm(process_noise)
    quaternion_delta = [
      [math.cos(angle/2)],
      [(axis * math.sin(angle/2))[0]],
      [(axis * math.sin(angle/2))[1]],
      [(axis * math.sin(angle/2))[2]],
      [(axis * math.sin(angle/2))[3]]
    ]
    new_quat = process_noise * quaternion_delta
    new_state_vector = 0
    return new_state_vector
    
    
    
    
  def execute_kalman_filter(self):
    print("Executing Kalman Filter...")

