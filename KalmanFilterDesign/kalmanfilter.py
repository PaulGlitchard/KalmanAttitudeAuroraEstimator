    
import numpy as np
import math
from constants import *
from scipy.linalg import sqrtm

zero_matrix = np.zeros((1, 2))

np.set_printoptions(precision=3)

def quaternion_mult(q1,q2):

  w1, x1, y1, z1 = q1
  w2, x2, y2, z2 = q2

  w = w1*w2 - x1*x2 - y1*y2 - z1*z2
  x = w1*x2 + x1*w2 - y1*z2 + z1*y2
  y = w1*y2 + x1*z2 + y1*w2 - z1*x2
  z = w1*z2 - x1*y2 + y1*x2 + z1*w2

  quaternion_new = np.array([w, x, y, z])
  return quaternion_new

def quaternion_inv(q):
  q[0] = q[0]
  q[1] = -q[1]
  q[1] = -q[2]
  q[2] = -q[3]
  return q

class KalmanFilter:
  def __init__(self):
    self.state_vector = np.ones((STATE_VEC_SIZE,1))
    self.state_vector[0] = 0.3
    self.state_vector[1] = 0.4
    self.state_vector[2] = 0.8
    self.state_vector[3] = 0.6
    self.state_vector[4] = 0.4
    self.state_vector[5] = 0.8
    self.state_vector[6] = 0.6
    self.process_noise_vector = np.zeros((STATE_VEC_SIZE,1))
    self.covariance_matrix = np.zeros((STATE_VEC_SIZE,STATE_VEC_SIZE))
  
    sigma_points = self.compute_sigma_points()
    
    state_vector_set = []
    for i in range(len(sigma_points)):
      state_vector_set.append(self.process_model(sigma_points[i],np.zeros((6,1)), 0))
      print(state_vector_set[i])
    
    state_vector_set_mean = self.mean_of_state_vector(state_vector_set)
    state_vector_set_cov = self.cov_of_state_vector(state_vector_set,state_vector_set_mean)
    print("state_vector_set_mean")
    print(state_vector_set_mean)
    print("state_vector_set_cov")
    print(state_vector_set_cov)
    
    projected_measurement_vectors = []
    for i in range(len(sigma_points)):
      projected_measurement_vectors.append(self.measurement_model(sigma_points[i],np.zeros((3,1)),0))
    
    
  def compute_sigma_points(self):
    sigma_points = []
    Q = np.zeros((STATE_VEC_SIZE,STATE_VEC_SIZE)) # TODO: wat is covariance Q
    W = sqrtm(2*STATE_VEC_SIZE * (self.covariance_matrix + Q))
    for i in range(len(W[:,0])):
      sigma_points.append(self.state_vector + W[:,i].reshape(len(W[:,i]),1))
      sigma_points.append(self.state_vector - W[:,i].reshape(len(W[:,i]),1))
    
    return sigma_points
  
  
  
  def process_model(self,state_vector,process_noise, time_difference):
    # print(len(state_vector))
    # print(len(process_noise))
    process_noise_q = process_noise[0:3]
    process_noise_w = process_noise[3:6]
    
    angle_w = np.linalg.norm(process_noise_q) * time_difference
    axis_w = np.zeros((3,1))
    if (np.linalg.norm(process_noise_q) != 0):
      axis_w  = process_noise_q / np.linalg.norm(process_noise_q)
    # print(axis_w)
    quaternion_w = np.zeros((QUATERNION,1))
    quaternion_w[0] = math.cos(angle_w/2)
    quaternion_w[1:4] = (axis_w * math.sin(angle_w/2))
    # print("quaternion_w")
    # print(quaternion_w)

    disturbed_quaternion =  state_vector[0:4] * quaternion_w
    disturbed_angular_velocity = state_vector[4:7] + process_noise_w
    
    
    angle_d = np.linalg.norm(process_noise[4:7]) * time_difference
    axis_d = np.zeros((3,1))
    if (np.linalg.norm(process_noise[4:7]) != 0):
      axis_d = process_noise[4:7] / np.linalg.norm(process_noise[4:7])
    
    quaternion_delta = np.zeros((QUATERNION,1))
    quaternion_delta[0] = math.cos(angle_d/2)
    quaternion_delta[1:4] = (axis_d * math.sin(angle_d/2))
    # print("quaternion_delta")
    # print(quaternion_delta)
    
    state_vector[0:4] = disturbed_quaternion * quaternion_delta
    state_vector[4:7] = disturbed_angular_velocity
    # print("state_vector")
    # print(state_vector)
    
    return state_vector
  
  def mean_of_state_vector(self,state_vector_list):
    quat_list = []
    vec_list = []
    
    for element in state_vector_list:
      quat_list.append(element[0:4])
      vec_list.append(element[4:7])
    
    # calc mean of anular velocity part
    vec_mean = sum(vec_list)/(2*STATE_VEC_SIZE)
    state_vector_mean = np.zeros((STATE_VEC_SIZE,1))
    state_vector_mean[4:7] = vec_mean
    
    # calc mean of orientation component
    q_mean = quat_list[0]
    error_quat = np.zeros((QUATERNION,1))
    for i in range(2*STATE_VEC_SIZE):
      error_quat = quaternion_mult(quat_list[i],quaternion_inv(q_mean))
      q_mean = quaternion_mult(error_quat,q_mean)
    state_vector_mean[0:4] = q_mean
    print(state_vector_mean)
    return state_vector_mean
    
  
  def cov_of_state_vector(self,state_vector_list, mean):
    priori_state_vec_cov = 0

    for i in range(2*STATE_VEC_SIZE):
      W_i = np.array(state_vector_list[i]-mean)
      priori_state_vec_cov = priori_state_vec_cov + W_i @ W_i.T
    
    priori_state_vec_cov = priori_state_vec_cov/(2*STATE_VEC_SIZE)
    print("TIS IS MAYBE STILL WRON")
    return priori_state_vec_cov
  
  def measurement_model(self,sigma_point,noise,time_diff):
    q = sigma_point[0:4]
    w = sigma_point[4:7]
    
    z_rot = w + noise # TODO: noise of rot
    
    g_vector = TODO = np.array([1, 2, 3])
    g = np.array([0, g_vector[0], g_vector[1], g_vector[2]])
    g_ = quaternion_mult(quaternion_mult(q,g),quaternion_inv(q))
    
    b_vector = TODO = np.array([1, 2, 3])
    b = np.array([0, b_vector[0], b_vector[1], b_vector[2]])
    b_ = quaternion_mult(quaternion_mult(q,b),quaternion_inv(q))
    
    z_acc = g_[1:4] + noise # TODO: noise of acc 
    z_mag = b_[1:4] + noise # TODO: noise of mag

    print(np.concatenate((z_rot,z_acc,z_mag)))
    return np.concatenate((z_rot,z_acc,z_mag))
    
    
  def execute_kalman_filter(self):
    print("Executing Kalman Filter...")

