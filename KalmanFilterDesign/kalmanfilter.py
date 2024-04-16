    
import numpy as np

zero_matrix = np.zeros((1, 2))

class KalmanFilter:
  def __init__(self):
    self.x = np.matrix([[0],
                        [0]]) # rotation speed
    
    self.x_mean_vector = np.matrix([[1],
                                  [1]])
    
    self.P = (self.x - self.x_mean_vector) @ (self.x - self.x_mean_vector).T
    
    self.x_mean_augmented = np.block([[self.x_mean_vector.T], [zero_matrix], [zero_matrix]]).T
    print("x_mean_augmented")
    print(self.x_mean_augmented)
    self.x_augmented = np.block([[self.x.T], [zero_matrix], [zero_matrix]]).T
    print("x_augmented")
    print(self.x_augmented)
    self.P_augmented = (self.x_augmented - self.x_mean_augmented).T @ (self.x_augmented - self.x_mean_augmented) # this has to be the other way around!
    print("P_augmented")
    print(self.P_augmented)
    print(self.x_augmented - self.x_mean_augmented)
    
    
  def choose_sigma_points(self):
    self.x_mean_augmented

  def prediction(self):
    pass

  def execute_kalman_filter(self):
    print("Executing Kalman Filter...")