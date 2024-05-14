    
import numpy as np

zero_matrix = np.zeros((1, 2))

class KalmanFilter:
  def __init__(self):
    # state vector
    self.x = np.matrix([[0.],  #q0  attitude
                        [0.],  #q1  attitude
                        [0.],  #q2  attitude
                        [0.],  #q3  attitude
                        [0.],  #wx  speed
                        [0.],  #wy  speed
                        [0.]]) #wz  speed
    
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

    self.sigma_points = self.choose_sigma_points()
    # print("sigma points:")
    # print(self.sigma_points)
    
    
    
  def choose_sigma_points(self):
    sigma_points = np.zeros((7,1))
    sigma_points[0] = self.x_aug
    for i in range(1,len(sigma_points)):
      sigma_points[i] = self.x_aug 
    return sigma_points
  
  def prediction(self):
    pass

  def execute_kalman_filter(self):
    print("Executing Kalman Filter...")

