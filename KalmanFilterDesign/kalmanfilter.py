    
import numpy as np

zero_matrix = np.zeros((1, 2))



def initialize():
    x = np.matrix([[0],
                   [0]]) # rotation speed
    
    mean_vector = np.matrix([[1],
                             [1]])
    
    P = (x - mean_vector) @ (x - mean_vector).T
    
    x_mean_augmented = np.block([[mean_vector.T], [zero_matrix], [zero_matrix]]).T
    x_augmented = np.block([[x.T], [zero_matrix], [zero_matrix]]).T
    P_augmented = (x_augmented - x_mean_augmented) @ (x_augmented - x_mean_augmented).T
    
    print(P_augmented)
    
    
def choose_sigma_points():
    sigma_points = np.matrix()

def prediction():
    pass

def execute_kalman_filter():
    print("Executing Kalman Filter...")