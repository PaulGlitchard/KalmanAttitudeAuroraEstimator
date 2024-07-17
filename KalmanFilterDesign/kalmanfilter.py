
import numpy as np
np.set_printoptions(threshold=np.inf, linewidth=np.inf, precision=6)
PRINT_ENABLE = 0

STATE_DIM = 7 # (orientation and angular velocity)
MEASUREMENT_DIM = 3
QUATERNION = 4

CUR_MEASUREMENT_MODEL = 1
# def getMeasurementModel():
#   global CUR_MEASUREMENT_MODEL
#   CUR_MEASUREMENT_MODEL += 1
#   if CUR_MEASUREMENT_MODEL > 3:
#     CUR_MEASUREMENT_MODEL = 1
#   return CUR_MEASUREMENT_MODEL

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
    w, x, y, z = q
    return np.array([w, -x, -y, -z])

def generate_zero_sigma_points(error_cov, process_noise_cov):
  combined_cov = error_cov + process_noise_cov
  # TODO: validate tis
  epsilon = 1e-6
  combined_cov += np.eye(combined_cov.shape[0]) * epsilon
  
  sqrt_cov = np.linalg.cholesky(combined_cov) * np.sqrt(2 * STATE_DIM)

  zero_sigma_points = []
  for i in range(STATE_DIM):
    zero_sigma_points.append(np.array(+sqrt_cov[:,i]).reshape(-1, 1))
    zero_sigma_points.append(np.array(-sqrt_cov[:,i]).reshape(-1, 1))

  return zero_sigma_points

def generate_sigma_points(state_vector, zero_sigma_points):
  sigma_points = []
  for i in range(2 * STATE_DIM):
    sigma_points.append(state_vector + zero_sigma_points[i])
  return sigma_points

def process_model(state_vector, process_noise, time_difference):
  q = state_vector[:4]      # Quaternion
  q_noise = process_noise[:4] 

  omega = state_vector[4:]  # angular velocity
  omega_noise = process_noise[4:]

  angle_delta = np.linalg.norm(omega) * time_difference
  if angle_delta > 0 and np.linalg.norm(omega) != 0:
    axis_delta = omega / np.linalg.norm(omega)
    vectorial_part = np.sin(angle_delta / 2) * axis_delta
    q_delta = np.array([0,0,0,0]).reshape(-1, 1)
    q_delta[0] =  np.cos(angle_delta / 2)
    q_delta[1] =  vectorial_part[0]
    q_delta[2] =  vectorial_part[1]
    q_delta[3] =  vectorial_part[2]
  else:
    q_delta = np.array([1,0,0,0]).reshape(-1, 1)

  # q_new = quaternion_mult(q,q_delta)

  angle_noise = np.linalg.norm(q_noise) * time_difference
  if angle_noise > 0 and np.linalg.norm(q_noise) != 0:
    axis_noise = q_noise / np.linalg.norm(q_noise)
    vectorial_part = np.sin(angle_noise / 2) * axis_noise
    q_dnoise[0] =  np.cos(angle_noise / 2)
    q_dnoise[1] =  vectorial_part[0]
    q_dnoise[2] =  vectorial_part[1]
    q_dnoise[3] =  vectorial_part[2]
  else:
    q_dnoise = np.array([1,0,0,0]).reshape(-1, 1)

  # q_disturbed = quaternion_mult(q_new,)
  quaternion_part   = quaternion_mult(quaternion_mult(q,q_dnoise),q_delta)
  angular_velo_part = omega + omega_noise
  transformed_sigma_point = np.array([*quaternion_part,*angular_velo_part])

  return transformed_sigma_point

def mean_of_transformed_sigma_points(state_vector_list):
  quat_list = []
  vec_list = []
  
  for element in state_vector_list:
    quat_list.append(element[:4])
    vec_list.append(element[4:])

  # calc mean of anular velocity part
  vec_mean = sum(vec_list)/(2*STATE_DIM)
  state_vector_mean = np.zeros((STATE_DIM,1))
  state_vector_mean[4:] = vec_mean
  
  # calc mean of orientation component
  q_mean = quat_list[0]
  for i in range(2*STATE_DIM):
    error_quat = quaternion_mult(quat_list[i],quaternion_inv(q_mean))
    error_vec = error_quat[1:]
  
  for _ in range(10):
    error_quat = np.zeros((QUATERNION,1))
    q_mean_list = [np.zeros(QUATERNION).reshape(-1, 1) for _ in range(2*STATE_DIM)]
    for i in range(2*STATE_DIM):
      error_quat = quaternion_mult(quat_list[i],quaternion_inv(q_mean))
      q_mean_list[i] = quaternion_mult(error_quat,q_mean)
    barycentic_mean = sum(q_mean_list) / (2*STATE_DIM)
    q_mean = barycentic_mean * q_mean

  state_vector_mean[:4] = q_mean
  return state_vector_mean

def compute_adj_sigma_points(transformed_sigma_points, priori_state_estimate):
  adj_sigma_points = []
  for i in range(2*STATE_DIM):
    adj_sigma_points.append(transformed_sigma_points[i] - priori_state_estimate)

  return adj_sigma_points

def compute_priori_error_cov(adjusted_sigma_points):
  priori_error_cov = np.zeros((STATE_DIM, STATE_DIM))
  for i in range(2*STATE_DIM):
    priori_error_cov += adjusted_sigma_points[i] @ adjusted_sigma_points[i].T
  priori_error_cov = priori_error_cov / (2*STATE_DIM)
  return priori_error_cov

def H1(sigma_point,measurement_noise):
  if measurement_noise != 0:
    exit(-1)
    
  return sigma_point[4:]

def H2(sigma_point,measurement_noise):
  if measurement_noise != 0:
    exit(-1)
    
  q = sigma_point[:4]
  gravity_vector = np.array([0, 0, -9.81])
  gravity_quat = np.array([0, gravity_vector[0], gravity_vector[1], gravity_vector[2]])
  g_prime = quaternion_mult(quaternion_mult(q, gravity_quat), quaternion_inv(q))
  g_prime = g_prime[1:]  # Extract the vector part from the quaternion

  return g_prime

def H3(sigma_point,measurement_noise):
  if measurement_noise != 0:
    exit(-1)
    
  q = sigma_point[:4]
  magnetic_field_vector = np.array([1, 0, 0])
  gravity_quat = np.array([0, magnetic_field_vector[0], magnetic_field_vector[1], magnetic_field_vector[2]])
  b_prime = quaternion_mult(quaternion_mult(q, gravity_quat), quaternion_inv(q))
  b_prime = b_prime[1:]
  
  return b_prime

def compute_measurement_sigma_points(sigma_points,measurement_noise):
  model = CUR_MEASUREMENT_MODEL
  measurement_sigma_points = []
  for i in range(2*STATE_DIM):
    if model == 1:
      measurement_sigma_points.append(H1(sigma_points[i],measurement_noise))
    elif model == 2:
      measurement_sigma_points.append(H2(sigma_points[i],measurement_noise))
    elif model == 3:
      measurement_sigma_points.append(H3(sigma_points[i],measurement_noise))
  return measurement_sigma_points

def mean_of_measurement_sigma_points(measurement_sigma_points):
  mean = 0
  for i in range(2*STATE_DIM):
    mean += measurement_sigma_points[i]
    
  return mean / (2*STATE_DIM)

def compute_innovation(predicted_measurement_estimate,measurement):
  return measurement - predicted_measurement_estimate

def compute_pred_measurement_cov(predicted_measurement_estimate,measurement_sigma_points):
  mean = 0
  for i in range(2*STATE_DIM):
    mean += measurement_sigma_points[i] - predicted_measurement_estimate
    
  return mean / (2*STATE_DIM)

def compute_innovation_cov(pred_measurement_cov,measurement_noise_cov):
  return pred_measurement_cov + measurement_noise_cov

def compute_cross_correlation(adjusted_sigma_points, measurement_sigma_points,predicted_measurement_estimate):
  mean = 0
  for i in range(2*STATE_DIM):
    mean = adjusted_sigma_points[i] @ (measurement_sigma_points[i]-predicted_measurement_estimate).T
    
  return mean / (2*STATE_DIM)

def compute_kalman_gain(cross_correlation,innovation_cov):


  return cross_correlation @ np.linalg.inv(innovation_cov)

def compute_state_estimate(priori_state_estimate,kalman_gain,innovation):
  return priori_state_estimate + kalman_gain @ innovation

def compute_error_cov(priori_error_cov,kalman_gain,innovation_cov):
  return priori_error_cov - kalman_gain @ innovation_cov @ np.array(kalman_gain).T

class KalmanFilter:
  def __init__(self):
    # State variables
    self.state_vector = np.zeros(STATE_DIM).reshape(-1, 1)

    # Covariance Matrices
    self.error_cov = np.eye(STATE_DIM)
    self.priori_error_cov = np.eye(STATE_DIM)
    self.process_noise_cov = np.eye(STATE_DIM) * 0.01
    self.measurement_noise_cov = np.eye(MEASUREMENT_DIM) * 0.01
    self.pred_measurement_cov = np.zeros((MEASUREMENT_DIM, MEASUREMENT_DIM))
    self.innovation_cov = np.zeros((MEASUREMENT_DIM, MEASUREMENT_DIM))
    self.cross_correlation = np.zeros((STATE_DIM, MEASUREMENT_DIM))

    # Sigma Points
    self.sigma_points = [np.zeros(STATE_DIM).reshape(-1, 1) for _ in range(2 * STATE_DIM)]
    self.zero_sigma_points = [np.zeros(STATE_DIM).reshape(-1, 1) for _ in range(2 * STATE_DIM)]
    self.transformed_sigma_points = [np.zeros(STATE_DIM).reshape(-1, 1) for _ in range(2 * STATE_DIM)]
    self.measurement_sigma_points = [np.zeros(MEASUREMENT_DIM).reshape(-1, 1) for _ in range(2 * STATE_DIM)]
    self.adjusted_sigma_points = [np.zeros(STATE_DIM).reshape(-1, 1) for _ in range(2 * STATE_DIM)]

    # Mean Values
    self.state_estimate = np.zeros(STATE_DIM).reshape(-1, 1)
    self.priori_state_estimate = np.zeros(STATE_DIM).reshape(-1, 1)
    self.predicted_measurement_estimate = np.zeros(MEASUREMENT_DIM).reshape(-1, 1)

    # Measurement and Innovation
    # self.measurement = np.zeros(MEASUREMENT_DIM).reshape(-1, 1)
    self.innovation = np.zeros(MEASUREMENT_DIM).reshape(-1, 1)

    # Kalman Gain
    self.kalman_gain = np.zeros((STATE_DIM, MEASUREMENT_DIM))

    # Time
    self.last_time = 0

  def initialization(self):
    # state variables
    self.state_vector = np.zeros(STATE_DIM).reshape(-1, 1)

    # Covariance Matrices
    self.error_cov = np.eye(STATE_DIM)
    self.priori_error_cov = np.eye(STATE_DIM)
    self.process_noise_cov = np.eye(STATE_DIM) * 0.01
    self.measurement_noise_cov = np.eye(MEASUREMENT_DIM) * 0.01
    self.pred_measurement_cov = np.zeros((MEASUREMENT_DIM, MEASUREMENT_DIM))
    self.innovation_cov = np.zeros((MEASUREMENT_DIM, MEASUREMENT_DIM))
    self.cross_correlation = np.zeros((STATE_DIM, MEASUREMENT_DIM))

    # Sigma Points
    self.sigma_points = [np.zeros(STATE_DIM).reshape(-1, 1) for _ in range(2 * STATE_DIM)]
    self.zero_sigma_points = [np.zeros(STATE_DIM).reshape(-1, 1) for _ in range(2 * STATE_DIM)]
    self.transformed_sigma_points = [np.zeros(STATE_DIM).reshape(-1, 1) for _ in range(2 * STATE_DIM)]
    self.measurement_sigma_points = [np.zeros(MEASUREMENT_DIM).reshape(-1, 1) for _ in range(2 * STATE_DIM)]
    self.adjusted_sigma_points = [np.zeros(STATE_DIM).reshape(-1, 1) for _ in range(2 * STATE_DIM)]

    # Mean Values
    self.state_estimate = np.zeros(STATE_DIM).reshape(-1, 1)
    self.priori_state_estimate = np.zeros(STATE_DIM).reshape(-1, 1)
    self.predicted_measurement_estimate = np.zeros(MEASUREMENT_DIM).reshape(-1, 1)

    # Measurement and Innovation
    self.measurement = np.zeros(MEASUREMENT_DIM).reshape(-1, 1)
    self.innovation = np.zeros(MEASUREMENT_DIM).reshape(-1, 1)

    # Kalman Gain
    self.kalman_gain = np.zeros((STATE_DIM, MEASUREMENT_DIM))

    # Time
    self.last_time = 0

  def prediction(self,time_diff):
    # 1
    self.zero_sigma_points = generate_zero_sigma_points(self.error_cov, self.process_noise_cov)
    print("TODO: 6D?")
    # 2
    self.sigma_points = generate_sigma_points(self.state_estimate, self.zero_sigma_points)
    
    # 3
    for i in range(2*STATE_DIM):
      self.transformed_sigma_points[i] = process_model(self.sigma_points[i], np.zeros(STATE_DIM).reshape(-1, 1), time_diff)
    
    # 4
    self.priori_state_estimate = mean_of_transformed_sigma_points(self.transformed_sigma_points)
    
    # 5
    self.adjusted_sigma_points = compute_adj_sigma_points(self.transformed_sigma_points, self.priori_state_estimate)
    
    # 6
    self.priori_error_cov = compute_priori_error_cov(self.adjusted_sigma_points)
    
    self.print_all()

  def update(self,acc,gyro,mag,time_diff):
    print("----measurements----")
    print("acc")
    print(acc)
    print("mag")
    print(mag)
    print("gyro")
    print(gyro)
    # 7
    self.measurement_sigma_points = compute_measurement_sigma_points(self.transformed_sigma_points,0)

    # 8
    self.predicted_measurement_estimate = mean_of_measurement_sigma_points(self.measurement_sigma_points)
    if CUR_MEASUREMENT_MODEL == 1:
      measurement = gyro
    elif CUR_MEASUREMENT_MODEL == 2:
      measurement = acc
    elif CUR_MEASUREMENT_MODEL == 3:
      measurement = mag
    else:
      measurement = -1
      exit(measurement)
      
    self.innovation = compute_innovation(self.predicted_measurement_estimate,measurement)
    
    # 9
    self.pred_measurement_cov = compute_pred_measurement_cov(self.predicted_measurement_estimate,self.measurement_sigma_points)
    self.innovation_cov = compute_innovation_cov(self.pred_measurement_cov,self.measurement_noise_cov)
    
    # 10
    self.cross_correlation = compute_cross_correlation(self.adjusted_sigma_points,self.measurement_sigma_points,self.predicted_measurement_estimate)
    
    # 11
    self.kalman_gain = compute_kalman_gain(self.cross_correlation,self.innovation_cov)
    self.state_estimate = compute_state_estimate(self.priori_state_estimate,self.kalman_gain,self.innovation)
    self.error_cov = compute_error_cov(self.priori_error_cov,self.kalman_gain,self.innovation_cov)
    
  
    self.print_all()


  def cycle(self,timestamp,acc,gyro,mag):
    global CUR_MEASUREMENT_MODEL
    print("CUR_MEASUREMENT_MODEL: ", CUR_MEASUREMENT_MODEL)
    time_diff = timestamp - self.last_time
    self.last_time = timestamp

    self.prediction(time_diff)
    self.update(acc,gyro,mag,time_diff)
    
    CUR_MEASUREMENT_MODEL += 1
    if CUR_MEASUREMENT_MODEL > 3:
      CUR_MEASUREMENT_MODEL = 1
    return self.state_estimate


  def print_all(self):
    if not PRINT_ENABLE:
      return
    print("======================")
    # state variables
    print("state_vector")
    print("NOT USED")
    print(self.state_vector)

    # Covariance Matrices
    print("error_cov")
    print(self.error_cov)
    print("priori_error_cov")
    print(self.priori_error_cov)
    print("process_noise_cov")
    print(self.process_noise_cov)
    print("measurement_noise_cov")
    print(self.measurement_noise_cov)
    print("pred_measurement_cov")
    print(self.pred_measurement_cov)
    print("innovation_cov")
    print(self.innovation_cov)
    print("cross_correlation")
    print(self.cross_correlation)

    # Sigma Points
    print("sigma_points")
    print(np.hstack(self.sigma_points))
    print("zero_sigma_points")
    print(np.hstack(self.zero_sigma_points))
    print("transformed_sigma_points")
    print(np.hstack(self.transformed_sigma_points))
    print("measurement_sigma_points")
    print(np.hstack(self.measurement_sigma_points))
    print("adjusted_sigma_points")
    print(np.hstack(self.adjusted_sigma_points))

    # Mean Values
    print("state_estimate")
    print(self.state_estimate)
    print("priori_state_estimate")
    print(self.priori_state_estimate)
    print("predicted_measurement_estimate")
    print(self.predicted_measurement_estimate)

    # Measurement and Innovation
    # print("measurement")
    # print(self.measurement)
    print("innovation")
    print(self.innovation)

    # Kalman Gain
    print("kalman_gain")
    print(self.kalman_gain)

    # Process and Measurement Models
    # self.process_model = None
    # self.measurement_model = None
    print("======================")
    
