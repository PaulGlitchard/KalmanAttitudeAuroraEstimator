import kalmanfilter as kf
import numpy as np

# Path to the CSV file
file_path = 'data/sensor_data.csv'

# Read the data
data = np.genfromtxt(file_path, delimiter=',', names=True)

# Extract timestamps and sensor data
timestamps = data['timestamp'].tolist()
acc_x = data['acc_x'].tolist()
acc_y = data['acc_y'].tolist()
acc_z = data['acc_z'].tolist()
gyro_x = data['gyro_x'].tolist()
gyro_y = data['gyro_y'].tolist()
gyro_z = data['gyro_z'].tolist()
mag_x = data['mag_x'].tolist()
mag_y = data['mag_y'].tolist()
mag_z = data['mag_z'].tolist()

def main():
  
  KF = kf.KalmanFilter()
  KF.initialization()
  liste = []
  for i in range(len(timestamps)):
    acc = np.array([acc_x[i],acc_y[i],acc_z[i]]).reshape(-1,1)
    gyro = np.array([gyro_x[i],gyro_y[i],gyro_z[i]]).reshape(-1,1)
    mag = np.array([mag_x[i],mag_y[i],mag_z[i]]).reshape(-1,1)
    liste.append(KF.cycle(i,acc,gyro,mag))
  for item in liste:
    print(item)

if __name__ == '__main__':
  main()  