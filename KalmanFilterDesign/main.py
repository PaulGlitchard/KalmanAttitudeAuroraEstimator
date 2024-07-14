import kalmanfilter as kf


def main():
  KF = kf.KalmanFilter()
  for i in range(10):
    KF.execute_kalman_filter(i)

if __name__ == '__main__':
  main()  