import kalmanfilter as kf
import numpy as np

def main():
  KF = kf.KalmanFilter()
  KF.initialization()
  liste = []
  for i in range(4096):
    liste.append(KF.cycle(i,np.array([1,1,1]).reshape(-1,1)))
  for item in liste:
    print(item)

if __name__ == '__main__':
  main()  