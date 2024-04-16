# KalmanAttitudeAuroraEstimator
This Repository will be used for the Bachelor Thesis "Real-Time Kalman Filter based attitude and theoretic Aurora Borealis direction Estimation""



## Research
- [Interactive explanation of Extended Kalman Filter](https://simondlevy.github.io/ekf-tutorial/)
- [Extended Kalman Filter Tutorial](https://homes.cs.washington.edu/~todorov/courses/cseP590/readings/tutorialEKF.pdf)
- [Maybe unscented Kalman Filter better?](https://www.cs.unc.edu/~welch/kalman/media/pdf/Julier1997_SPIE_KF.pdf)
- [Making EKF and UKF more robust](https://msol.people.uic.edu/ECE531/papers/Robust%20Kalman%20filtering%20for%20Satellite%20Attitude%20Estimation.pdf)
- [Kalman Filter using Sun Sensor, Gyro and Magnetometer](https://www.sciencedirect.com/science/article/pii/S187770581101678X)
- [Kalman Filter using Sun Sensor, Gyro and Magnetometer 2](https://www.researchgate.net/publication/268555795_Attitude_Determination_by_Magnetometer_and_Gyros_During_Eclipse)
- [Unscented Kalman Filter Paper](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=882463)
- [Video Guide](https://www.youtube.com/watch?v=c_6WDC66aVk)




## UKF
State: x_t
Initial State: x_0
Control Input: u_t
Process Noise: w_t
Measurement: y_t
Measurement Noise: n_t


### Prediction Step
Transformation Function?
Cholesky Decomposition
1) Define AUgmented State and Covariance
2) Compute Sigma Points and Weights
3) Decompose Sigma Points
4) Pass Through Motion Model
5) Compute Resulting Mean and Covariance

### Correction Step
0) Generalized Gaussian Filter: Another KF Correction Derivation
1) Define Augmented STate Covariance
2) Compute Sigma Points and Weights
3) Decompose Sigma Points
4) Pass Through Motion Model
5) Compute Resulting Mean, Covariance and Cross Covariance
6) Fill in Generalized Gaussian Filter!
