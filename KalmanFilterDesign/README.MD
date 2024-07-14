

# kalman filter
| in code | in paper | 
|----------|----------|
| state_vector | $x_k$ |
| error_cov | $P_k$ |
| priori_error_cov | $P^-_k$ |
| process_noise_cov | $Q$ |
| measurement_noise_cov | $R$ |
| pred_measurement_cov | $P_{zz}$ |
| innovation_cov | $P_{vv}$ |
| cross_correlation | $P_{xz}$ |
| sigma_points | $X_i$ |
| zero_sigma_points | $W_i$ |
| transformed_sigma_points | $Y_i$ |
| measurement_sigma_points | $Z_i$ |
| adjusted_sigma_points | $W'_i$ |
| state_estimate | $\hat{x}_k$ |
| priori_state_estimate | $\hat{x}_k^-$ |
| predicted_measurement_estimate | $\hat{z}_k^-$ |
| measurement | $z_k$ |
| innovation | $ν_k$ |
| kalman_gain | $K_k$ |
| process_model | $A(x,w)$ |
| measurement_model | $H(x,v)$ |
|  | $$ |


# kalman filter test
| in code | in paper | 
|----------|----------|
| state_vector | $\hat{x}_k$ |
| error_covariance_matrix | $P_k$ |
| process_noise_covariance_matrix | $Q$ |
| measurement_noise | $v$ |
| sigma_points | $X$ |
| state_vector_set | $Y$ |
| state_vector_set_mean | $\hat{x}^-_k$ |
| state_vector_set_cov | $P^-_k$ |
| measurement | $z_k$ |
| projected_measurement_vectors | $Z$ |
| expected_measurement_vector | $z^-_k$ |
| measurement_estimate_cov | $P_{zz}$ |
| measurement_noise_cov | $R$ |
| expected_covariance | $P_{vv}$ |
| cross_correlation_matrix | $P_{xz}$ |

