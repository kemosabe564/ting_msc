import numpy as np
import math

# Author: Addison Sears-Collins
# https://automaticaddison.com
# Description: Extended Kalman Filter example (two-wheeled mobile robot)
 
# Supress scientific notation when printing NumPy arrays
# np.set_printoptions(precision=3,suppress=True)

class Kalman:
    def __init__(self):
        # A_k_1
        self.A_k_1 = np.array([[1.0,   0,   0],
                               [  0, 1.0,   0],
                               [  0,   0, 1.0]])
        # state_noise
        self.process_noise_v_k_minus_1 = np.array([0.00, 0.00, 0.000])

        self.Q_k = np.array([[0.01,     0,     0],
                             [    0, 0.01,     0],
                             [    0,     0,  0.005]])
             
        self.H_k = np.array([[1.0,   0,   0],
                             [  0, 1.0,   0],
                             [  0,   0, 1.0]])
                        
        self.R_k = np.array([[0.3,   0,    0],
                             [  0, 0.3,    0],
                             [  0,    0, 1.0]]) 
         
        self.sensor_noise_w_k = np.array([0.00, 0.00, 0.00])
        
        self.P_k_1 = np.array([[0.1,   0,   0],
                               [  0, 0.1,   0],
                               [  0,   0, 0.1]])

 
    def sr_EKF(self, z_k, state_estimate_k, dk = 1):

        ######################### Predict #############################
        # Predict the state estimate at time k based on the state 
        # estimate at time k-1 and the control input applied at time k-1.
        # B_K_1 = self.getB(state_estimate_k_1[2],dk)
        # state_estimate_k = self.A_k_1 @ (state_estimate_k_1) + (B_K_1) @ (u_k_1) + (self.process_noise_v_k_minus_1)
                
        # print(f'State Estimate Before EKF={state_estimate_k}')
                
        # Predict the state covariance estimate based on the previous
        # covariance and some noise
        P_k = self.A_k_1 @ self.P_k_1 @ self.A_k_1.T + (self.Q_k)
            
        ################### Update (Correct) ##########################
        # Calculate the difference between the actual sensor measurements
        # at time k minus what the measurement model predicted 
        # the sensor measurements would be for the current timestep k.
        measurement_residual_y_k = z_k - ((self.H_k @ state_estimate_k) + (self.sensor_noise_w_k))
    
        # print(f'Observation={z_k}')
                
        # Calculate the measurement residual covariance
        S_k = self.H_k @ P_k @ self.H_k.T + self.R_k
            
        # Calculate the near-optimal Kalman gain
        # We use pseudoinverse since some of the matrices might be
        # non-square or singular.
        K_k = P_k @ self.H_k.T @ np.linalg.pinv(S_k)
            
        # Calculate an updated state estimate for time k
        optimal_state_estimate_k = state_estimate_k + (K_k @ measurement_residual_y_k)
        
        # Update the state covariance estimate for time k
        P_k = P_k - (K_k @ self.H_k @ P_k)
        
        # Print the best (near-optimal) estimate of the current state of the robot
        # print(f'State Estimate After EKF={optimal_state_estimate_k}')
    
        # Return the updated state and covariance estimates
        return optimal_state_estimate_k, P_k
    
    
    def get_B_M(self, t):
        
        B_M = self.H_k
        
        for i in range(t-1):
            B_M = np.concatenate((B_M, self.H_k), axis = 1)

        return B_M
    
    def mr_EKF(self, z_k_M, state_estimate_k_M, u_k_1, dk = 1):
        
        B_M = self.get_B_M(5)
        
        P_k_M = self.A_k_1 @ self.P_k_1 @ self.A_k_1.T + 5 * self.Q_k
        
        measurement_residual_y_k = z_k_M - ((self.H_k @ state_estimate_k_M) + (self.sensor_noise_w_k))
        
        S_k_M = self.H_k @ P_k_M @ self.H_k.T + self.R_k
        
        K_k_M = P_k_M @ self.H_k.T @ np.linalg.pinv(S_k_M)
        
        optimal_state_estimate_k_M = state_estimate_k_M + (K_k_M @ measurement_residual_y_k)
        
        P_k_M = P_k_M - (K_k_M @ self.H_k @ P_k_M)
        
        return optimal_state_estimate_k_M, P_k_M
        