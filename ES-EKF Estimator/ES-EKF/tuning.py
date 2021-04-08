import pickle
import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from rotations import angle_normalize, rpy_jacobian_axis_angle, skew_symmetric, Quaternion
import time
import sys

with open('data/pt1_data.pkl', 'rb') as file:
    data = pickle.load(file)

gt = data['gt']
imu_f = data['imu_f']
imu_w = data['imu_w']
gnss = data['gnss']
lidar = data['lidar']

# Correct calibration rotation matrix, corresponding to Euler RPY angles (0.05, 0.05, 0.1).
C_li = np.array([
   [ 0.99376, -0.09722,  0.05466],
   [ 0.09971,  0.99401, -0.04475],
   [-0.04998,  0.04992,  0.9975 ]
])

# Incorrect calibration rotation matrix, corresponding to Euler RPY angles (0.05, 0.05, 0.05).
# C_li = np.array([
#      [ 0.9975 , -0.04742,  0.05235],
#      [ 0.04992,  0.99763, -0.04742],
#      [-0.04998,  0.04992,  0.9975 ]
# ])

t_i_li = np.array([0.5, 0.1, 0.5])

# Transform from the LIDAR frame to the vehicle (IMU) frame.
lidar.data = (C_li @ lidar.data.T).T + t_i_li

var_imu_f = 0.01
var_imu_w = 0.24
var_gnss  = 0.01
var_lidar = 100

g = np.array([0, 0, -9.81])  # gravity
l_jac = np.zeros([9, 6])
l_jac[3:, :] = np.eye(6)  # motion model noise jacobian
h_jac = np.zeros([3, 9])
h_jac[:, :3] = np.eye(3)  # measurement model jacobian

p_est = np.zeros([imu_f.data.shape[0], 3])  # position estimates
v_est = np.zeros([imu_f.data.shape[0], 3])  # velocity estimates
q_est = np.zeros([imu_f.data.shape[0], 4])  # orientation estimates as quaternions
p_cov = np.zeros([imu_f.data.shape[0], 9, 9])  # covariance matrices at each timestep

p_est[0] = gt.p[0]
v_est[0] = gt.v[0]
q_est[0] = Quaternion(euler=gt.r[0]).to_numpy()
p_cov[0] = np.zeros(9)  # covariance of estimate
gnss_i  = 0  # index of GNSS data timestamps to see if GNSS data is available
lidar_i = 0  # index of LIDAR data timestamps to see if LIDAR data is available

def measurement_update(sensor_var, p_cov_check, y_k, p_check, v_check, q_check):
    # 3.1 Compute Kalman Gain
    R = sensor_var * np.eye(3)  # sensor variance (var_gnss or var_lidar)
    Kk = p_cov_check @ h_jac.T @ inv(h_jac @ p_cov_check @ h_jac.T + R)

    # 3.2 Compute error state
    delta_xk = Kk @ (y_k - p_check)
    # xk in R^9, composed of position 3x1, velocity 3x1, and orientation 3x1 in Euler

    # 3.3 Correct predicted state
    p_hat = p_check + delta_xk[:3]
    v_hat = v_check + delta_xk[3:6]
    q_hat = Quaternion(euler=delta_xk[6:]).quat_mult_left(q_check)

    # 3.4 Compute corrected covariance
    p_cov_hat = (np.eye(9) - Kk @ h_jac) @ p_cov_check

    return p_hat, v_hat, q_hat, p_cov_hat

def main():
    gnss_i  = 0
    lidar_i = 0 
    for k in range(1, imu_f.data.shape[0]):  # start at 1 b/c we have initial prediction from gt
        delta_t = imu_f.t[k] - imu_f.t[k - 1]
        f = imu_f.data[k-1]

        # 1. Update state with IMU inputs
        # 1.1 Rotation matrix from sensor frame to navigation frame
        Cns = Quaternion(q_est[k-1][0], q_est[k-1][1], q_est[k-1][2], q_est[k-1][3]).to_mat()
        p_est[k] = p_est[k-1] + delta_t * v_est[k-1] + 0.5 * delta_t**2 * (Cns @ f + g)
        v_est[k] = v_est[k-1] + delta_t * (Cns @ f + g)
        q_tmp = imu_w.data[k-1] * delta_t
        q_est[k] = Quaternion(euler=q_tmp).quat_mult_right(q_est[k-1])

        # 1.2 Linearize the motion model and compute Jacobians
        F = np.eye(9)
        F[:3,3:6] = delta_t * np.eye(3)
        F[3:6,6:] = -skew_symmetric(Cns @ f) * delta_t
        Q = np.eye(6)
        Q[:3,:3] *= delta_t**2 * var_imu_f
        Q[-3:,-3:] *= delta_t**2 * var_imu_w

        # 2. Propagate uncertainty
        p_cov[k] = F @ p_cov[k-1] @ F.T + l_jac @ Q @ l_jac.T

        # 3. Check availability of GNSS and LIDAR measurements
        if gnss_i < gnss.t.shape[0] and gnss.t[gnss_i] == imu_f.t[k-1]:
            p_est[k], v_est[k], q_est[k], p_cov[k] = measurement_update(
                var_gnss, p_cov[k], gnss.data[gnss_i], p_est[k], v_est[k], q_est[k])
            gnss_i += 1
        
        if lidar_i < lidar.t.shape[0] and lidar.t[lidar_i] == imu_f.t[k-1]:
            p_est[k], v_est[k], q_est[k], p_cov[k] = measurement_update(
                var_lidar, p_cov[k], lidar.data[lidar_i], p_est[k], v_est[k], q_est[k])
            lidar_i += 1

def calc_error():
    num_gt = gt.p.shape[0]
    p_est_euler = []

    size = (num_gt, 6)
    errors = np.zeros(size)

    # Convert estimated quaternions to euler angles
    for q in q_est:
        p_est_euler.append(Quaternion(*q).to_euler())
    p_est_euler = np.array(p_est_euler)

    # Get uncertainty estimates from P matrix
    #p_cov_diag_std = np.sqrt(np.diagonal(self.p_cov, axis1=1, axis2=2))

    errors[:,:3] = (gt.p - p_est[:num_gt])**2
    errors[:,3:6] = (gt.p - p_est_euler[:num_gt])**2
    avg_sqr_error = np.average(errors)    

    return avg_sqr_error

#### Variance Tuning ##########################################################################

################################################################################################
# I tuned the four variances at the same time, which would take a long time if the computation
# ability is not strong enough.
# Feel free to change the range of the values to choose from for each variance. 
# Set the first value in the list to be a non-zero value in order to avoid singular matrix when
# computing Kk in measurement update. 
################################################################################################
var_gnss_list = np.linspace(0, 10, num=11)
var_gnss_list[0] = 0.1
var_lidar_list = np.linspace(0, 150, num=11)
var_lidar_list[0] = 1

var_imu_f_list = np.linspace(0, 0.2, num=11)
var_imu_f_list[0] = 0.01
var_imu_w_list = np.linspace(0, 0.4, num=11)
var_imu_w_list[0] = 0.01

errors = np.zeros((11, 11, 11, 11))

#toolbar_width = 11 * 11 * 11 * 11  # number of iterations
#sys.stdout.write("[%s]" % (("-") * toolbar_width))
#sys.stdout.flush()

for i, var_gnss in enumerate(var_gnss_list):
    for j, var_lidar in enumerate(var_lidar_list):
        for m, var_imu_f in enumerate(var_imu_f_list):
            for n, var_imu_w in enumerate(var_imu_w_list):

                main()
                errors[i][j][m][n] = calc_error()
                best_idxs = np.argwhere(errors == np.min(errors))
                it = (i+1) * (j+1) * (m+1) * (n+1)
                if it % 500 == 0:
                    print("Iteration: ", it, " outta 14641")
        
                #sys.stdout.write("\r") # return to start of line
                #sys.stdout.flush()
                #sys.stdout.write("[") # Overwrite over the existing text from the start 
                #sys.stdout.write("#" * ((i+1)*(j+1))) # number of # denotes the progress completed 
                #sys.stdout.flush()
#sys.stdout.write("]\n")
best_var_1_idx = best_idxs[0][0]
best_var_2_idx = best_idxs[0][1]
best_var_3_idx = best_idxs[0][2]
best_var_4_idx = best_idxs[0][3]
ret = ''
ret += '%.3f  ' % (var_gnss_list[best_var_1_idx])
ret += '%.3f  ' % (var_lidar_list[best_var_2_idx])
ret += '%.3f  ' % (var_imu_f_list[best_var_3_idx])
ret += '%.3f  ' % (var_imu_w_list[best_var_4_idx])
with open('results.txt', 'w') as file:
    file.write(ret)
