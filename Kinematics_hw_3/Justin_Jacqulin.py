# You are conducting some quadrotor testing in a motion capture arena, which provides you
# with ”ground truth” baseline data for your experiments.
# The motion capture system provides you with relevant 3-dimensional orientation data at
# each timestep. Orientation data is characterized by Euler angles in the Z-Y-X configuration
# relative to the motion capture system’s coordinates. Your quadrotor IMU is aligned with the
# body axes in the typical aerospace north-east-down (NED) configuration.
# Lately you’ve noticed that your quadrotor has not been flying as reliably as it once was and a
# senior graduate student has a hunch that you are getting some bad gyroscope readings after
# you heavily crashed the drone last week. You remember during this crash that the drone was
# not able to maintain a constant roll angle, and seemed to constantly drift in one direction,
# making it very hard for you to control. You decide to test this theory by manually moving the
# drone in the flight space. This allows you to compare ”ground truth” data from the motion
# capture system to onboard gyroscope readings, which will provide insight into the quality of
# gyroscope readings you are getting. Note that this question is to be completed within the
# code structure found in the file q1.py.
# a. [3 points] Import raw motion-capture data from the file mocap.csv. Import quadrotor
# gyroscope data from the file quad.csv. At what average rate does the quadrotor collect
# IMU data? At what average rate does the motion capture system collect data? Print
# these values in your python file (no need to include them in your pdf writeup).
# b. [2 points] Interpolate the motion capture data so that the timestamps align with those of
# the quadrotor gyroscope. You should now have 1200 distinct, equally spaced datapoints
# for both the motion-capture and gyroscope data after this step. Bonus question [2
# points]: why is it a good idea to interpolate the motion capture data to align with the
# timestamps of the gyroscope and not they other way around? If you answer this question,
# please include it in the pdf writeup and clearly indicate that you attempted this bonus
# question.
# c. [5 points] The gyroscope data is aligned with the standard aerospace north-east-down (X-
# Y-Z) reference frame attached to the quadrotor body (F N ED
# b = [xN ED
# b , yN ED
# b , zN ED
# b ]).
# The motion capture system, however, uses an inertial (fixed) reference frame as defined
# by the coordinate axes seen in the figure (F M
# i = [xM
# i , yM
# i , zM
# i ]). Rotate the motion
# capture orientation data to align with an inertial reference frame in the north-east-down
# configuration (F N ED
# i = [xN ED
# i , yN ED
# i , zN ED
# i ]) to match the quadrotor body reference
# frame.
# d. [5 points] The motion capture system provides pose tracking information in the form
# of Euler angles. From this data, estimate the Euler rates (  ̇ψ,  ̇θ,  ̇φ) of the quadrotor
# from the motion capture pose data. Hint: use a central difference scheme for numerical
# differentiation.
# e. [10 points] The onboard gyroscope data is measured in local body coordinates which
# give you the body rates - p, q and r. Calculate estimated body rate data from the Euler
# rate data you calculated in part (d).
# f. [5 points] Plot both sets of body rate data (measured by the onboard IMU and calculated
# from the motion capture data). Also include the associated Euler rate values for each
# axis. A good idea would be to plot each axis on a separate subplot, where each plot
# has 3 data lines: gyroscope rate, estimated body rate from motion capture data and
# associated Euler rates estimated from motion capture data.
# Be sure to include axes labels, title(s), appropriate fonts and font sizes and a legend.
# Make sure the data is easily distinguishable on the figure.
# In a separate figure, plot the ”error” of the gyroscope readings for each axis (assuming
# the motion capture system provides the ”ground truth”).
# g. [2 points] Based on your plot in (f), are the Euler rates equal to the body rates you
# calculated? Why or why not?
# h. [3 point] Do you think the quadrotor IMU is producing bad gyroscope readings? Why
# are these readings bad? How do they impact the performance of your quadrotor? Ex-
# plain your answer. Hint: Think about how the quadrotor calculates its orientation using
# gyroscope readings

import numpy as np
import math as m
import pandas as pd
from matplotlib import pyplot as plt
from scipy import interpolate

# HELPER FUNCTIONS

def differentiate_data(sample_rate, data):
    # takes the numerical derivative of data based on the sampling rate
    # INPUTS:
    # sample_rate: rate at which data is sampled
    # data: data to be differentiated
    # OUTPUTS:
    # diff_data - differentiated data
    data = data.to_numpy()
    # use central differencing scheme for differentiation
    diff_data = central_diff(sample_rate, data) 
    
    # use simple forward difference scheme for first and last datapoint
    diff_data[0,:] = forward_backward_diff(sample_rate, data[0:2,:])
    diff_data[-1,:] = forward_backward_diff(sample_rate, data[-3:-1,:])
    
    return diff_data

def central_diff(sample_rate, data):
    # uses the central differencing scheme to calculate numerical derivatives
    # INPUTS:
    # sample_rate: rate at which data is sampled
    # data: data to be differentiated
    # OUTPUTS:
    # diff_data - differentiated data
    
    l = data.shape[0]
    n = data.shape[1]
    centralDiff = np.zeros((l, n))
    
    for i in range(1, l - 1):
        centralDiff[i, :] = (data[i + 1, :] - data[i - 1, :]) * (sample_rate / 2)
    
    return centralDiff

def forward_backward_diff(sample_rate, data):
    # forward or backward difference calculation
    forwardBackwardDiff = (data[1,:] - data[0,:]) * sample_rate
    return forwardBackwardDiff  

def eulRate2BodyMatrix(phi, theta, psi):
    # calculates the rotation matrix from euler rates to body rates
    # assumes 3-2-1 rotation order
    # INPUTS:
    # phi - rotation about x (Roll)
    # theta - rotation about y (Pitch)
    # psi - rotation about z (Yaw)
    # OUTPUTS:
    # R - 3x3 rotation matrix
    
    R = np.array([
        [1, 0, -np.sin(theta)],
        [0, np.cos(phi), np.cos(theta) * np.sin(phi)],
        [0, -np.sin(phi), np.cos(theta) * np.cos(phi)]
    ])
    
    return R

# IMPORT DATA

# Import motion capture data
motion_data = pd.read_csv("./hw_3/mocap_data.csv", header=None)
m_timestamp = motion_data[0]
x, y, z = motion_data[1], motion_data[2], motion_data[3]

# Calculate motion capture sampling rate
num_sample = len(m_timestamp)
total_time = m_timestamp.iloc[-1] - m_timestamp.iloc[0]
mocap_sampling_rate = num_sample / total_time
print(f"Sampling rate of motion capture data: {mocap_sampling_rate} ")

# Import gyroscope data
gyro_data = pd.read_csv("./hw_3/gyro_data.csv", header=None)
gyro_time = gyro_data[0]
gyro_x, gyro_y, gyro_z = gyro_data[1], gyro_data[2], gyro_data[3]

# Calculate gyro sampling rate
gyro_sampling_rate = len(gyro_time) / (gyro_time.iloc[-1] - gyro_time.iloc[0])
print(f"Sampling rate of gyroscope data: {gyro_sampling_rate} ")

# INTERPOLATE MOCAP DATA TO GYRO TIMESTAMPS
interpolated_x = np.interp(gyro_time, m_timestamp, x)
interpolated_y = np.interp(gyro_time, m_timestamp, y)
interpolated_z = np.interp(gyro_time, m_timestamp, z)

# Create a DataFrame with interpolated data
aligned_data = pd.DataFrame({
    'gyro_time': gyro_time,
    'gyro_x': gyro_x,
    'gyro_y': gyro_y,
    'gyro_z': gyro_z,
    'mocap_x': interpolated_x,
    'mocap_y': interpolated_y,
    'mocap_z': interpolated_z
})
print(f"Interpolated Data { aligned_data.head()}")
# ROTATE MOCAP DATA TO INERTIAL NED FRAME
# rotation_matrix = R_z @ R_x
rotation_matrix = np.array([
    [0,0,1],
    [1,0,0],
    [0,-1,0]
])


n_x, n_y, n_z = [], [], []
for i in range(len(gyro_time)):
    vec = np.array([interpolated_x[i], interpolated_y[i], interpolated_z[i]])
    transformed_vec = rotation_matrix @ vec
    n_x.append(transformed_vec[0])
    n_y.append(transformed_vec[1])
    n_z.append(transformed_vec[2])

rotated_mocap = pd.DataFrame({
    'x': n_x,
    'y': n_y,
    'z': n_z
})
print(f"Rotated Values : {rotated_mocap.values}")
euler_rates = differentiate_data(mocap_sampling_rate, rotated_mocap)
print(f"Euler rates: {euler_rates} \n Euler rates shape :{euler_rates.shape}")

# CALCULATE BODY RATES FROM EULER RATES
body_rates = np.zeros_like(euler_rates)
for i in range(len(gyro_time)):
  
    phi, theta, psi = n_x[i],n_y[i],n_z[i] 
    R = eulRate2BodyMatrix(phi, theta, psi)
    body_rates[i] = R @ euler_rates[i]

gyro_body_rates = np.vstack((gyro_x, gyro_y, gyro_z)).T

print(f"Body rates Shape : {gyro_body_rates}")
# PLOT BODY RATES
plt.figure(figsize=(10, 8))

# Roll (p)
plt.subplot(311)
plt.plot(gyro_time, body_rates[:, 0], label="Estimated Body rate p (roll)", color="blue")
plt.plot(gyro_time, euler_rates[:, 0], label="Euler rate p (roll)", linestyle='--', color="green")
plt.plot(gyro_time, gyro_body_rates[:, 0], label="Gyro rate p (roll)", linestyle=':', color="red")
plt.ylim([-0.15, 0.15])
plt.legend()

# Pitch (q)
plt.subplot(312)
plt.plot(gyro_time, body_rates[:, 1], label="Estimated Body rate q (pitch)", color="blue")
plt.plot(gyro_time, euler_rates[:, 1], label="Euler rate q (pitch)", linestyle='--', color="green")
plt.plot(gyro_time, gyro_body_rates[:, 1], label="Gyro rate q (pitch)", linestyle=':', color="red")
plt.ylim([-0.15, 0.15])
plt.legend()

# Yaw (r)
plt.subplot(313)
plt.plot(gyro_time, body_rates[:, 2], label="Estimated Body rate r (yaw)", color="blue")
plt.plot(gyro_time, euler_rates[:, 2], label="Euler rate r (yaw)", linestyle='--', color="green")
plt.plot(gyro_time, gyro_body_rates[:, 2], label="Gyro rate r (yaw)", linestyle=':', color="red")
plt.ylim([-0.15, 0.15])
plt.legend()

plt.xlabel("Time (s)")
plt.show()

errors = gyro_body_rates - body_rates

plt.figure()
plt.subplot(311)
plt.plot(gyro_time, errors[:, 0], label="Error in p (roll)")
plt.ylim([-0.1, 0.1])
plt.legend()

plt.subplot(312)
plt.plot(gyro_time, errors[:, 1], label="Error in q (pitch)")
plt.ylim([-0.1, 0.1])
plt.legend()

plt.subplot(313)
plt.plot(gyro_time, errors[:, 2], label="Error in r (yaw)")
plt.ylim([-0.1, 0.1])
plt.legend()
plt.xlabel("Time (s)")
plt.show()

# Question 1(g): Answer - Are Euler Rates Equal to Body Rates?
print("No, Euler rates are not equal to body rates because they represent different rotational speeds.")
print("Euler rates are the rates of change of angles in a non-inertial frame, while body rates are the rotational speeds in the body's own coordinate system.")

# Question 1(h): Answer - Are Gyroscope Readings Faulty?
print("The gyroscope appears to provide inconsistent readings based on the errors seen in the plots.")
print("Such errors likely affect control accuracy, as the quadrotor depends on accurate gyroscope data to maintain stable orientation and respond precisely to control inputs.")


# Question 2
# Consider a planar 3-link manipulator
# with the following properties:
# Link 1: Length L1 = 2 m, angle θ1 = 30◦
# Link 2: Length L2 = 1.5 m, angle θ2 = 45◦
# Link 3: Length L3 = 1 m, angle θ3 = 60◦
# a. [5 points] Write python code that leverages these values alongside relevant translation
# matrices T1, T2, T3 and T4 to find the (x,y) coordinates of the end-effector.
# b. [2 points] Using the same code, identify the end-effector coordinates when the angles are
# θ1 = 120◦, θ2 = 60◦, θ3 = 30◦
# c. [3 points] On paper, write the set of analytical equations to perform inverse kinematics
# on the manipulator. Here, the result would be two equations with three variables where
# the x and y coordinates of the end effector is related to the joint angles θ1, θ2 and θ3. Use
# these equations to find the set of angles θ1, θ2 and θ3 when the end-effector coordinates
# are (4.387,1).
# d. [2 points] On paper, use the same set of analytical equations to perform inverse kine-
# matics on the manipulator to find the set of angles θ3 and θ2 when θ1 = 45◦ when the
# end-effector coordinates are (2,2)

import numpy as np
from scipy.optimize import fsolve

def transformation_matrix(L, theta_deg):
    # Convert angle to radians
    theta = np.radians(theta_deg)
    return np.array([
        [np.cos(theta), -np.sin(theta), L * np.cos(theta)],
        [np.sin(theta), np.cos(theta), L * np.sin(theta)],
        [0, 0, 1]
    ])

def end_effector_position(L1, theta1, L2, theta2, L3, theta3):
    # Compute individual transformation matrices
    T1 = transformation_matrix(L1, theta1)
    T2 = transformation_matrix(L2, theta2)
    T3 = transformation_matrix(L3, theta3)
    
    # Multiply T1, T2, and T3 to get the final transformation matrix T
    T = T1 @ T2 @ T3
    
    # Extract the x, y coordinates of the end-effector
    x, y = T[0, 2], T[1, 2]
    return x, y

def equations_fixed_theta3(angles, theta3_fixed):
    x_target, y_target = 4.387, 1
    theta1, theta2 = angles  # Unpack theta1 and theta2 in radians
    # Fixed theta3 value
    theta3 = theta3_fixed
    # Compute x and y based on the angles provided
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2) + L3 * np.cos(theta1 + theta2 + theta3)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2) + L3 * np.sin(theta1 + theta2 + theta3)
    return [x - x_target, y - y_target]


# Part (a) with initial angles, finding the coordinated of end effector
L1, theta1 = 2, 30
L2, theta2 = 1.5, 45
L3, theta3 = 1, 60
x, y = end_effector_position(L1, theta1, L2, theta2, L3, theta3)
print(f"End-effector position with initial angles: x = {x}, y = {y}")

# Part (b) with new angles
theta1_new, theta2_new, theta3_new = 120, 60, 30
x_new, y_new = end_effector_position(L1, theta1_new, L2, theta2_new, L3, theta3_new)
print(f"End-effector position with new angles: x = {x_new}, y = {y_new}")


# part c) inverse kinematics
# Fix theta3 (for example, at 30 degrees)
theta3_fixed = np.radians(30)

# Initial guesses for theta1 and theta2 in radians
initial_guess_theta1_theta2 = [np.radians(0), np.radians(0)]

# Solve the equations for fixed theta3
theta1_sol, theta2_sol = fsolve(equations_fixed_theta3, initial_guess_theta1_theta2, args=(theta3_fixed,))

# Convert solutions from radians to degrees
theta1_deg = np.degrees(theta1_sol)
theta2_deg = np.degrees(theta2_sol)
theta3_deg = np.degrees(theta3_fixed)

print(f"Theta1: {theta1_deg:.2f}°, Theta2: {theta2_deg:.2f}°, Theta3: {theta3_deg:.2f}°")

# part d) inverse kinematics when theta1 = 45
theta3_fixed = np.radians(45)

# Initial guesses for theta1 and theta2 in radians
initial_guess_theta1_theta2 = [np.radians(0), np.radians(0)]

# Solve the equations for fixed theta3
theta1_sol, theta2_sol = fsolve(equations_fixed_theta3, initial_guess_theta1_theta2, args=(theta3_fixed,))

# Convert solutions from radians to degrees
theta1_deg = np.degrees(theta1_sol)
theta2_deg = np.degrees(theta2_sol)
theta3_deg = np.degrees(theta3_fixed)

print(f"Theta1: {theta1_deg:.2f}°, Theta2: {theta2_deg:.2f}°, Theta3: {theta3_deg:.2f}°")


# Question 3
# Consider the SCARA robot shown in the figure below:
# a. [5 points] On paper, calculate the Denavit-Hartenberg parameters for this robot and
# derive the transformation matrices between every consecutive pair of joints, resulting
# in T1, T2, T3, T4 and T5. Use these transformation matrices to find the transformation
# between the base of the robot and the end-effector. Assume that a1 = 0.15, b1 = 0.3 m
# , a2 = 0.2 m, b2 = 0.1 m , b3 is a variable and b4 = 0 m.
# b. [8 points] Implement the DH parameters in python, resulting in a function forward kinematics,
# which takes the angles θ1, θ2, b3 and θ4 as input and returns the (x,y,z) coordinate of
# the end-effector.
# c. [3 points] Using matplotlib, plot a 3D graph of the region that can be covered by this
# manipulator when θ4 = 0◦, b3 = 0.1 m and θ2 = 0◦, whereas θ1 is a variable. θ1 has
# actuator limits of [0, 2π]. (Hint : For every 1◦ increment in θ1, plot the (x,y,z) coordinate
# of the end-effector, resulting in a planar circle in 3-dimensions.)
# d. [7 points] Using matplotlib, plot a 3D graph of the region that can be covered by this
# manipulator when θ4 = 0◦ and θ2 = 0◦, whereas θ1 and b3 are variables. θ1 has actu-
# ator limits of [0, 2π] while b3 has actuator limits of [0 m, 0.3 m]. (Hint : For every 1◦
# increment in θ1 and for every 0.05 m increment in b3, plot the (x,y,z) coordinate of the
# end-effector, resulting in a cylinder in 3-dimensions.)

import numpy as np
import matplotlib.pyplot as plt

# Define the DH transformation matrix function
def dh_transform_matrix(a, alpha, d, theta):
  
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, a],
        [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha), d * np.sin(alpha)],
        [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha), d * np.cos(alpha)],
        [0, 0, 0, 1]
    ])

# Define the forward kinematics function
def forward_kinematics(theta1, theta2, b3, theta4):
    
    a1, b1 = 0.15, 0.3
    a2, b2 = 0.2, 0.1
    b4 = 0
    
    T1 = dh_transform_matrix(a1, 0, b1, theta1)
    T2 = dh_transform_matrix(a2, 0, b2, theta2)
    T3 = dh_transform_matrix(0, 0, b3, 0)
    T4 = dh_transform_matrix(0, 0, b4, theta4)
    T5 = dh_transform_matrix(0, 0, 0, 0) 
    
    # Compute the final transformation matrix 
    T_base_end_effector = T1 @ T2 @ T3 @ T4 @ T5
    
    # Extract the (x, y, z) position of the end-effector
    x, y, z = T_base_end_effector[0, 3], T_base_end_effector[1, 3], T_base_end_effector[2, 3]
    
    return x, y, z


theta1 = np.radians(30)  
theta2 = np.radians(45)
b3 = 0.15
theta4 = np.radians(60)

x, y, z = forward_kinematics(theta1, theta2, b3, theta4)
print(f"End-effector position: x = {x}, y = {y}, z = {z} for theta1 = {theta1}, theta2 = {theta2}, b3 = {b3}, theta4 = {theta4}")

# Part (c): 3D Plot for planar circle with varying theta1, fixed theta2, b3, and theta4
theta2 = np.radians(0)
b3 = -0.1
theta4 = np.radians(0)


x_vals, y_vals, z_vals = [], [], []


for theta1_deg in range(0, 360):
    theta1 = np.radians(theta1_deg)
    x, y, z = forward_kinematics(theta1, theta2, b3, theta4)
    x_vals.append(x)
    y_vals.append(y)
    z_vals.append(z)

# Plotting the planar circle
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x_vals, y_vals, z_vals, 'b', label='Planar Circle')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('3D Plot of End-Effector Position (Planar Circle)')
ax.legend()
plt.show()

# Part (d): 3D Plot for cylindrical coverage with varying theta1 and b3, fixed theta2 and theta4
x_vals_cylinder, y_vals_cylinder, z_vals_cylinder = [], [], []


for theta1_deg in range(0, 360):
    theta1 = np.radians(theta1_deg)
    for b3 in np.arange(-0.3, 0.0, 0.05):
        x, y, z = forward_kinematics(theta1, theta2, b3, theta4)
        x_vals_cylinder.append(x)
        y_vals_cylinder.append(y)
        z_vals_cylinder.append(z)

# Plotting the cylinder
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# print(x_vals_cylinder)
ax.scatter(x_vals_cylinder, y_vals_cylinder, z_vals_cylinder, c='r', marker='o', label='Cylindrical Coverage')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('3D Plot of End-Effector Position (Cylinder)')
ax.legend()
plt.show()