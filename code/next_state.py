from modern_robotics import core
import numpy as np
from numpy import cos, sin
from math import pi
import csv

'''
    To run the file : python code/next_state.py 
    Results for the tests are in testing_milestone_results/milestone 1
'''

"""
Author ashleetiwari2021@u.northwestern.edu
Milestone 1: 
"""


def NextState(current_config, speeds, dt, max_ang_speed):
    """ computes the configuration of the robot in the next time step.
    # -Using simple first-order Euler step, i.e.:
    Input:
      current_config - A 12-vector representing the current configuration of the robot (3 variables for
                                    the chassis configuration, 5 variables for the arm configuration, and 4 variables
                                    for the wheel angles).
      speeds - A 9-vector of controls indicating the arm joint speeds theta_dot (5 variables) and the
                                    wheel speeds u (4 variables).
      dt - The time step Δt.
      max_ang_speed - A positive real value indicating the maximum angular speed of the arm joints and
                                    the wheels.
    Return: 
      new_config - A 12-vector representing the configuration of the robot time Δt later.
    """
    # The forward-backward distance between the wheels to frame {b} 
    r = 0.0475			#  radius of each wheel
    l = 0.47/2
    w = 0.3/2			# side-to-side distance between the wheels to frame {b} 
    

    # Get current chassis configuration, arm configuration (joints angles) and wheel angles:
    current_q = current_config[:3]
    current_joint_ang = current_config[3:8]
    current_wheel_ang = current_config[8:12]

    # Restrict the speeds executed by the wheels and joints to the maximum speed:
    for i in range(len(speeds)):
        abs_speed = abs(speeds[i])
        if abs_speed > max_ang_speed:
            speeds[i] = speeds[i]/abs_speed * max_ang_speed

    # Get current arm joint speeds and wheel speeds (with restricted speeds):
    theta_dot = speeds[:5]
    u = speeds[5:]

    # Calculate new arm joint angles and wheel angles (according to equations 1,2):
    new_joint_ang = current_joint_ang + theta_dot * dt
    new_wheel_ang = current_wheel_ang + u * dt

    # Calculate new chasis configuration (according to Chapter 13.4):
    F = r/4 * np.array([[-1/(l + w), 1/(l + w), 1/(l + w), -1/(l + w)],
                        [1,         1,         1,          1],
                        [-1,         1,        -1,          1]])
    Vb = F.dot((u * dt).T).T
    w_bz, v_bx, v_by = Vb

    if w_bz == 0.:
        delta_qb = np.array([0, v_bx, v_by]).T
    else:
        delta_qb = np.array([w_bz, (v_bx * sin(w_bz) + v_by * (cos(w_bz) - 1))/w_bz,
                             (v_by * sin(w_bz) + v_bx * (1 - cos(w_bz)))/w_bz])

    # Transforming the ∆q b in {b} to ∆q in the fixed frame {s} using the chassis angle:
    theta = current_config[0]
    Tsb = np.array([[1,                  0,					  0],
                    [0, cos(theta), -sin(theta)],
                    [0, sin(theta),  cos(theta)]])
    delta_q = Tsb.dot(delta_qb.T)

    # Calculating new chasis configuration:
    new_q = current_q + delta_q

    # Combining the three vectors to the new configuration vector:
    new_config = np.concatenate(
        (new_q, new_joint_ang, new_wheel_ang), axis=None)

    return new_config



current_config = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])


###############################  testing milestone 1  #######################################

############  test 1 #################

# speeds = np.array([0, 0, 0, 0, 0, 10, 10, 10, 10])      # #The robot chassis should move forward 

############  test 2 #################

# speeds = np.array([0, 0, 0, 0, 0, -10, 10, -10, 10])   #The robot chassis should slide sideways 



############  test 3 #################

# speeds = np.array([0, 0, 0, 0, 0, -10, 10, 10, -10])   # robot rotates clockwise 
# # Restrictions on the speeds vector:
# max_ang_speed = 5

# dt = 0.01						# Time step [sec]
# t_total = 1							# Simulation run time [sec]
# iteration = int(t_total/dt)  # Number of iterations

# # Initialize configuration array (with current_config as the first raw):
# config_array = np.zeros((iteration, 13))
# config_array[0] = current_config

# # Calculate the new configuration for every iteration:
# for i in range(1, iteration):
#     current_config = NextState(current_config, speeds, dt, max_ang_speed)
#     config_array[i][:12] = current_config

# with open("clockwise_rot.csv", "w+") as my_csv:
#     csvWriter = csv.writer(my_csv, delimiter=',')
#     csvWriter.writerows(config_array)