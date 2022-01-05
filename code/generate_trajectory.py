import modern_robotics as mr
import numpy as np
import csv
from math import pi
'''
    To run the file : python code/generate_tracjectory.py 
    Results for the tests are in testing_milestone_results/milestone 2
'''



"""
Author:ashleetiwari2021@u.northwestern.edu
Milestone 2: 
TrajectoryGenerator function  generates the reference trajectory for the  end-effector frame {e}.
This trajectory consists of eight concatenated trajectory segments. Each trajectory segment beginsand ends at rest.
"""


def TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_goal, Tce_grasp, Tce_standoff, k):
    """ generates the reference trajectory for the end-effector frame {e}.
   
    Input:
      Tse_initial - The initial configuration of the end-effector in the reference trajectory.
      Tsc_initial - The cube's initial configuration.
      Tsc_goal - The cube's desired final configuration.
      Tce_grasp - The end-effector's configuration relative to the cube when it is grasping the cube.
      Tce_standoff - The end-effector's standoff configuration above the cube, before and after
                              grasping, relative to the cube.
      k - The number of trajectory reference configurations per 0.01 seconds.

    Return: 
      trajectory_list - N configurations of the end-effector along the entire
      concatenated eight-segment reference trajectory along with gripper state.

      trajectory = [r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper_state]
    """

    trajectory_list = []
    # Gripper initial state:
    gripper_state = 0  # Open
    # time-scaling method : considered fifth-order polynomial
    method = 5
    # total time of the motion in seconds:
    # T_tot = 5 
    # no of points (Start and stop) in the discrete representation of the trajectory
    N_driving = 380
    N_picking = 380

    # initial configuration of the end-effector above the cube(relative to the world frame) before and after grasping
    Tse_standoff = np.dot(Tsc_initial, Tce_standoff)

    # end-effector's configuration (relative to the world frame) when it is grasping the cube
    Tse_grasp = np.dot(Tsc_initial, Tce_grasp)

    # end-effector's configuration (relative to the world frame) when the cube arrives its final configuration
    Tse_goal = np.dot(Tsc_goal, Tce_standoff)

    # end-effector's final configuration (relative to the world frame)
    Tse_final = np.dot(Tsc_goal, Tce_grasp)

    ########## Generating Trajectory ##########

   # Initial to standoff postion
    trajectory_1 = np.asarray(mr.ScrewTrajectory(
        Tse_initial, Tse_standoff,  N_driving*0.01, N_driving, method))
    trajectory_list = get_trajectory(
        trajectory_list, trajectory_1, N_driving, gripper_state)

    # Standoff position to grasp position
    # N = N_picking
    trajectory_2 = np.asarray(mr.ScrewTrajectory(
        Tse_standoff, Tse_grasp,N_picking*0.01 , N_picking, method))
    trajectory_list = get_trajectory(trajectory_list, trajectory_2, N_picking, gripper_state)

    # Closing  the gripper
    N = N_picking
    gripper_state = 1
    trajectory_3 = np.asarray(mr.ScrewTrajectory(
        Tse_grasp, Tse_grasp, N_picking*0.01, N_picking, method))
    trajectory_list = get_trajectory(
        trajectory_list, trajectory_3, N_picking, gripper_state)

    # Grasp to standoff position:
    N = N_picking
    trajectory_4 = np.asarray(mr.ScrewTrajectory(
        Tse_grasp, Tse_standoff, N_picking*0.01, N_picking, method))
    trajectory_list = get_trajectory(
        trajectory_list, trajectory_4, N_picking, gripper_state)

    # Standoff to goal position:
    N = N_picking
    trajectory_5 = np.asarray(mr.ScrewTrajectory(
        Tse_standoff, Tse_goal, N_picking*0.01, N_picking, method))
    trajectory_list = get_trajectory(
        trajectory_list, trajectory_5, N_picking, gripper_state)

    # From goal position to final position:
    N = N_picking
    trajectory_6 = np.asarray(mr.ScrewTrajectory(
        Tse_goal, Tse_final, N_picking*0.01, N_picking, method))
    trajectory_list = get_trajectory(
        trajectory_list, trajectory_6, N_picking, gripper_state)

    # open the gripper:
    N = N_picking
    gripper_state = 0
    trajectory_7 = np.asarray(mr.ScrewTrajectory(
        Tse_final, Tse_final, N_picking*0.01, N_picking, method))
    trajectory_list = get_trajectory(
        trajectory_list, trajectory_7, N_picking, gripper_state)

    # From final position to standoff position:
    N = N_picking
    trajectory_8 = np.asarray(mr.ScrewTrajectory(
        Tse_final, Tse_goal, N_picking*0.01, N_picking, method))
    trajectory_list = get_trajectory(
        trajectory_list, trajectory_8, N_picking, gripper_state)

    return trajectory_list


def get_trajectory(trajectory_list, trajectory, N, gripper_state):
    """ 
    Input:
      trajectory_list: A representation of the N configurations of the end-effector along the entire
      concatenated eight-segment reference trajectory (including the gripper state).
      trajectory: The trajectory on the segment
      N: The number of points (Start and stop) in the discrete representation of the trajectory:
      gripper_state: The state of the gripper (0 = open, 1 = close)

    Return: 
      trajectory_list
    """

    for i in range(N):
        r11 = trajectory[i][0][0]
        r12 = trajectory[i][0][1]
        r13 = trajectory[i][0][2]
        r21 = trajectory[i][1][0]
        r22 = trajectory[i][1][1]
        r23 = trajectory[i][1][2]
        r31 = trajectory[i][2][0]
        r32 = trajectory[i][2][1]
        r33 = trajectory[i][2][2]
        px = trajectory[i][0][3]
        py = trajectory[i][1][3]
        pz = trajectory[i][2][3]

        trajectory_list.append(
            [r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper_state])
    return trajectory_list


#################################  testing milestone 2 #########################################

# The initial configuration of the end-effector in the reference trajectory:
Tse_initial = np.array([[0, 0, 1,   0],
                        [0, 1, 0,   0],
                        [-1, 0, 0, 0.5],
                        [0, 0, 0,   1]])

# The cube's initial configuration:
Tsc_initial = np.array([[1, 0, 0,     1],
                        [0, 1, 0,     0],
                        [0, 0, 1, 0.025],
                        [0, 0, 0,     1]])

# The cube's desired final configuration:
Tsc_goal = np.array([[0,  1, 0,     0],
                     [-1, 0, 0,    -1],
                     [0,  0, 1, 0.025],
                     [0,  0, 0,     1]])

# The end-effector's configuration relative to the cube when it is grasping the cube
# (the two frames located in the same coordinates, rotated about the y axis):
Tce_grasp = np.array([[-1/np.sqrt(2), 0,  1/np.sqrt(2), 0],
                      [0, 			  1,  0, 			0],
                      [-1/np.sqrt(2), 0, -1/np.sqrt(2), 0],
                      [0, 			  0, 0,          	1]])

# The end-effector's standoff configuration above the cube, before and after grasping, relative
# to the cube (the {e} frame located 0.1m above the {c} frame, rotated about the y axis):
Tce_standoff = np.array([[-1/np.sqrt(2), 0,  1/np.sqrt(2),   0],
                         [0, 			 1,  0,   			 0],
                         [-1/np.sqrt(2), 0, -1/np.sqrt(2), 0.1],
                         [0, 			 0,  0,   			 1]])

# The number of trajectory reference configurations per 0.01 seconds:
k = 1

# Call the function to get the trajectory:
trajectory = TrajectoryGenerator(
    Tse_initial, Tsc_initial, Tsc_goal, Tce_grasp, Tce_standoff, k)

# with open("trajectory.csv", "w+") as my_csv:
#     csvWriter = csv.writer(my_csv, delimiter=',')
#     csvWriter.writerows(trajectory)