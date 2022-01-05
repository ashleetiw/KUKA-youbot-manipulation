import modern_robotics as mr
import numpy as np
from numpy import cos, sin
from math import pi
import logging
import csv
import matplotlib.pyplot as plt
from generate_trajectory import TrajectoryGenerator
from next_state import NextState
from feedback_control import FeedbackControl


def plot_data(Xerr_array,kp_gain,ki_gain):
    # Plot the error as function of time:
    logging.info('plotting error data')
    t_traj = np.linspace(1, 30, 3000)
    # print("Xerr_array: ", Xerr_array)
    # print("Xerr_array.shape: ", Xerr_array.shape)
    plt.figure()
    plt.plot(t_traj, Xerr_array[:, 0], label='Xerr[0]')
    plt.plot(t_traj, Xerr_array[:, 1], label='Xerr[1]')
    plt.plot(t_traj, Xerr_array[:, 2], label='Xerr[2]')
    plt.plot(t_traj, Xerr_array[:, 3], label='Xerr[3]')
    plt.plot(t_traj, Xerr_array[:, 4], label='Xerr[4]')
    plt.plot(t_traj, Xerr_array[:, 5], label='Xerr[5]')
    plt.title(f'Xerr, kp={kp_gain}, ki={ki_gain}')
    plt.xlabel('Time (s)')
    plt.ylabel('Error')
    plt.legend(loc="best")
    plt.grid()
    plt.savefig(f'Xerr,kp={kp_gain},ki={ki_gain}.png')
    plt.show()



# Initialize log file
LOG_FILENAME = 'best.log'
# LOG_FILENAME = 'overshoot.log'
# LOG_FILENAME = 'newtask.log'
# LOG_FILENAME = 'tril.log'
logging.basicConfig(filename=LOG_FILENAME, level=logging.INFO)

def main():
    """ The project's main function.
    This function Generates the robot's trajectory using the TrajectoryGenerator, NextState and FeedbackControl
    functions.
    Input:
      None
    Return:
      A saved csv file represents the N configurations of the end-effector along the entire
      concatenated eight-segment reference trajectory, including the gripper state (a 13-vector)
      A saved csv file represents the Xerr (a 6-vector)
      A saved plot of the Xerr changing with time
      A saved log file
    """

    ########## Initialization ##########

    # Initialization of configuration matrices:

    # The initial configuration of the robot
    # (chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper state)
    
    initial_config = np.array(
        [0.1, -0.2, 0, 0, 0, 0.2, -1.6, 0, 0, 0, 0, 0, 0])


    # # new task config 
    # initial_config = np.array([0.5236,.2,-.2,0,0,-1,0,0,0,0,0,0,0])

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
    Tsc_goal = np.array([[0, 1, 0,  	 0],
                         [-1, 0, 0,    -1],
                         [0, 0, 1, 0.025],
                         [0, 0, 0,     1]])


    # The end-effector's configuration relative to the cube when it is grasping the cube
    # (the two frames located in the same coordinates, rotated about the y axis):
    Tce_grasp = np.array([[-1/np.sqrt(2), 0,  1/np.sqrt(2), 0],
                          [0, 1,             0, 0],
                          [-1/np.sqrt(2), 0, -1/np.sqrt(2), 0],
                          [0, 0,             0, 1]])

    # The end-effector's standoff configuration above the cube, before and after grasping, relative
    # to the cube (the {e} frame located 0.1m above the {c} frame, rotated about the y axis):
    Tce_standoff = np.array([[-1/np.sqrt(2), 0,  1/np.sqrt(2),   0],
                             [0, 1,             0,   0],
                             [-1/np.sqrt(2), 0, -1/np.sqrt(2), 0.1],
                             [0, 0,             0,   1]])

    # The fixed offset from the chassis frame {b} to the base frame of the arm {0}:
    Tb0 = np.array([[1, 0, 0, 0.1662],
                    [0, 1, 0,      0],
                    [0, 0, 1, 0.0026],
                    [0, 0, 0,      1]])

    # The end-effector frame {e} relative to the arm base frame {0}, when the arm is at its home configuration:
    M0e = np.array([[1, 0, 0,  0.033],
                    [0, 1, 0,      0],
                    [0, 0, 1, 0.6546],
                    [0, 0, 0,      1]])

    # The screw axes B for the 5 joints expressed in the end-effector frame {e}, when the arm is at its home configuration:
    Blist = np.array([[0,  0, 1,       0, 0.0330, 0],
                      [0, -1, 0, -0.5076,      0, 0],
                      [0, -1, 0, -0.3526,      0, 0],
                      [0, -1, 0, -0.2176,      0, 0],
                      [0,  0, 1,       0,      0, 0]]).T

    # Initialization of feedback control constants:
    #  new task
    # kp_gain = 2.5						# The kp gain
    # ki_gain = 0.1			    	# The ki gain


    # # # best
    kp_gain = 5.5						# The kp gain
    ki_gain = 0.2
    # # overshoot 
    # kp_gain =0.8			# The kp gain
    # ki_gain = 1
   
    Kp = np.eye(6) * kp_gain	    # The P gain matrix
    Ki = np.eye(6) * ki_gain		# The I gain matrix

    # Restrictions on the speeds vector:
    max_ang_speed = 10

    # Initialization of simulation constants:
    k =1								# The number of trajectory reference configurations per 0.01 seconds
    delta_t = 0.01						# Time step [sec]
    t_total = 30				# Simulation run time [sec]
    iteration = int(t_total/delta_t)  # Number of iterations

    # Initialization of variable lists:
    config_array = np.zeros((iteration, 13))
    Xerr_array = np.zeros((iteration, 6))
    error_integral = np.zeros(6)

    # Add the initial configuration to the config_array:
    config_array[0] = initial_config

    ########## Calculating Configurations ##########

    # Trajectory Generation:
    logging.info('Generating trajectory')
    trajectory = TrajectoryGenerator(
        Tse_initial, Tsc_initial, Tsc_goal, Tce_grasp, Tce_standoff, k)

    # print(len(trajectory) )
    for i in range(1, iteration-1):
        print('iteration no',i)
        current_config = config_array[i-1, :]
        theta=current_config[0]
        x=current_config[1]
        y=current_config[2]
        z= 0.0963

        # Define transformation matrices to hepl find the current, desired and next desired position:
        Tsb = np.array([[cos(theta), -sin(theta), 0, x],
                        [sin(theta),  cos(theta), 0, y],
                        [0,                0,     1, z],
                        [0,                0,     0, 1]])

        theta_list=current_config[3:8]
        T0e = mr.FKinBody(M0e, Blist,theta_list)
        Tbe = Tb0.dot(T0e)
        # Define current, desired and next desired position:
        X = Tsb.dot(Tbe)
        # print("X: ", X)

        Xd = np.array([[trajectory[i][0], trajectory[i][1], trajectory[i][2],  trajectory[i][9]],
                       [trajectory[i][3], trajectory[i][4],trajectory[i][5], trajectory[i][10]],
                       [trajectory[i][6], trajectory[i][7],trajectory[i][8], trajectory[i][11]],
                       [0,                0,                0,                1]])
        # print("Xd: ", Xd)

        Xd_next = np.array([[trajectory[i+1][0], trajectory[i+1][1], trajectory[i+1][2],  trajectory[i+1][9]],
                            [trajectory[i+1][3], trajectory[i+1][4],trajectory[i+1][5], trajectory[i+1][10]],
                            [trajectory[i+1][6], trajectory[i+1][7],trajectory[i+1][8], trajectory[i+1][11]],
                            [0,                  0,                  0,                  1]])
        # print("Xd_next: ", Xd_next)

        # Calculate the control law:
        logging.info('Feedback control ')
        V, controls, Xerr, error_integral = FeedbackControl(X, Xd, Xd_next, Kp, Ki, delta_t, current_config, error_integral)

         # Flip command speeds for the NextState
        w_control = controls[:4]
        a_control = controls[4:9]
        controls_flipped = np.concatenate(
        (a_control, w_control), axis=None)
        # Calculate the next configuration:
        logging.info('getting Nextstate')
        current_config = NextState(
            current_config[:12], controls_flipped, delta_t, max_ang_speed)
        config_array[i] = np.concatenate(
            (current_config, trajectory[i][12]), axis=None)

        # Calculate the error:
        Xerr_array[i-1] = Xerr

        
    # Save the configurations as a csv file:
    logging.info('Generating trajectory csv file')
    with open("config.csv", "w+") as my_csv1:
        csvWriter = csv.writer(my_csv1, delimiter=',')
        csvWriter.writerows(config_array)

    # Save the error as a csv file:
    logging.info('Generating error csv file')
    with open("Xerr.csv", "w+") as my_csv2:
        csvWriter = csv.writer(my_csv2, delimiter=',')
        csvWriter.writerows(Xerr_array)

    plot_data(Xerr_array,kp_gain,ki_gain)
    


if __name__ == '__main__':
    main()


