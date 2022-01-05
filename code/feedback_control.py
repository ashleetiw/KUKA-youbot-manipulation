import modern_robotics as mr
import numpy as np
import csv

'''
 To run the file : python code/feedback_control.py 
    Results for the tests are in testing_milestone_results/milestone 3
'''

"""
Author:ashleetiwari2021@u.northwestern.edu
Code for Milestone 3: Feedback Control
"""


def testjointlimit(joint,value):
  if int(joint)+1==3 and value>-0.2:
           return joint,True 

  if int(joint)+1=='4' and value>-0.2:
          return joint,True

  return 0,False
    
def FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, curr_config, err_integral):
    """ calculates the kinematic task-space feedforward plus feedback control law
    Input:
      X - The current actual end-effector configuration (Tse).
      Xd - The current end-effector reference configuration (Tse_d).
      Xd_next - The end-effector reference configuration at the next timestep in the reference trajectory (Tse_d_next).
      KP - the P gain matrix.
      KI - the I gain matrix.
      dt - The time step delta_t between reference trajectory configurations.
      curr_config - The current configuration.
    Return: 
      V - The commanded end-effector twist, expressed in the end-effector frame {e}.
      controls - The commanded wheel and arm joint controls.
      Xerr - The error.
      err_integral - The error integral.
    """

    # forward-backward distance between the wheels to frame {b} 
    l = 0.47/2
    # side-to-side distance between the wheels to frame {b} 
    w = 0.3/2
    # radius of each wheel 			
    r = 0.0475			

    # fixed offset from the chassis frame {b} to the base frame of the arm {0}
    T_b0 = np.array([[1, 0, 0, 0.1662],
                    [0, 1, 0,      0],
                    [0, 0, 1, 0.0026],
                    [0, 0, 0,      1]])

    # end-effector frame {e} relative to the arm base frame {0}, when the arm is at its home configuration
    M_0e = np.array([[1, 0, 0,  0.033],
                    [0, 1, 0,      0],
                    [0, 0, 1, 0.6546],
                    [0, 0, 0,      1]])

    # screw axes B for the 5 joints expressed in the end-effector frame {e}, when the arm is at its home configuration
    Blist = np.array([[0,  0, 1,       0, 0.0330, 0],
                      [0, -1, 0, -0.5076,      0, 0],
                      [0, -1, 0, -0.3526,      0, 0],
                      [0, -1, 0, -0.2176,      0, 0],
                      [0,  0, 1,       0,      0, 0]]).T

    # current arm joint angles:
    arm_joints = curr_config[3:8]    

    T_0e = mr.FKinBody(M_0e, Blist, arm_joints)

    T_eb= (mr.TransInv(T_0e)).dot(mr.TransInv(T_b0))

    time=1/dt

    mat_log=mr.MatrixLog6((mr.TransInv(Xd)).dot(Xd_next))*time

    # calculate the feedforward reference twist
    Vd = mr.se3ToVec(mat_log)
    print("Vd: ", Vd)

    # calculate the Adx-1xd matrix
    ADx1xd = mr.Adjoint((mr.TransInv(X)).dot(Xd))
    ADx1xdVd = ADx1xd.dot(Vd)
    print("ADx1xdVd: ", ADx1xdVd)

    Xerr = mr.se3ToVec(mr.MatrixLog6((mr.TransInv(X)).dot(Xd)))
    print("Xerr: ", Xerr)

    # Calculate command end effector twist (when the numerical integral of the error is err_integral + Xerr * dt)
    err_integral += Xerr * dt
    V = ADx1xdVd + Kp.dot(Xerr) + Ki.dot(err_integral)
    print("V: ", V)

    F = r/4 * np.array([[0,         0,         0,          0],
                        [0,         0,         0,          0],
                        [-1/(l + w), 1/(l + w), 1/(l + w), -1/(l + w)],
                        [1,         1,          1,         1],
                        [-1,         1,         -1,         1],
                        [0,         0,          0,        0]])
    
    # print('..................................................',arm_joints)

    # col,bool=testjointlimit('2',arm_joints[2])         # joint3
    # if bool ==True:
    #   arm_joints[2]=-0.2

    # col,bool=testjointlimit('3',arm_joints[3])           # joint 4
    # if bool ==True: 
    #   arm_joints[3]=-0.2
    

    Ja = mr.JacobianBody(Blist, arm_joints)
    Jb = (mr.Adjoint(T_eb)).dot(F)
    Je = np.concatenate((Jb, Ja), axis=1)
    print("Je: ", Je)
    Je_inv = np.linalg.pinv(Je,rcond=1e-3)
    # Je_inv = np.linalg.pinv(Je)
    controls = Je_inv.dot(V)
    print("controls: ", controls)

    return V, controls, Xerr, err_integral


# # ########## Testing ##########

# curr_config = np.array([0, 0, 0, 0, 0, 0.2, -1.6, 0, 0, 0, 0, 0, 0])

# # actual end-effector configuration
# X = np.array([[0.170, 0, 0.985, 0.387],
#               [0, 1,     0,     0],
#               [-0.985, 0, 0.170, 0.570],
#               [0, 0,     0,     1]])

# # end-effector reference configuration
# Xd = np.array([[0, 0, 1, 0.5],
#                [0, 1, 0,   0],
#                [-1, 0, 0, 0.5],
#                [0, 0, 0,   1]])

# # end-effector reference configuration at the next timestep in the reference trajector
# Xd_next = np.array([[0, 0, 1, 0.6],
#                     [0, 1, 0,   0],
#                     [-1, 0, 1, 0.3],
#                     [0, 0, 0,   1]])

# err_integral = np.zeros(6)

# KP_gain = 0						
# KI_gain = 0						
# KP = np.identity(6) * KP_gain   
# KI = np.identity(6) * KI_gain   
# dt = 0.01	  			     

# V, speeds, Xerr, err_integral = FeedbackControl(
#     X, Xd, Xd_next, KP, KI, dt, curr_config, err_integral)



# with open("Xerr.csv", "w+") as my_csv:
#     csvWriter = csv.writer(my_csv, delimiter=',')
#     csvWriter.writerows(Xerr)

# with open("controls.csv", "w+") as my_csv:
#     csvWriter = csv.writer(my_csv, delimiter=',')
#     csvWriter.writerows(speeds)


