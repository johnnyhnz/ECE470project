#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm, logm

import project_header 
"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
   # =================== Your code starts here ====================#
   # Fill in the correct values for w1~6 and v1~6, as well as the M matrix
   M = np.eye(4)
   #S = np.zeros((6,6))
   M = np.array([[0,-1,0,.390],
   [0,0,-1,.401],
   [1,0,0,.2155],
   [0,0,0,1] ])
   w1 = np.array([0,0,1])
   q1 = np.array([-.150,.150,.162])
   v1 = np.cross(-w1,q1)
   S1 = np.array([[0,-1,0,v1[0]],
   [1,0,0,v1[1]],
   [0,0,0,v1[2]],
   [0,0,0,0]])
 
   w2 = np.array([0,1,0])
   q2 = np.array([-.150,.270,.162])
   v2 = np.cross(-w2,q2)
   S2 =np.array([[0,0,1,v2[0]],
   [0,0,0,v2[1]],
   [-1,0,0,v2[2]],
   [0,0,0,0]])
 
   w3 = np.array([0,1,0])
   q3 = np.array([.094,.270,.162])
   v3 = np.cross(-w3,q3)
   S3 =np.array( [[0,0,1,v3[0]],
   [0,0,0,v3[1]],
   [-1,0,0,v3[2]],
   [0,0,0,0]])
 
   w4 = np.array([0,1,0])
   q4 = np.array([.307,.177,.162])
   v4 = np.cross(-w4,q4)
   S4 = np.array([[0,0,1,v4[0]],
   [0,0,0,v4[1]],
   [-1,0,0,v4[2]],
   [0,0,0,0]])
 
   w5 = np.array([1,0,0])
   q5 = np.array([.307,.260,.162])
   v5 = np.cross(-w5,q5)
   S5 = np.array([[0,0,0,v5[0]],
   [0,0,-1,v5[1]],
   [0,1,0,v5[2]],
   [0,0,0,0]]  )
 
   w6 = np.array([0,1,0])
   q6 = np.array([.390,.260,.162])
   v6 = np.cross(-w6,q6)
   S6 = np.array([[0,0,1,v6[0]],
   [0,0,0,v6[1]],
   [-1,0,0,v6[2]],
   [0,0,0,0]])
 
   S = np.array([S1,S2,S3,S4,S5,S6])
 
 
 
 
   # ==============================================================#
   return M, S
 
 
"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):
   PI = 3.1415926535
   # Initialize the return_value
   return_value = [None, None, None, None, None, None]
 
   #print("Foward kinematics calculated:\n")

 
   # =================== Your code starts here ====================#
   theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
   T = np.eye(4)
   M, S = Get_MS()
   S1 = S[0]
   S2 = S[1]
   S3 = S[2]
   S4 = S[3]
   S5 = S[4]
   S6 = S[5]
 
 
   T1 = expm(S1*theta1)
   T2 = expm(S2*theta2)
   T3 = expm(S3*theta3)
   T4 = expm(S4*theta4)
   T5 = expm(S5*theta5)
   T6 = expm(S6*theta6)
   T_1 = np.matmul(T1,T2)
   T_2 = np.matmul(T_1,T3)
   T_3 = np.matmul(T_2,T4)
   T_4 = np.matmul(T_3,T5)
   T_5 = np.matmul(T_4,T6)
   T = np.matmul(T_5,M)
  
 
 
 
 
   # ==============================================================#
 
   return_value[0] = theta1 + PI
   return_value[1] = theta2
   return_value[2] = theta3
   return_value[3] = theta4 - (0.5*PI)
   return_value[4] = theta5
   return_value[5] = theta6
 
   return return_value
 
 
"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
   # =================== Your code starts here ====================#
   xgrip = xWgrip + .15
   ygrip = yWgrip - .15
   zgrip = zWgrip - 0.01
   yawgrip_rad = yaw_WgripDegree*(np.pi/180)
 
 
   L1 = .152
   L2 = .12
   L3 = .244
   L4 = .093
   L5 = .213
   L6 = .083
   L7 = .083
   L8 = .082
   L9 = 0.0535
   L10 = .059
 
   zcen = zgrip
   xcen = xgrip - L9*np.cos(yawgrip_rad)
   ycen = ygrip - L9*np.sin(yawgrip_rad)
 
 
   phi = np.arctan2(ycen,xcen)
   dist = L2-L4+L6
   hyp = np.sqrt((ycen**2)+(xcen**2))
   alpha = np.arcsin(dist/hyp)
   theta1 = phi-alpha
 
   theta6 = np.pi/2 + theta1 - yawgrip_rad
 
 
   x3end = xcen - L7*np.cos(theta1) + (L6 + 0.027)*np.sin(theta1)
   y3end = ycen - L7*np.sin(theta1) - (L6 + 0.027)*np.cos(theta1)
   z3end = zcen + L10 + L8
  
 
   z3 = z3end - L1
   c = np.sqrt(x3end**2 + y3end**2)
   L3end = np.sqrt(c**2 + z3**2)
   theta3 = np.pi - np.arccos((L3end**2 - L3**2 - L5**2)/(-2*L3*L5))
 
   phi2 = np.arctan2(z3,c)
   beta = np.arccos((L5**2 - L3**2 - L3end**2)/(-2*L3*L3end))
   #print(beta)
   theta2 = -(beta + phi2)
 
   theta4 = -np.arcsin((-z3 + L3*np.sin(-theta2))/L5)
   theta5 = -np.pi/2
 
 
 
  
   # ==============================================================#
   return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)

