#! /usr/bin/env python3
import numpy as np

#Physical Parameters in SI UNTIS
J = np.zeros((3,3)) #Inertia Matrix
J[0][0] = 0.0380847    #Ixx
J[1][1] = 0.0260876168 #Iyy
J[2][2] = 0.068123168  #Izz

l = 0.24703 #Arm length in meters

J_r = 0.013 #Assuming propellers are discs with evenly distributed mass
