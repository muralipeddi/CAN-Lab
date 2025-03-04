#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 12 15:28:28 2023

@author: sayanchakraborty
"""

import time as tt
import os
import sys
import optparse
import subprocess
import random
import math
# import matplotlib.pyplot as plt
# from scipy.linalg import expm
from numpy.linalg import inv
# import traci
import numpy as np 
import numpy.matlib
# from scipy.io import loadmat
from numpy import linalg as LA
# from scipy.integrate import solve_ivp
# from scipy.integrate import odeint

def learning_K(K, Dxx, Ixx, Ixu, Iuu):
    
    epsi = 1e-4;
    it = 0; 
    POld = np.zeros(4)
    KOld = np.zeros([1,4])
    R = 1
    Q = 100*np.diag([1.0, 1.0, 1.0, 1.0])
   
    learning_start_time = tt.time()
    while (LA.norm(K-KOld) > epsi) and it<100:
        it = it + 1; 
        KOld = K
        Qk = Q + np.matmul(K.T*R,K)
        # Qk = Q + K.T@R@K
        # Theta = np.append(Dxx, -2*np.matmul(Ixx,np.kron(np.eye(4),K.T))-2*Ixu, axis=-1)
        Theta = np.append( np.append(Dxx, -2*np.matmul(Ixx,np.kron(np.eye(4),K.T))-2*Ixu, axis=-1) , np.matmul(Ixx,np.kron(K.T,K.T))-Iuu, axis=-1)
        # Theta = np.append( np.append(Dxx, -2*Ixx@np.kron(np.eye(4),K.T)-2*Ixu, axis=-1) , Ixx@np.kron(K.T,K.T)-Iuu, axis=-1)
        # Theta = np.array(Theta, dtype='float')
        # print("ThetaShape = ", Theta.shape)
        QkVec = Qk.reshape(16,1)
        Xi = np.matmul(-Ixx,QkVec);
        # Xi = -Ixx@QkVec;
        pp = np.matmul(LA.pinv(Theta),Xi); 
        # pp = LA.pinv(Theta)@Xi; 
        # pp = (LA.inv(Theta.T@Theta)@Theta.T)@Xi; 
        
        # print("pp = ", pp)
        P = np.array([[pp[0], pp[1]/2, pp[2]/2, pp[3]/2], 
           [pp[1]/2, pp[4], pp[5]/2, pp[6]/2],
           [pp[2]/2, pp[5]/2, pp[7], pp[8]/2],
           [pp[3]/2, pp[6]/2, pp[8]/2, pp[9]]])
        P =  P.reshape(4,4)
        # print("P = ", P.reshape(4,4))
        
        BPA = pp[10:14].reshape(1,4)
        # print("BPA = ", BPA)
        BPB = pp[14]
        # print("BPB = ", BPB)
        K = np.matmul(1/(R+BPB),BPA)
        K = K.reshape(1,4)
        # print("K = ", K)
        POld = P  
    learning_end_time = tt.time()
    print('learning iteration:',it, 'rank Theta=', LA.matrix_rank(Theta))
    print('learning time:', learning_end_time-learning_start_time)
   
    learningTime = learning_end_time-learning_start_time
    return P,K, learningTime


def model(x, k, K, ifLEarned):
    
    Ad,Bd = getAB()
    # x = x[0:4]
    if ifLEarned == 0:
        # ExpNoiseFreq = loadmat('ExpNoiseFreq1.mat')
        # ExpNoiseFreq = ExpNoiseFreq['expl_noise_freq']
        # ExpNoise = sum(np.sin(ExpNoiseFreq[0]*(k)))
        ExpNoise = np.sin(10*k)
        # u = -K@x+ExpNoise
        u = -np.matmul(K,x)+ExpNoise
    else:
        # u = -K@x
        u = -np.matmul(K,x)

    # xn = Ad@x.reshape(4,1)+Bd*u
    xn = np.matmul(Ad,x.reshape(4,1))+Bd*u
    
    return xn,u


def getAB():
    Vx = 40
    T=0.125;  # length of each learning time interval
    # define parameters
    # Vx = 20; #in cm
    
    slipAng = 0.436332; # 25deg
    
    m = 4000.0; # in g
    Iz = 454.0*1e7; # g.cm^2
    lf = 15.0; # cm
    lr = 15.0; #cm
    Fyr = ((m*(Vx*Vx))/1.485)*(lf/(lr+lf)); #Fyr = ((m*V^2)/R))*(lf/(lr+lf))
    Fyf = Fyr*(lr/lf); 
    Cf = Fyf/slipAng;
    Cr = Fyr/slipAng;
    
    A = np.array([[ 0, 1, 0, 0 ],
                            [ 0, -(1/(m*Vx))*(2*Cf+2*Cr), (2*Cf+2*Cr)/(m), -(2*Cf*lf-2*Cr*lr)/(m*Vx) ],
                            [ 0, 0, 0, 1],
                            [ 0, -(1/(Iz*Vx))*(2*lf*Cf-2*lr*Cr), (2*Cf*lf-2*Cr*lr)/Iz, -(1/(Iz*Vx))*(2*lf*lf*Cf+2*lr*lr*Cr)]])
    B = np.array([[0],[ 2*Cf/m],[ 0], [2*lf*Cf/Iz]])
    
    n = 4; # size of state
    m = 1; # size of input
    r = 1; # size of output
    
    Ad = (np.eye(4)+A*T)
    Bd = B*T
    return Ad, Bd

def mainFun():
    x0 = np.array([[1],[0],[1],[0]])
    K = np.array([[0.0046,   -0.0450,    1.9931,    0.0026]])
    print(K.shape)
    Dxx = np.zeros((1,16))
    Ixx = np.zeros((1,16))
    Ixu = np.zeros((1,4))
    Iuu = np.zeros((1,1))
    for k in range(100):
        x,u = model(x0,k, K, 0)
        print('u = ', u)
       	print('x0 =', x0)
        b0 = np.kron(x.T,x.T)-np.kron(x0.T,x0.T)
        Dxx = np.append(Dxx, b0, axis=0)
        
        b1 =  np.kron(x0.T,x0.T)
        Ixx = np.append(Ixx, b1, axis=0)
        
        b2 =  np.kron(x0.T,u.T)
        Ixu = np.append(Ixu, b2, axis=0)
        
        b3 =  np.kron(u.T,u.T)
        Iuu = np.append(Iuu, b3, axis=0)
        x0 = x
         
    # DxxU = np.unique(np.round(Dxx,4), axis=1)
    DxxU = Dxx[1:-1,[0,1,2,3,5,6,7,10,11,15]]
    IuuU = Iuu[1:-1,:]
    Ixx = Ixx[1:-1,:]
    Ixu = Ixu[1:-1,:]
    # from scipy.io import savemat
    # savemat("DataMatrices.mat", {"DxxU":DxxU,"Dxx":Dxx,"IuuU":IuuU,"Ixx":Ixx,"Ixu":Ixu})
    # print(DxxU)
    print(DxxU.shape, Ixx.shape, IuuU.shape, Ixu.shape)
    print(LA.matrix_rank(DxxU), LA.matrix_rank(Ixx), LA.matrix_rank(IuuU), LA.matrix_rank(Ixu))
    Pn, Kn, learningTime = learning_K(K, DxxU, Ixx, Ixu, IuuU)

    # learn using data from MATLAB
    # Data = loadmat('DataFromMATLAB.mat')
    # Pn, Kn, learningTime = learning_K(K, Data['Delta_een'], Data['I_ee'], Data['I_eu'], Data['I_uu'])
    
    
    # print(Pn)
    # print(Kn)
    return Pn, Kn

Pn, Kn = mainFun()
print('P = ', Pn)
print('K = ', Kn)




