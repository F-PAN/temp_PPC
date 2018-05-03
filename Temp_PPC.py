#!/usr/bin/env python

## Python ##
##
from __future__ import division
import numpy as np
import pitasc_core.pitasc_logging as logging
import math
##############################################################
#TODO import for tf
import rospy
import tf

##############################################################
#TODO quaternions to matrices
from transforms3d import quaternions
from transforms3d import euler




## piTaSC ##
##
from pitasc_core.controller import Controller
import pitasc_core.utilities as util

class Controller_PPC(Controller):
    """ Controller for prescribed performance contact establishment

        Parameters:
        M_f, M_y, M_z, M_o
        a_f, a_y, a_z, a_o
        b,

        rho_f, rho_y, rho_z, rho_o, rho_s

        f, f_d
        R_s, R_t, R_d
        P_s, P_t, P_d
    """


    def __init__(self, M_f=0, M_y=1, M_z=1, M_o=1, a_f=0.04, a_y=0.004, a_z=0.004, a_o=0.002, b=50):
        self.perf = {}

        # start time, (contact moment)  t-t_s in order to get time


        self.perf['M_f'] = M_f
        self.perf['M_y'] = M_y
        self.perf['M_z'] = M_z
        self.perf['M_o'] = M_o

        self.params = {}
        self.params['t_s'] = rospy.get_time()
        self.params['a_f'] = a_f
        self.params['a_y'] = a_y
        self.params['a_z'] = a_z
        self.params['a_o'] = a_o
        self.params['b'] = b

        self.signals = {}


    def calc_velocity(self, f_d=10, f_m, y_d, y_m, V_m): # reference force, pos and orien.; measured force, pos and orien., velocity

        # define time and performance
        t = rospy.get_time() - self.params['t_s']
        rho_f = 9
        rho_y = (0.02-0.01)*math.exp(-2*t)+0.01
        rho_z = (0.02-0.01)*math.exp(-2*t)+0.01
        rho_o = (0.02-0.01)*math.exp(-2*t)+0.01
        #TODO rho_s
        rho_s = (0.02-0.01)*math.exp(-2*t)+0.01


        #TODO tf listener
        t = tf.Transformer(True, rospy.Duration(10.0))
        t.getFrameStrings()
        (P_t, Rot_t) = t.lookupTransform('tool','base',rospy.Time(0))
        (P_s, Rot_s) = t.lookupTransform('start_position','tool',rospy.Time(0))
        #TODO quaternions to matrices
        R_t = quaternions.quat2mat(self.signals['Rot_t'])
        R_s = quaternions.quat2mat(self.signals['Rot_s'])
        #TODO euler to matrices, y_d struktor
        R_d = euler.euler2mat(y_d[3],y_d[4],y_d[5])
        P_d = np.array([y_d[0]],[y_d[1]],[y_d[2])











        ########################################################################
        ## Controller
        # split rotation matrices
        n_s, o_s, a_s = np.array_split(self.signals['R_s'],3,axis=1)
        n_t, o_t, a_t = np.array_split(self.signals['R_t'],3,axis=1)
        n_d, o_d, a_d = np.array_split(self.signals['R_d'],3,axis=1)

        # control error
        e_f = f - f_d
        e_y = float(np.transpose(o_s).dot(P_t-P_d))
        e_z = float(np.transpose(a_s).dot(P_t-P_d))
        e_o = (np.cross(n_t,n_d,axis=0) + np.cross(o_t,o_d,axis=0) + np.cross(a_t,a_d,axis=0))/2
        L = (skew(n_t).dot(skew(n_d)) + skew(o_t).dot(skew(o_d)) + skew(a_t).dot(skew(a_d)))/2

        # error transformation
        epsilon_f = T(e_f,rho_f,self.perf['M_f'])
        epsilon_y = T(e_y,rho_y,self.perf['M_y'])
        epsilon_z = T(e_z,rho_z,self.perf['M_z'])
        epsilon_o = np.array([[T(e_o[0][0],rho_o[0][0],self.perf['M_o'])], [T(e_o[1][0],rho_o[1][0],self.perf['M_o'])], [T(e_o[2][0],rho_o[2][0],self.perf['M_o'])]])

        # derivative of error transformation
        Psi_f = Psi(e_f,rho_f,self.perf['M_f'])
        Psi_y = Psi(e_y,rho_y,self.perf['M_y'])
        Psi_z = Psi(e_z,rho_z,self.perf['M_z'])
        Psi_o = np.diag((Psi(e_o[0][0],rho_o[0][0],self.perf['M_o']),Psi(e_o[1][0],rho_o[1][0],self.perf['M_o']),Psi(e_o[2][0],rho_o[2][0],self.perf['M_o'])))

        # reference velocity in surface frame
        Vr1 = np.array([[-a_f*Psi_f*epsilon_f], [-a_y*Psi_y*epsilon_y],[-a_z*Psi_z*epsilon_z]])
        Vr2 = -a_o * np.linalg.inv(L).dot(Psi_o).dot(epsilon_o)
        Vr = np.concatenate((Vr1, Vr2), axis=0)

        # reference velocity in base frame
        V_d = np.linalg.inv(R_t).dot(np.linalg.inv(R_s)).dot(Vr)

        #

        return
    #########################################################################################################
    # tranform vector to skew symmetric matrix
    def skew(v):
        return np.array([0,-v[2][0],v[1][0],v[2][0],0,-v[0][0],-v[1][0],v[0][0],0]).reshape(3,3)

    # error transformation
    def T(e,rho,M):
        a = e/rho
        if e >= 0:
            return float(np.log((M+a)/(1-a)))
        else:    #e<=0?
            return float(np.log((1+a)/(M-a)))

    # derivative of error transformation
    def Psi(e,rho,M):
        a = e/rho
        if e >= 0:
            return float(((1/(M+a)+1/(1-a))/rho))
        else:    #e<=0?
            return float(((1/(1+a)+1/(M-a))/rho))
