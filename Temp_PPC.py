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
        b

        rho_f, rho_y, rho_z, rho_o, rho_s

        f, f_d,
        R_s, R_t, R_d
        P_s, P_t, P_d
    """


    def __init__(self, M_f=0, M_y=1, M_z=1, M_o=1, a_f=0.04, a_y=0.004, a_z=0.004, a_o=0.002, b=50):
        self.perf = {}

        # start time, (contact moment)  t-t_s in order to get time
        self.perf['t_s'] = rospy.get_time()

        self.perf['M_f'] = M_f
        self.perf['M_y'] = M_y
        self.perf['M_z'] = M_z
        self.perf['M_o'] = M_o

        self.params = {}
        self.params['a_f'] = a_f
        self.params['a_y'] = a_y
        self.params['a_z'] = a_z
        self.params['a_o'] = a_o
        self.params['b'] = b

        self.signals = {}


    def calc_velocity(self, f_d, f_m, y_d, y_m):

        # define time and performance
        t = rospy.get_time() - self.perf['t_s']
        self.perf['rho_f'] = 9
        self.perf['rho_y'] = (0.02-0.01)*math.exp(-2*t)+0.01
        self.perf['rho_z'] = (0.02-0.01)*math.exp(-2*t)+0.01
        self.perf['rho_o'] = (0.02-0.01)*math.exp(-2*t)+0.01
        #TODO rho_s
        self.perf['rho_s'] = (0.02-0.01)*math.exp(-2*t)+0.01


        #TODO tf listener
        t = tf.Transformer(True, rospy.Duration(10.0))
        t.getFrameStrings()
        (self.signals['P_t'],self.signals['Rot_t']) = t.lookupTransform('tool0','base',rospy.Time(0))
        (self.signals['P_s'],self.signals['Rot_s']) = t.lookupTransform('start_position','tool0',rospy.Time(0))
        #TODO quaternions to matrices
        self.signals['R_t'] = quaternions.quat2mat(self.signals['Rot_t'])
        self.signals['R_s'] = quaternions.quat2mat(self.signals['Rot_s'])
        #TODO euler to matrices, y_d struktor
        self.signals['R_d'] = euler.euler2mat(y_d[3],y_d[4],y_d[5])
