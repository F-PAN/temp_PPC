#!/usr/bin/env python

## Python ##
##
from __future__ import division
import numpy as np
import pitasc_core.pitasc_logging as logging
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


    def __init__(self, rho_f, rho_y, rho_z, rho_o, rho_s, M_f, M_y, M_z, M_o, a_f, a_y, a_z, a_o, b):
        self.perf = {}
        self.perf['rho_f'] = rho_f
        self.perf['rho_y'] = rho_y
        self.perf['rho_z'] = rho_z
        self.perf['rho_o'] = rho_o
        self.perf['rho_s'] = rho_s
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
