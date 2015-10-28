# -*- coding: utf-8 -*-
"""
Useful Whole-body IK functions for DRC-Hubo model
Created on Tue Jul 30 10:59:00 2013

@author: Andy Park (andypark@purdue.edu)
@copyright: Andy Park
"""

## 07-29-13 Correct the CoM position fixing hand pos/orientation
def WB_CoM_correction(q_data_init):
    
    import time
    import robot
    from copy import copy, deepcopy
    from construct_DRC_Hubo import *
    from motion2ach_update import *
    #from Ani_Robot import *
#    from motion2ach import *

    robot = construct_DRC_Hubo()
    q_data = mat(zeros((robot.NJ,1)))
    q_data_zero = deepcopy(q_data)
    
#    q_data_init = q_data
#    q_data_init[robot.J_HPY-1] = 0
#    q_data_init[robot.J_LHP-1] = -pi/8
#    q_data_init[robot.J_LKP-1] = pi/4
#    q_data_init[robot.J_LAP-1] = -pi/8
#    q_data_init[robot.J_RHP-1] = -pi/8
#    q_data_init[robot.J_RKP-1] = pi/4
#    q_data_init[robot.J_RAP-1] = -pi/8
#    
#    q_data_init[robot.J_RSP-1] = 0.5
#    q_data_init[robot.J_RSR-1] = -0.2618
#    q_data_init[robot.J_RSY-1] = 0
#    q_data_init[robot.J_REP-1] = -1.5
#    q_data_init[robot.J_RWY-1] = 0
#    q_data_init[robot.J_RWP-1] = 0
#    q_data_init[robot.J_RWR-1] = -1.5708
#    
#    q_data_init[robot.J_LSP-1] = 0.5
#    q_data_init[robot.J_LSR-1] = 0.2618
#    q_data_init[robot.J_LSY-1] = 0
#    q_data_init[robot.J_LEP-1] = -1.5
#    q_data_init[robot.J_LWY-1] = 0
#    q_data_init[robot.J_LWP-1] = 0
#    q_data_init[robot.J_LWR-1] = 1.5708
    
    q_init = q_data_init
    
    ## Find Init Pose - 90 degree wrist position 
    X_offset = 0.001*0
    Y_offset = -0.001*0
    Z_offset = 0.001*0
    axis_rotation = [0, 1, 0]
    angle = deg2rad(0)
    
    Kp = 100
    tsamp = 0.01
    err_thresh = 0.001
    
    ContactState = robot.LF
    F_swF = robot.F_RF
    
    F_RH = robot.F_RH
    F_LH = robot.F_LH
    
    q_tmp = q_data_init
    #q = q_tmp

    # right hand pose
    T_RH_ref = robot.FK_tree(q_tmp, robot.F_RH, ContactState)
    T_RH_init = T_RH_ref
#    R_RH_ref = t2r(T_RH_ref)
#    R_RH_ref = rotvec2r(angle,axis_rotation)
    T_RH_ref[0,3] = T_RH_ref[0,3] + X_offset
    T_RH_ref[1,3] = T_RH_ref[1,3] + Y_offset
    T_RH_ref[2,3] = T_RH_ref[2,3] + Z_offset
#    T_RH_ref[0:3,0:3] = R_RH_ref
    x_RH_ref = tr2diff(T_RH_ref)
    
    # left hand pose
    T_LH_ref = robot.FK_tree(q_tmp, robot.F_LH, ContactState)
    T_LH_init = T_LH_ref
#    R_LH_ref = t2r(T_LH_ref)
#    R_LH_ref = rotvec2r(angle,axis_rotation)
    T_LH_ref[0,3] = T_LH_ref[0,3] + X_offset
    T_LH_ref[1,3] = T_LH_ref[1,3] + Y_offset
    T_LH_ref[2,3] = T_LH_ref[2,3] + Z_offset
#    T_LH_ref[0:3,0:3] = R_LH_ref
    x_LH_ref = tr2diff(T_LH_ref)
    
    # waist pose
    T_W_ref = robot.FK_tree(q_tmp, robot.F_HPY, ContactState)
    T_W_init = T_W_ref
    T_W_ref[0,3] = T_W_ref[0,3]# + 0.001*50;
    T_W_ref[1,3] = T_W_ref[1,3]
    T_W_ref[2,3] = T_W_ref[2,3]
    x_W_ref = tr2diff(T_W_ref)
    
    # swing foot pose
    T_swF_ref = robot.FK_tree(q_tmp, F_swF, ContactState)
    T_swF_init = T_swF_ref
    R_swF_ref = t2r(T_swF_ref)
    R_swF_ref = rotvec2r(angle,axis_rotation)
    T_swF_ref[0,3] = T_swF_ref[0,3] + X_offset
    T_swF_ref[1,3] = T_swF_ref[1,3] + Y_offset
    T_swF_ref[2,3] = T_swF_ref[2,3] + Z_offset
    T_swF_ref[0:3,0:3] = R_swF_ref
    x_swF_ref = tr2diff(T_swF_ref)
    
    # CoM location
    x_com_ref = robot.CoM(q_data_zero, ContactState)
    x_com_ref[0,0] = x_com_ref[0,0]
    x_com_ref[0,1] = x_com_ref[0,1]
    x_com_ref[0,2] = x_com_ref[0,2]
    
    q_tmp_init = q_tmp    
    
    error = 10000
    cnt = 1
    
    #tic = time.clock()
    while norm(error) > err_thresh:
        x_W = tr2diff(robot.FK_tree(q_tmp, robot.F_HPY, ContactState))
        err_x_W = x_W_ref - x_W
        err_x_W = err_x_W[3:]
        
        x_RH = tr2diff(robot.FK_tree(q_tmp, robot.F_RH, ContactState))
        err_x_RH = x_RH_ref - x_RH
        
        x_LH = tr2diff(robot.FK_tree(q_tmp, robot.F_LH, ContactState))
        err_x_LH = x_LH_ref - x_LH
        
        x_swF = tr2diff(robot.FK_tree(q_tmp, F_swF, ContactState))
        err_x_swF = x_swF_ref - x_swF

        J_W = robot.J_tree(q_tmp, robot.F_HPY, ContactState)
        J_W = J_W[3:,]        
        
        x_com = robot.CoM(q_tmp, ContactState)
        err_x_com = x_com_ref.T - x_com.T
            
        J_RH = robot.J_tree(q_tmp, F_RH, ContactState)
        J_RH = J_RH

        J_LH = robot.J_tree(q_tmp, F_LH, ContactState)
        J_LH = J_LH
        
        J_swF = robot.J_tree(q_tmp, F_swF, ContactState)
        J_swF = J_swF
        
        J_com = robot.J_com_tree(q_tmp, ContactState)
        J_com_legs = hstack((zeros((3,1)), J_com[:,robot.J_LHY-1:robot.J_RAR], zeros((3,14))))              
        
#        Jacob = vstack((J_RH))
#        error = vstack((err_x_RH))
        
        Jacob = vstack((J_LH,J_RH,J_com_legs[:2,],J_W,J_swF))
        error = vstack((err_x_LH,err_x_RH,err_x_com[:2,],err_x_W,err_x_swF))
        
        qd_tmp = Kp*pinv(Jacob)*error
        q_tmp = q_tmp + qd_tmp*tsamp
        q_tmp = robot.joint_limit_check(q_tmp)
        
        cnt = cnt + 1
        
        #clc
        print cnt
        print norm(error)
        
        if cnt > 200:
                error = 0.0001
                q_sol = q_tmp_init
                print('solution is not found, try with different numbers')
                flag_sol = 0
            else: ## if solution is found
                flag_sol = 1
                q_sol = q_tmp
                
    return q_sol