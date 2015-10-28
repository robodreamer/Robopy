# -*- coding: utf-8 -*-
"""
Created on Fri Jul 19 15:45:09 2013

@author: Andy Park
"""
select = 7

## 07-22-13 basic setting and test
# testing 07-18-13 Test Wheel Turning
if select == 1:
    
    import time
    import robot
    from copy import copy, deepcopy
    from construct_DRC_Hubo import *
    from motion2ach_update import *
    #from Ani_Robot import *
#    from motion2ach import *

    
    
    robot = construct_DRC_Hubo()
    q_data = mat(zeros((robot.NJ,1)))
    
    q_data_init = q_data
    q_data_init[robot.J_HPY-1] = 0
    q_data_init[robot.J_LHP-1] = -pi/8
    q_data_init[robot.J_LKP-1] = pi/4
    q_data_init[robot.J_LAP-1] = -pi/8
    q_data_init[robot.J_RHP-1] = -pi/8
    q_data_init[robot.J_RKP-1] = pi/4
    q_data_init[robot.J_RAP-1] = -pi/8
    q_data_init[robot.J_RSP-1] = 0.36
    q_data_init[robot.J_RSR-1] = -0.2618
    q_data_init[robot.J_RSY-1] = 0
    q_data_init[robot.J_REP-1] = -1.9599
    q_data_init[robot.J_RWY-1] = 0
    q_data_init[robot.J_RWP-1] = 0
    q_data_init[robot.J_RWR-1] = -1.5708
    q_tmp = q_data_init
    q_init = q_data_init
    
    ## Find Init Pose - 90 degree wrist position 
    X_offset = 0.001*0
    Y_offset = -0.001*50
    Z_offset = 0.001*0
    axis_rotation = [0, 1, 0]
    angle = deg2rad(-90)
    
    Kp = 100
    tsamp = 0.01
    err_thresh = 0.001
    
    ContactState = robot.BODY
    
    F_RH = robot.F_RWP
    

    #q = q_tmp
    
    # right hand pose
    T_RH_ref = robot.FK_tree(q_tmp, F_RH, ContactState)
    T_RH_init = T_RH_ref
    R_RH_ref = t2r(T_RH_ref)
    R_RH_ref = rotvec2r(angle,axis_rotation)
    T_RH_ref[0,3] = T_RH_ref[0,3] + X_offset
    T_RH_ref[1,3] = T_RH_ref[1,3] + Y_offset
    T_RH_ref[2,3] = T_RH_ref[2,3] + Z_offset
    T_RH_ref[0:3,0:3] = R_RH_ref
    x_RH_ref = tr2diff(T_RH_ref)
    
    error = 10000
    cnt = 1
    
    #tic = time.clock()
    while norm(error) > err_thresh:
        x_RH = tr2diff(robot.FK_tree(q_tmp, F_RH, ContactState))
        err_x_RH = x_RH_ref - x_RH
        
        J_RH = robot.J_tree(q_tmp, F_RH, ContactState)
        J_RH = J_RH
        Jacob = J_RH
        error = err_x_RH
        
        qd_tmp = Kp*pinv(Jacob)*error
        q_tmp = q_tmp + qd_tmp*tsamp
        q_tmp = robot.joint_limit_check(q_tmp)
        
        cnt = cnt + 1
        
        #clc
        #print cnt
        #print norm(error)
    #toc = time.clock()
    
    #print toc - tic
    
    q_data_init = q_tmp
    ## generate init pose
    # from zero configuration to init pose
    (Q, Qd, Qdd) = jtraj(zeros((1,robot.NJ)),q_data_init,60)
    #Q = Qdata[0] # joint position trajectory
    qdata = Q
    #qdata1 = spline_interp(qdata, 100)    
    #moveRobot(qdata1,0.00)    
    
    ## Generate Steering Wheel Rotating motion
    Wheel_diameter = 0.001*330.02
    Wheel_thickness = 0.001*25.4
    Wheel_radius = (Wheel_diameter - Wheel_thickness/2)/2
    
    Wheel_rotation = deg2rad(90)
    
    N_hand_step = 30
    
    # right hand
    T_RH_ref = robot.FK_tree(q_tmp, F_RH, ContactState)
    T_RH_init = T_RH_ref
    
    ang_data = CosTraj(N_hand_step, 0, Wheel_rotation)
    
    # generate right hand trajectory
    Tdata_RH = zeros((4,4,N_hand_step))
    
    for I in range(N_hand_step):
        T_RH_tmp = deepcopy(T_RH_init)
        ang = ang_data[0,I]
        T_RH_tmp[1,3] = T_RH_init[1,3] + (Wheel_radius - Wheel_radius*cos(ang))
        T_RH_tmp[2,3] = T_RH_init[2,3] + Wheel_radius*sin(ang)
        Tdata_RH[:,:,I] = T_RH_tmp
    
    Nframes = N_hand_step
    
    err_thresh = 0.001
    #qdata = vstack((qdata,q_tmp.T))
    
    for I in range(Nframes):
        T_RH_ref = Tdata_RH[:,:,I]
        x_RH_ref = tr2diff(T_RH_ref)
        
        error = 10000
        cnt = 1
        while norm(error) > err_thresh:
            x_RH = tr2diff(robot.FK_tree(q_tmp, F_RH, ContactState))
            err_x_RH = x_RH_ref - x_RH
    
            J_RH = robot.J_tree(q_tmp, F_RH, ContactState)
            Jacob = J_RH
            error = err_x_RH
            
            qd_tmp = Kp*pinv(Jacob)*error
            q_tmp = q_tmp + qd_tmp*tsamp
            q_tmp = robot.joint_limit_check(q_tmp)
    
            cnt = cnt + 1
            
            #print I
            #print cnt
            #print norm(error)
        q_tmp[robot.J_RWR-1,0] = q_data_init[robot.J_RWR-1,0] + ang_data[0,I]
        qdata = vstack((qdata,q_tmp.T))
    

    
    ## generate end pose
    (Q, Qd, Qdd) = jtraj(qdata[-1:,:],zeros((1,robot.NJ)),60)
    #Q = Qdata[0] # joint position trajectory
    qdata = vstack((qdata,Q))
    
    ## interpolation
    qdata1 = spline_interp(qdata, 100)    
    #print qdata1
    
    ## send the motion to Hubo-Ach
    print "sending data to Hubo-Ach"
    #moveRobot(qdata1,0.003)    
    
    ## plot qdata
    #robot.Ani_robot(qdata,2)
    

## 07-26-13 Static Backward Walking Example
if select == 2:
    import time
    #import robot
    from copy import copy, deepcopy
    from construct_DRC_Hubo_fast import *
    from motion2ach_update import *
    #from Ani_Robot import *
#    from motion2ach import *
    
    robot = construct_DRC_Hubo()
    q_data = mat(zeros((robot.NJ,1)))
    
    q_data_init = q_data
    q_data_init[robot.J_HPY-1] = 0
    q_data_init[robot.J_LHP-1] = -pi/8
    q_data_init[robot.J_LKP-1] = pi/4
    q_data_init[robot.J_LAP-1] = -pi/8
    q_data_init[robot.J_RHP-1] = -pi/8
    q_data_init[robot.J_RKP-1] = pi/4
    q_data_init[robot.J_RAP-1] = -pi/8
    q_init = q_data_init
    
    q_data_init[robot.J_LEP-1] = -3*pi/4
    q_data_init[robot.J_REP-1] = -3*pi/4
    q_data_init[robot.J_LSR-1] = pi/12
    q_data_init[robot.J_RSR-1] = -pi/12
    
    Kp = 100
    tsamp = 0.01
    err_thresh = 0.001    
    
    q_tmp = q_data_init 
    ContactState = robot.LF
    
    # CoM of the init pose 
    CoM_init = robot.CoM(q_init,ContactState)
    
    ## generate init pose
    # from zero configuration to init pose
    (Q, Qd, Qdd) = jtraj(zeros((1,robot.NJ)),q_data_init,60)
    qdata = Q
    
    REF_FRAMES = robot.LF*ones((60,1))
    
    ## static backward staircase climbing motion generation
    
    q_tmp = q_data_init
    ## first step - left foot support
    ### parameters
    steplength = - 0.001*150
    stepheight = 0.001*75
    steplength_v = 0
    
    N_frames_step = 30
    N_frames_com = 40
    # contact state and swing foot
    ContactState = robot.LF
    F_swF = robot.F_RF
    
    ### current pose of the robot
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
    T_swF_ref[0,3] = T_swF_ref[0,3]# + 0.001*50;
    T_swF_ref[1,3] = T_swF_ref[1,3]
    T_swF_ref[2,3] = T_swF_ref[2,3]
    x_swF_ref = tr2diff(T_swF_ref)
    
    # right hand pose
    T_LH_ref = robot.FK_tree(q_tmp, robot.F_LH, ContactState)
    T_LH_init = T_LH_ref
    T_LH_ref[0,3] = T_LH_ref[0,3]# + 0.001*50;
    T_LH_ref[1,3] = T_LH_ref[1,3]
    T_LH_ref[2,3] = T_LH_ref[2,3]
    x_LH_ref = tr2diff(T_LH_ref)
    
    # left hand pose
    T_RH_ref = robot.FK_tree(q_tmp, robot.F_RH, ContactState)
    T_RH_init = T_RH_ref
    T_RH_ref[0,3] = T_RH_ref[0,3]# + 0.001*50;
    T_RH_ref[1,3] = T_RH_ref[1,3]
    T_RH_ref[2,3] = T_RH_ref[2,3]
    x_RH_ref = tr2diff(T_RH_ref)
    
    # CoM location
    x_com_ref = robot.CoM(q_tmp, ContactState)
    x_com_ref[0,0] = x_com_ref[0,0]
    x_com_ref[0,1] = x_com_ref[0,1]
    x_com_ref[0,2] = x_com_ref[0,2]
    
    Nframes_com = 30
    # generate CoM trajectory
    ydata = CosTraj(Nframes_com,CoM_init[0,1],0)
    
    com_data = zeros((Nframes_com,3))
    
    for i in range(Nframes_com):
        com_tmp = array(CoM_init)
        com_tmp[0,1] = ydata[0,i]
        com_data[i,] = com_tmp        
    
#    # generate right hand trajectory
#    Tdata_RH = zeros((4,4,N_hand_step))
#    
#    for I in range(N_hand_step):
#        T_RH_tmp = deepcopy(T_RH_init)
#        ang = ang_data[0,I]
#        T_RH_tmp[1,3] = T_RH_init[1,3] + (Wheel_radius - Wheel_radius*cos(ang))
#        T_RH_tmp[2,3] = T_RH_init[2,3] + Wheel_radius*sin(ang)
#        Tdata_RH[:,:,I] = T_RH_tmp
    
    Nframes = Nframes_com
    
    err_thresh = 0.001
    #qdata = vstack((qdata,q_tmp.T))
    
    #tic = time.clock()
    for I in range(Nframes):
        x_com_ref = mat(com_data[I,]).T
        #T_RH_ref = Tdata_RH[:,:,I]
        #x_RH_ref = tr2diff(T_RH_ref)
        
        error = 10000
        cnt = 1
        while norm(error) > err_thresh:
            x_W = tr2diff(robot.FK_tree(q_tmp, robot.F_HPY, ContactState))
            err_x_W = x_W_ref - x_W
            err_x_W = err_x_W[3:]
    
            x_swF = tr2diff(robot.FK_tree(q_tmp, F_swF, ContactState))
            err_x_swF = x_swF_ref - x_swF
            
            x_RH = tr2diff(robot.FK_tree(q_tmp, robot.F_RH, ContactState))
            err_x_RH = x_RH_ref - x_RH
            
            x_LH = tr2diff(robot.FK_tree(q_tmp, robot.F_LH, ContactState))
            err_x_LH = x_LH_ref - x_LH
            
            x_com = robot.CoM(q_tmp, ContactState)
            err_x_com = x_com_ref - x_com.T
    
            J_W = robot.J_tree(q_tmp, robot.F_HPY, ContactState)
            J_W = J_W[3:,]
            
            J_swF = robot.J_tree(q_tmp, F_swF, ContactState)
                        
            J_RH = robot.J_tree(q_tmp, robot.F_RH, ContactState)
            J_LH = robot.J_tree(q_tmp, robot.F_LH, ContactState)
            
            J_com = robot.J_com_tree(q_tmp, ContactState)
            J_com_legs = hstack((zeros((3,1)), J_com[:,robot.J_LHY-1:robot.J_RAR], zeros((3,14))))              
            
            Jacob = vstack((J_com_legs,J_swF,J_W))
            error = vstack((err_x_com,err_x_swF,err_x_W))
            
            qd_tmp = Kp*pinv(Jacob)*error
            q_tmp = q_tmp + qd_tmp*tsamp
            #q_tmp = robot.joint_limit_check(q_tmp)
    
            cnt = cnt + 1
            
            print I
            print cnt
            print norm(error)
        #q_tmp[robot.J_RWR-1,0] = q_data_init[robot.J_RWR-1,0] + ang_data[0,I]
        qdata = vstack((qdata,q_tmp.T))
        REF_FRAMES = vstack((REF_FRAMES,[ContactState]))
        
    #toc = time.clock()
    
    #print toc - tic
    ## generate end pose
    (Q, Qd, Qdd) = jtraj(qdata[-1:,:],zeros((1,robot.NJ)),60)
    #Q = Qdata[0] # joint position trajectory
    qdata = vstack((qdata,Q))
    REF_FRAMES = vstack((REF_FRAMES,ContactState*ones((60,1))))
    
    ## interpolation
    qdata1 = spline_interp(qdata, 100)    
    #print qdata1
    
    ## send the motion to Hubo-Ach
    #print "sending data to Hubo-Ach"
    #moveRobot(qdata1,0.005)    
    
    ## plot qdata
    #from Ani_Robot import *
    #Ani_robot(robot,qdata,REF_FRAMES)  


## 07-28-13 Both Hands Wheel Turning
if select == 3:
    
    import time
    import robot
    from copy import copy, deepcopy
    from construct_DRC_Hubo import *
    from motion2ach_update import *
    #from Ani_Robot import *
#    from motion2ach import *

    robot = construct_DRC_Hubo()
    q_data = mat(zeros((robot.NJ,1)))
    q_data_zero = q_data
    
    q_data_init = q_data
    q_data_init[robot.J_HPY-1] = 0
    q_data_init[robot.J_LHP-1] = -pi/8
    q_data_init[robot.J_LKP-1] = pi/4
    q_data_init[robot.J_LAP-1] = -pi/8
    q_data_init[robot.J_RHP-1] = -pi/8
    q_data_init[robot.J_RKP-1] = pi/4
    q_data_init[robot.J_RAP-1] = -pi/8
    q_data_init[robot.J_RSP-1] = 0.36
    q_data_init[robot.J_RSR-1] = -0.2618
    q_data_init[robot.J_RSY-1] = 0
    q_data_init[robot.J_REP-1] = -1.9599
    q_data_init[robot.J_RWY-1] = 0
    q_data_init[robot.J_RWP-1] = 0
    q_data_init[robot.J_RWR-1] = -1.5708
    q_data_init[robot.J_LSP-1] = 0.36
    q_data_init[robot.J_LSR-1] = 0.2618
    q_data_init[robot.J_LSY-1] = 0
    q_data_init[robot.J_LEP-1] = -1.9599
    q_data_init[robot.J_LWY-1] = 0
    q_data_init[robot.J_LWP-1] = 0
    q_data_init[robot.J_LWR-1] = 1.5708
    
    q_init = q_data_init
    
    ## Find Init Pose - 90 degree wrist position 
    X_offset = 0.001*0
    Y_offset = -0.001*50
    Z_offset = 0.001*0
    axis_rotation = [0, 1, 0]
    angle = deg2rad(-90)
    
    Kp = 100
    tsamp = 0.01
    err_thresh = 0.001
    
    ContactState = robot.LF
    F_swF = robot.F_RF
    
    F_RH = robot.F_RWP
    F_LH = robot.F_LWP
    
    q_tmp = q_data_init
    #q = q_tmp
    
    
    
    
    
    # right hand pose
    T_RH_ref = robot.FK_tree(q_tmp, F_RH, ContactState)
    T_RH_init = T_RH_ref
    R_RH_ref = t2r(T_RH_ref)
    R_RH_ref = rotvec2r(angle,axis_rotation)
    T_RH_ref[0,3] = T_RH_ref[0,3] + X_offset
    T_RH_ref[1,3] = T_RH_ref[1,3] + Y_offset
    T_RH_ref[2,3] = T_RH_ref[2,3] + Z_offset
    T_RH_ref[0:3,0:3] = R_RH_ref
    x_RH_ref = tr2diff(T_RH_ref)
    
    # left hand pose
    T_LH_ref = robot.FK_tree(q_tmp, F_LH, ContactState)
    T_LH_init = T_LH_ref
    R_LH_ref = t2r(T_LH_ref)
    R_LH_ref = rotvec2r(angle,axis_rotation)
    T_LH_ref[0,3] = T_LH_ref[0,3] + X_offset
    T_LH_ref[1,3] = T_LH_ref[1,3] + Y_offset
    T_LH_ref[2,3] = T_LH_ref[2,3] + Z_offset
    T_LH_ref[0:3,0:3] = R_LH_ref
    x_LH_ref = tr2diff(T_LH_ref)
    
    
    
    error = 10000
    cnt = 1
    
    #tic = time.clock()
    while norm(error) > err_thresh:
        x_RH = tr2diff(robot.FK_tree(q_tmp, F_RH, ContactState))
        err_x_RH = x_RH_ref - x_RH
        
        x_LH = tr2diff(robot.FK_tree(q_tmp, F_LH, ContactState))
        err_x_LH = x_LH_ref - x_LH
        
            
        J_RH = robot.J_tree(q_tmp, F_RH, ContactState)
        J_RH = J_RH

        J_LH = robot.J_tree(q_tmp, F_LH, ContactState)
        J_LH = J_LH
        
        Jacob = vstack((J_RH, J_LH))
        error = vstack((err_x_RH, err_x_LH))
        
        qd_tmp = Kp*pinv(Jacob)*error
        q_tmp = q_tmp + qd_tmp*tsamp
        q_tmp = robot.joint_limit_check(q_tmp)
        
        cnt = cnt + 1
        
        #clc
        print cnt
        print norm(error)
    #toc = time.clock()
    
    #print toc - tic
    
    q_data_init = q_tmp
    ## generate init pose
    # from zero configuration to init pose
    (Q, Qd, Qdd) = jtraj(zeros((1,robot.NJ)),q_data_init,60)
    #Q = Qdata[0] # joint position trajectory
    qdata = Q
    #qdata1 = spline_interp(qdata, 100)    
    #moveRobot(qdata1,0.00)    
    
    ## Generate Steering Wheel Rotating motion
    Wheel_diameter = 0.001*330.02
    Wheel_thickness = 0.001*25.4
    Wheel_radius = (Wheel_diameter - Wheel_thickness/2)/2
    
    Wheel_rotation = deg2rad(45)
    
    N_hand_step = 30
    
    ### current pose of the robot
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
    T_swF_ref[0,3] = T_swF_ref[0,3]# + 0.001*50;
    T_swF_ref[1,3] = T_swF_ref[1,3]
    T_swF_ref[2,3] = T_swF_ref[2,3]
    x_swF_ref = tr2diff(T_swF_ref)
    
    # right hand
    T_RH_ref = robot.FK_tree(q_tmp, robot.F_RH, ContactState)
    T_RH_init = T_RH_ref
    
    # left hand
    T_LH_ref = robot.FK_tree(q_tmp, robot.F_LH, ContactState)
    T_LH_init = T_LH_ref
    
    # CoM location
    x_com_ref = robot.CoM(q_data_zero, ContactState)
    x_com_ref[0,0] = x_com_ref[0,0]
    x_com_ref[0,1] = x_com_ref[0,1]
    x_com_ref[0,2] = x_com_ref[0,2]
    
    ang_data = CosTraj(N_hand_step, 0, Wheel_rotation)
    
    # generate both hands trajectory
    Tdata_RH = zeros((4,4,N_hand_step))
    Tdata_LH = zeros((4,4,N_hand_step))
    
    for I in range(N_hand_step):
        T_RH_tmp = deepcopy(T_RH_init)
        T_LH_tmp = deepcopy(T_LH_init)
        ang = ang_data[0,I]
        T_RH_tmp[1,3] = T_RH_init[1,3] + (Wheel_radius - Wheel_radius*cos(ang))
        T_RH_tmp[2,3] = T_RH_init[2,3] + Wheel_radius*sin(ang)
        
        ang = ang_data[0,I]
        T_LH_tmp[1,3] = T_LH_init[1,3] + (Wheel_radius - Wheel_radius*cos(ang))
        T_LH_tmp[2,3] = T_LH_init[2,3] + Wheel_radius*sin(ang)
        
        Tdata_RH[:,:,I] = T_RH_tmp
        Tdata_LH[:,:,I] = T_LH_tmp
    
    Nframes = N_hand_step
    
    err_thresh = 0.001
    #qdata = vstack((qdata,q_tmp.T))
    
    for I in range(Nframes):
        T_RH_ref = Tdata_RH[:,:,I]
        x_RH_ref = tr2diff(T_RH_ref)
        
        T_LH_ref = Tdata_LH[:,:,I]
        x_LH_ref = tr2diff(T_LH_ref)
        
        error = 10000
        cnt = 1
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
            
            x_com = robot.CoM(q_tmp, ContactState)
            err_x_com = x_com_ref.T - x_com.T
            
            J_W = robot.J_tree(q_tmp, robot.F_HPY, ContactState)
            J_W = J_W[3:,]
            
            J_swF = robot.J_tree(q_tmp, F_swF, ContactState)
            
            J_RH = robot.J_tree(q_tmp, robot.F_RH, ContactState)
            J_RH = J_RH
    
            J_LH = robot.J_tree(q_tmp, robot.F_LH, ContactState)
            J_LH = J_LH
            
            J_com = robot.J_com_tree(q_tmp, ContactState)
            J_com_legs = hstack((zeros((3,1)), J_com[:,robot.J_LHY-1:robot.J_RAR], zeros((3,14))))              
            
            Jacob = vstack((J_com_legs[0,:], J_swF))
            error = vstack((err_x_com[0], err_x_swF))
            
            qd_tmp = Kp*pinv(Jacob)*error
            q_tmp = q_tmp + qd_tmp*tsamp
            q_tmp = robot.joint_limit_check(q_tmp)
    
            cnt = cnt + 1
            
            print I
            print cnt
            print norm(error)
        q_tmp[robot.J_RWR-1,0] = q_data_init[robot.J_RWR-1,0] + ang_data[0,I]
        q_tmp[robot.J_LWR-1,0] = q_data_init[robot.J_LWR-1,0] + ang_data[0,I]
        qdata = vstack((qdata,q_tmp.T))
        
        #print tr2diff(robot.FK_tree(q_tmp, F_swF, ContactState))
        #print robot.CoM(q_tmp, ContactState)
        
    ## generate end pose
    (Q, Qd, Qdd) = jtraj(qdata[-1:,:],zeros((1,robot.NJ)),60)
    #Q = Qdata[0] # joint position trajectory
    qdata = vstack((qdata,Q))
    
    ## interpolation
    qdata1 = spline_interp(qdata, 100)    
    #print qdata1
    
    ## send the motion to Hubo-Ach
    print "sending data to Hubo-Ach"
    #moveRobot(qdata1,0.003)    
    
    ## plot qdata
    
    #robot.Ani_robot(qdata,2)    
    #from Ani_Robot import *
    #Ani_robot(robot,qdata,REF_FRAMES) 
    
## 07-29-13 Check Swing Foot 
if select == 4:
    
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
    
    q_data_init = q_data
    q_data_init[robot.J_HPY-1] = 0
    q_data_init[robot.J_LHP-1] = -pi/8
    q_data_init[robot.J_LKP-1] = pi/4
    q_data_init[robot.J_LAP-1] = -pi/8
    q_data_init[robot.J_RHP-1] = -pi/8
    q_data_init[robot.J_RKP-1] = pi/4
    q_data_init[robot.J_RAP-1] = -pi/8
    
    q_data_init[robot.J_RSP-1] = 0.5
    q_data_init[robot.J_RSR-1] = -0.2618
    q_data_init[robot.J_RSY-1] = 0
    q_data_init[robot.J_REP-1] = -1.5
    q_data_init[robot.J_RWY-1] = 0
    q_data_init[robot.J_RWP-1] = 0
    q_data_init[robot.J_RWR-1] = -1.5708
    
    q_data_init[robot.J_LSP-1] = 0.5
    q_data_init[robot.J_LSR-1] = 0.2618
    q_data_init[robot.J_LSY-1] = 0
    q_data_init[robot.J_LEP-1] = -1.5
    q_data_init[robot.J_LWY-1] = 0
    q_data_init[robot.J_LWP-1] = 0
    q_data_init[robot.J_LWR-1] = 1.5708
    
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
    #toc = time.clock()
    #print toc - tic
#    from Ani_Robot import *
#    Ani_robot(robot,q_tmp.T,2)
    
    
#    q_data_init = q_tmp
#    ## generate init pose
#    # from zero configuration to init pose
#    (Q, Qd, Qdd) = jtraj(zeros((1,robot.NJ)),q_data_init,60)
#    #Q = Qdata[0] # joint position trajectory
#    qdata = Q
#    #qdata1 = spline_interp(qdata, 100)    
#    #moveRobot(qdata1,0.00)    
#    
#    ## Generate Steering Wheel Rotating motion
#    Wheel_diameter = 0.001*330.02
#    Wheel_thickness = 0.001*25.4
#    Wheel_radius = (Wheel_diameter - Wheel_thickness/2)/2
#    
#    Wheel_rotation = deg2rad(45)
#    
#    N_hand_step = 30
#    
#    ### current pose of the robot
#    # waist pose
#    T_W_ref = robot.FK_tree(q_tmp, robot.F_HPY, ContactState)
#    T_W_init = T_W_ref
#    T_W_ref[0,3] = T_W_ref[0,3]# + 0.001*50;
#    T_W_ref[1,3] = T_W_ref[1,3]
#    T_W_ref[2,3] = T_W_ref[2,3]
#    x_W_ref = tr2diff(T_W_ref)
#    
#    # swing foot pose
#    T_swF_ref = robot.FK_tree(q_tmp, F_swF, ContactState)
#    T_swF_init = T_swF_ref
#    T_swF_ref[0,3] = T_swF_ref[0,3]# + 0.001*50;
#    T_swF_ref[1,3] = T_swF_ref[1,3]
#    T_swF_ref[2,3] = T_swF_ref[2,3]
#    x_swF_ref = tr2diff(T_swF_ref)
#    
#    # right hand
#    T_RH_ref = robot.FK_tree(q_tmp, F_RH, ContactState)
#    T_RH_init = T_RH_ref
#    
#    # left hand
#    T_LH_ref = robot.FK_tree(q_tmp, F_LH, ContactState)
#    T_LH_init = T_LH_ref
#    
#    # CoM location
#    x_com_ref = robot.CoM(q_data_zero, ContactState)
#    x_com_ref[0,0] = x_com_ref[0,0]
#    x_com_ref[0,1] = x_com_ref[0,1]
#    x_com_ref[0,2] = x_com_ref[0,2]
#    
#    ang_data = CosTraj(N_hand_step, 0, Wheel_rotation)
#    
#    # generate both hands trajectory
#    Tdata_RH = zeros((4,4,N_hand_step))
#    Tdata_LH = zeros((4,4,N_hand_step))
#    
#    for I in range(N_hand_step):
#        T_RH_tmp = deepcopy(T_RH_init)
#        T_LH_tmp = deepcopy(T_LH_init)
#        ang = ang_data[0,I]
#        T_RH_tmp[1,3] = T_RH_init[1,3] + (Wheel_radius - Wheel_radius*cos(ang))
#        T_RH_tmp[2,3] = T_RH_init[2,3] + Wheel_radius*sin(ang)
#        
#        ang = ang_data[0,I]
#        T_LH_tmp[1,3] = T_LH_init[1,3] + (Wheel_radius - Wheel_radius*cos(ang))
#        T_LH_tmp[2,3] = T_LH_init[2,3] + Wheel_radius*sin(ang)
#        
#        Tdata_RH[:,:,I] = T_RH_tmp
#        Tdata_LH[:,:,I] = T_LH_tmp
#    
#    Nframes = N_hand_step
#    
#    err_thresh = 0.001
#    #qdata = vstack((qdata,q_tmp.T))
#    
#    for I in range(Nframes):
#        T_RH_ref = Tdata_RH[:,:,I]
#        x_RH_ref = tr2diff(T_RH_ref)
#        
#        T_LH_ref = Tdata_LH[:,:,I]
#        x_LH_ref = tr2diff(T_LH_ref)
#        
#        error = 10000
#        cnt = 1
#        while norm(error) > err_thresh:
#            x_W = tr2diff(robot.FK_tree(q_tmp, robot.F_HPY, ContactState))
#            err_x_W = x_W_ref - x_W
#            err_x_W = err_x_W[3:]
#            
#            x_RH = tr2diff(robot.FK_tree(q_tmp, F_RH, ContactState))
#            err_x_RH = x_RH_ref - x_RH
#            
#            x_LH = tr2diff(robot.FK_tree(q_tmp, F_LH, ContactState))
#            err_x_LH = x_LH_ref - x_LH
#            
#            x_swF = tr2diff(robot.FK_tree(q_tmp, F_swF, ContactState))
#            err_x_swF = x_swF_ref - x_swF
#            
#            x_com = robot.CoM(q_tmp, ContactState)
#            err_x_com = x_com_ref.T - x_com.T
#            
#            J_W = robot.J_tree(q_tmp, robot.F_HPY, ContactState)
#            J_W = J_W[3:,]
#            
#            J_swF = robot.J_tree(q_tmp, F_swF, ContactState)
#            
#            J_RH = robot.J_tree(q_tmp, F_RH, ContactState)
#            J_RH = J_RH
#    
#            J_LH = robot.J_tree(q_tmp, F_LH, ContactState)
#            J_LH = J_LH
#            
#            J_com = robot.J_com_tree(q_tmp, ContactState)
#            J_com_legs = hstack((zeros((3,1)), J_com[:,robot.J_LHY-1:robot.J_RAR], zeros((3,14))))              
#            
#            Jacob = vstack((J_com_legs[0,:], J_swF))
#            error = vstack((err_x_com[0], err_x_swF))
#            
#            qd_tmp = Kp*pinv(Jacob)*error
#            q_tmp = q_tmp + qd_tmp*tsamp
#            q_tmp = robot.joint_limit_check(q_tmp)
#    
#            cnt = cnt + 1
#            
#            print I
#            print cnt
#            print norm(error)
#        q_tmp[robot.J_RWR-1,0] = q_data_init[robot.J_RWR-1,0] + ang_data[0,I]
#        q_tmp[robot.J_LWR-1,0] = q_data_init[robot.J_LWR-1,0] + ang_data[0,I]
#        qdata = vstack((qdata,q_tmp.T))
#        
#        print tr2diff(robot.FK_tree(q_tmp, F_swF, ContactState))
#        print robot.CoM(q_tmp, ContactState)
#        
#    ## generate end pose
#    (Q, Qd, Qdd) = jtraj(qdata[-1:,:],zeros((1,robot.NJ)),60)
#    #Q = Qdata[0] # joint position trajectory
#    qdata = vstack((qdata,Q))
#    
#    ## interpolation
#    qdata1 = spline_interp(qdata, 100)    
#    #print qdata1
#    
#    ## send the motion to Hubo-Ach
#    print "sending data to Hubo-Ach"
#    #moveRobot(qdata1,0.003)    
#    
#    ## plot qdata
#    
#    #robot.Ani_robot(qdata,2)    
#    #from Ani_Robot import *
#    #Ani_robot(robot,qdata,REF_FRAMES)     


## 07-29-13 Hand Position Modification
if select == 5:
    
    import time
    import robot
    from copy import copy, deepcopy
    from construct_DRC_Hubo import *
    from motion2ach_update import *
    #from Ani_Robot import *
#    from motion2ach import *

    robot = construct_DRC_Hubo()
    q_data = mat(zeros((robot.NJ,1)))
    
    q_data_init = q_data
    q_data_init[robot.J_HPY-1] = 0
    q_data_init[robot.J_LHP-1] = -pi/8
    q_data_init[robot.J_LKP-1] = pi/4
    q_data_init[robot.J_LAP-1] = -pi/8
    q_data_init[robot.J_RHP-1] = -pi/8
    q_data_init[robot.J_RKP-1] = pi/4
    q_data_init[robot.J_RAP-1] = -pi/8
    q_data_init[robot.J_RSP-1] = -1.42
    q_data_init[robot.J_RSR-1] = -1.3782
    q_data_init[robot.J_RSY-1] = -2.1016
    q_data_init[robot.J_REP-1] = -2.0999
    q_data_init[robot.J_RWY-1] = 0
    q_data_init[robot.J_RWP-1] = 0
    q_data_init[robot.J_RWR-1] = -0.24   
    q_init = q_data_init
    
    ## Find Init Pose - 90 degree wrist position 
    X_offset = 0.001*50
    Y_offset = 0.001*0
    Z_offset = 0.001*0
    axis_rotation = [0, 1, 0]
    angle = deg2rad(-90)
    
    Kp = 100
    tsamp = 0.01
    err_thresh = 0.001
    
    ContactState = robot.BODY
    
    F_RH = robot.F_RH
    
    q_tmp = q_data_init
    #q = q_tmp
    
    # right hand pose
    T_RH_ref = robot.FK_tree(q_tmp, F_RH, ContactState)
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
    
#    N_hand_step = 10    
    
#    # generate right hand trajectory
#    Tdata_RH = zeros((4,4,N_hand_step))
#    
#    for I in range(N_hand_step):
#        T_RH_tmp = deepcopy(T_RH_init)
#        ang = ang_data[0,I]
#        T_RH_tmp[1,3] = T_RH_init[1,3] + (Wheel_radius - Wheel_radius*cos(ang))
#        T_RH_tmp[2,3] = T_RH_init[2,3] + Wheel_radius*sin(ang)
#        Tdata_RH[:,:,I] = T_RH_tmp
#    
#    Nframes = N_hand_step
#    
#    for I in range(Nframes):
#        T_RH_ref = Tdata_RH[:,:,I]
#        x_RH_ref = tr2diff(T_RH_ref)

    ## generate init pose
    # from zero configuration to init pose
    (Q, Qd, Qdd) = jtraj(zeros((1,robot.NJ)),q_data_init,30)
    #Q = Qdata[0] # joint position trajectory
    qdata = Q
        
    error = 10000
    cnt = 1
    #tic = time.clock()
    while norm(error) > err_thresh:
        x_RH = tr2diff(robot.FK_tree(q_tmp, robot.F_RH, ContactState))
        err_x_RH = x_RH_ref - x_RH
        
#        x_LH = tr2diff(robot.FK_tree(q_tmp, robot.F_LH, ContactState))
#        err_x_LH = x_LH_ref - x_LH
        
        J_RH = robot.J_tree(q_tmp, robot.F_RH, ContactState)
        J_RH = J_RH
        
#        J_LH = robot.J_tree(q_tmp, robot.F_LH, ContactState)
#        J_LH = J_LH
#        
        Jacob = J_RH
        error = err_x_RH
        
        qd_tmp = Kp*pinv(Jacob)*error
        q_tmp = q_tmp + qd_tmp*tsamp
        q_tmp = robot.joint_limit_check(q_tmp)
        
        cnt = cnt + 1
        
            #clc
        print cnt
        print norm(error)
#        qdata = vstack((qdata,q_tmp.T))
        #toc = time.clock()
        
        #print toc - tic
    
    q_sol = q_tmp

    ## generate intermediate pose
    # from zero configuration to init pose
    (Q, Qd, Qdd) = jtraj(q_data_init,q_sol,10)
    #Q = Qdata[0] # joint position trajectory
    qdata = vstack((qdata,Q))

    ## generate pause
    (Q, Qd, Qdd) = jtraj(qdata[-1:,:],qdata[-1:,:],60)
    #Q = Qdata[0] # joint position trajectory
    qdata = vstack((qdata,Q))    
    
    ## generate end pose
    (Q, Qd, Qdd) = jtraj(qdata[-1:,:],zeros((1,robot.NJ)),30)
    #Q = Qdata[0] # joint position trajectory
    qdata = vstack((qdata,Q))

    ## interpolation
    qdata1 = spline_interp(qdata, 100)    
    #print qdata1
    
    ## send the motion to Hubo-Ach
    print "sending data to Hubo-Ach"
    moveRobot(qdata1,0.003)   
    
## 07-29-13 Hand Position Modification
# interactive - one arm
if select == 6:
    
    import time
    import robot
    from copy import copy, deepcopy
    from construct_DRC_Hubo import *
    from motion2ach_update import *
    #from Ani_Robot import *
#    from motion2ach import *

    robot = construct_DRC_Hubo()
    q_data = mat(zeros((robot.NJ,1)))
    
    q_data_init = q_data
    q_data_init[robot.J_HPY-1] = 0
    q_data_init[robot.J_LHP-1] = -pi/8
    q_data_init[robot.J_LKP-1] = pi/4
    q_data_init[robot.J_LAP-1] = -pi/8
    q_data_init[robot.J_RHP-1] = -pi/8
    q_data_init[robot.J_RKP-1] = pi/4
    q_data_init[robot.J_RAP-1] = -pi/8
    q_data_init[robot.J_RSP-1] = -1.42
    q_data_init[robot.J_RSR-1] = -1.3782
    q_data_init[robot.J_RSY-1] = -2.1016
    q_data_init[robot.J_REP-1] = -2.0999
    q_data_init[robot.J_RWY-1] = 0
    q_data_init[robot.J_RWP-1] = 0
    q_data_init[robot.J_RWR-1] = -0.24   
    q_init = q_data_init
    
    q_tmp = q_data_init
    
    ## generate init pose
    # from zero configuration to init pose
    (Q, Qd, Qdd) = jtraj(zeros((1,robot.NJ)),q_data_init,60)
    #Q = Qdata[0] # joint position trajectory
    qdata = Q
    
    ## interpolation
    qdata1 = spline_interp(qdata, 100)    
    ## send the motion to Hubo-Ach
    print "sending data to Hubo-Ach"
    moveRobot(qdata1,0.003)      

    Loop = 1
    while Loop == 1:
        ## Find Init Pose - 90 degree wrist position 
        x_input = float(raw_input('Enter x displacement [mm]:'))
        y_input = float(raw_input('Enter y displacement [mm]:'))
        z_input = float(raw_input('Enter z displacement [mm]:'))
        
        X_offset = 0.001*x_input
        Y_offset = 0.001*y_input
        Z_offset = 0.001*z_input
        
        s = raw_input('Enter Rotation Axis (x:1 0 0, y:0 1 0, z:0 0 1): ')
        axis_input = map(int, s.split())
        
        angle_input = raw_input('Enter angle in degree: ')
        axis_rotation = axis_input
        angle = deg2rad(float(angle_input))
        
        Kp = 100
        tsamp = 0.01
        err_thresh = 0.001
        
        ContactState = robot.BODY
        
        F_RH = robot.F_RH
        #q = q_tmp
        
        # right hand pose
        T_RH_ref = robot.FK_tree(q_tmp, F_RH, ContactState)
        T_RH_init = T_RH_ref
        R_RH_ref = t2r(T_RH_ref)
        R_RH_ref = rotvec2r(angle,axis_rotation)*R_RH_ref
        T_RH_ref[0,3] = T_RH_ref[0,3] + X_offset
        T_RH_ref[1,3] = T_RH_ref[1,3] + Y_offset
        T_RH_ref[2,3] = T_RH_ref[2,3] + Z_offset
        T_RH_ref[0:3,0:3] = R_RH_ref
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
        
    #    N_hand_step = 10    
        
    #    # generate right hand trajectory
    #    Tdata_RH = zeros((4,4,N_hand_step))
    #    
    #    for I in range(N_hand_step):
    #        T_RH_tmp = deepcopy(T_RH_init)
    #        ang = ang_data[0,I]
    #        T_RH_tmp[1,3] = T_RH_init[1,3] + (Wheel_radius - Wheel_radius*cos(ang))
    #        T_RH_tmp[2,3] = T_RH_init[2,3] + Wheel_radius*sin(ang)
    #        Tdata_RH[:,:,I] = T_RH_tmp
    #    
    #    Nframes = N_hand_step
    #    
    #    for I in range(Nframes):
    #        T_RH_ref = Tdata_RH[:,:,I]
    #        x_RH_ref = tr2diff(T_RH_ref)
    
     
        q_tmp_init = q_tmp    
        error = 10000
        cnt = 1
        #tic = time.clock()
        while norm(error) > err_thresh:
            x_RH = tr2diff(robot.FK_tree(q_tmp, robot.F_RH, ContactState))
            err_x_RH = x_RH_ref - x_RH
            
    #        x_LH = tr2diff(robot.FK_tree(q_tmp, robot.F_LH, ContactState))
    #        err_x_LH = x_LH_ref - x_LH
            
            J_RH = robot.J_tree(q_tmp, robot.F_RH, ContactState)
            J_RH = J_RH
            
    #        J_LH = robot.J_tree(q_tmp, robot.F_LH, ContactState)
    #        J_LH = J_LH
    #        
            Jacob = J_RH
            error = err_x_RH
            
            qd_tmp = Kp*pinv(Jacob)*error
            q_tmp = q_tmp + qd_tmp*tsamp
            q_tmp = robot.joint_limit_check(q_tmp)
            
            cnt = cnt + 1
            
                #clc
            print cnt
            print norm(error)
    #        qdata = vstack((qdata,q_tmp.T))
            #toc = time.clock()
            
            #print toc - tic
        
        #q_sol = q_tmp
    
        ## generate intermediate pose
        # from zero configuration to init pose
        (Q, Qd, Qdd) = jtraj(q_tmp_init,q_tmp,30)
        #Q = Qdata[0] # joint position trajectory
        qdata = Q
        
        ## interpolation
        qdata1 = spline_interp(qdata, 100)    
        ## send the motion to Hubo-Ach
        print "sending data to Hubo-Ach"
        moveRobot(qdata1,0.003)   
        
        Loop = int(raw_input('Do you want to continue? 1: Yes, 0: No: '))

    ## generate end pose
    (Q, Qd, Qdd) = jtraj(q_tmp,zeros((1,robot.NJ)),60)
    #Q = Qdata[0] # joint position trajectory
    qdata = Q

    ## interpolation
    qdata1 = spline_interp(qdata, 100)    
    #print qdata1
    
    ## send the motion to Hubo-Ach
    print "sending data to Hubo-Ach"
    moveRobot(qdata1,0.003)   
    
## 07-29-13 Hand Position Modification
# interactive - both arms
if select == 7:
    
    import time
    import robot
    from copy import copy, deepcopy
    from construct_DRC_Hubo import *
    from motion2ach_update import *
    #from Ani_Robot import *
#    from motion2ach import *

    robot = construct_DRC_Hubo()
    q_data = mat(zeros((robot.NJ,1)))
    
    q_data_init = q_data
    q_data_init[robot.J_HPY-1] = 0
    q_data_init[robot.J_LHP-1] = -pi/8
    q_data_init[robot.J_LKP-1] = pi/4
    q_data_init[robot.J_LAP-1] = -pi/8
    q_data_init[robot.J_RHP-1] = -pi/8
    q_data_init[robot.J_RKP-1] = pi/4
    q_data_init[robot.J_RAP-1] = -pi/8
    q_data_init[robot.J_RSP-1] = -1.42
    q_data_init[robot.J_RSR-1] = -1.3782
    q_data_init[robot.J_RSY-1] = -2.1016
    q_data_init[robot.J_REP-1] = -1.5
    q_data_init[robot.J_RWY-1] = 0
    q_data_init[robot.J_RWP-1] = 0
    q_data_init[robot.J_RWR-1] = -0.24   
    q_data_init[robot.J_LSP-1] = -1.42
    q_data_init[robot.J_LSR-1] = 1.3782
    q_data_init[robot.J_LSY-1] = 2.1016
    q_data_init[robot.J_LEP-1] = -1.5
    q_data_init[robot.J_LWY-1] = 0
    q_data_init[robot.J_LWP-1] = 0
    q_data_init[robot.J_LWR-1] = 0.24   
    q_init = q_data_init
    
    q_tmp = q_data_init
    
    ## generate init pose
    # from zero configuration to init pose
    (Q, Qd, Qdd) = jtraj(zeros((1,robot.NJ)),q_data_init,60)
    #Q = Qdata[0] # joint position trajectory
    qdata = Q
    
    ## interpolation
    qdata1 = spline_interp(qdata, 100)    
    ## send the motion to Hubo-Ach
    print "sending data to Hubo-Ach"
    moveRobot(qdata1,0.003)      
    
    cnt_loop = 0
    q_sol_data = q_init.T

    Loop = 1
    while Loop == 1:
        ## Find Init Pose - 90 degree wrist position 

        
#        s = raw_input('Enter Rotation Axis (x:1 0 0, y:0 1 0, z:0 0 1): ')
#        axis_input = map(int, s.split())
#        
#        angle_input = raw_input('Enter angle in degree: ')
#        axis_rotation = axis_input
#        angle = deg2rad(float(angle_input))
        
        Kp = 100
        tsamp = 0.01
        err_thresh = 0.001
        
        ContactState = robot.BODY
        
        F_RH = robot.F_RH
        #q = q_tmp

        print('right hand position')
        x_input = float(raw_input('Enter x displacement [mm]:'))
        y_input = float(raw_input('Enter y displacement [mm]:'))
        z_input = float(raw_input('Enter z displacement [mm]:'))
        
        X_offset = 0.001*x_input
        Y_offset = 0.001*y_input
        Z_offset = 0.001*z_input        
        
        # right hand pose
        T_RH_ref = robot.FK_tree(q_tmp, F_RH, ContactState)
        T_RH_init = T_RH_ref
#        R_RH_ref = t2r(T_RH_ref)
#        R_RH_ref = rotvec2r(angle,axis_rotation)*R_RH_ref
        T_RH_ref[0,3] = T_RH_ref[0,3] + X_offset
        T_RH_ref[1,3] = T_RH_ref[1,3] + Y_offset
        T_RH_ref[2,3] = T_RH_ref[2,3] + Z_offset
#        T_RH_ref[0:3,0:3] = R_RH_ref
        x_RH_ref = tr2diff(T_RH_ref)

        
        print('left hand position')
        x_input = float(raw_input('Enter x displacement [mm]:'))
        y_input = float(raw_input('Enter y displacement [mm]:'))
        z_input = float(raw_input('Enter z displacement [mm]:'))
        
        X_offset = 0.001*x_input
        Y_offset = 0.001*y_input
        Z_offset = 0.001*z_input        
        
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
        
    #    N_hand_step = 10    
        
    #    # generate right hand trajectory
    #    Tdata_RH = zeros((4,4,N_hand_step))
    #    
    #    for I in range(N_hand_step):
    #        T_RH_tmp = deepcopy(T_RH_init)
    #        ang = ang_data[0,I]
    #        T_RH_tmp[1,3] = T_RH_init[1,3] + (Wheel_radius - Wheel_radius*cos(ang))
    #        T_RH_tmp[2,3] = T_RH_init[2,3] + Wheel_radius*sin(ang)
    #        Tdata_RH[:,:,I] = T_RH_tmp
    #    
    #    Nframes = N_hand_step
    #    
    #    for I in range(Nframes):
    #        T_RH_ref = Tdata_RH[:,:,I]
    #        x_RH_ref = tr2diff(T_RH_ref)
    
     
        q_tmp_init = q_tmp    
        error = 10000
        cnt = 1
        #tic = time.clock()
        while norm(error) > err_thresh:
            x_RH = tr2diff(robot.FK_tree(q_tmp, robot.F_RH, ContactState))
            err_x_RH = x_RH_ref - x_RH
            
            x_LH = tr2diff(robot.FK_tree(q_tmp, robot.F_LH, ContactState))
            err_x_LH = x_LH_ref - x_LH
            
            J_RH = robot.J_tree(q_tmp, robot.F_RH, ContactState)
            J_RH = J_RH
            
            J_LH = robot.J_tree(q_tmp, robot.F_LH, ContactState)
            J_LH = J_LH
    #        
            Jacob = vstack((J_LH,J_RH))
            error = vstack((err_x_LH,err_x_RH))
            
            qd_tmp = Kp*pinv(Jacob)*error
            q_tmp = q_tmp + qd_tmp*tsamp
            q_tmp = robot.joint_limit_check(q_tmp)
            
            cnt = cnt + 1
            
                #clc
#            print cnt
#            print norm(error)
            if cnt > 200:
                error = 0.0001
                q_tmp = q_tmp_init
                print('solution is not found, try with different numbers')
                flag_sol = 0
            else:
                flag_sol = 1
    #        qdata = vstack((qdata,q_tmp.T))
            #toc = time.clock()
            
            #print toc - tic

        # if the solution is not found
        # go to previous motions 
        if flag_sol == 0:
            index_tmp = int(raw_input('how many steps back?:'))
            q_tmp = q_sol_data[cnt_loop-index_tmp,].T
            
        else:
            # store the configuration
            q_sol = q_tmp
            q_sol_data = vstack((q_sol_data,q_sol.T))
            cnt_loop = cnt_loop + 1
            
        ## generate intermediate pose
        # from zero configuration to init pose
        (Q, Qd, Qdd) = jtraj(q_tmp_init,q_tmp,30)
        #Q = Qdata[0] # joint position trajectory
        qdata = Q    
            
        ## interpolation
        qdata1 = spline_interp(qdata, 100)    
        ## send the motion to Hubo-Ach
        print "sending data to Hubo-Ach"
        moveRobot(qdata1,0.003)   
        
        Loop = int(raw_input('Do you want to continue? 1: Yes, 0: No: '))

    ## generate end pose
    (Q, Qd, Qdd) = jtraj(q_tmp,zeros((1,robot.NJ)),60)
    #Q = Qdata[0] # joint position trajectory
    qdata = Q

    ## interpolation
    qdata1 = spline_interp(qdata, 100)    
    #print qdata1
    
    ## send the motion to Hubo-Ach
    print "sending data to Hubo-Ach"
    moveRobot(qdata1,0.003)   
    
## 07-30-13 Foot Step Correction
if select == 8:
    import time
    #import robot
    from copy import copy, deepcopy
    from construct_DRC_Hubo import *
    from motion2ach_update import *
    #from Ani_Robot import *
#    from motion2ach import *
    
    robot = construct_DRC_Hubo()
    q_data = mat(zeros((robot.NJ,1)))
    
    q_data_init = q_data
    q_data_init[robot.J_HPY-1] = 0
    q_data_init[robot.J_LHP-1] = -pi/8
    q_data_init[robot.J_LKP-1] = pi/4
    q_data_init[robot.J_LAP-1] = -pi/8
    q_data_init[robot.J_RHP-1] = -pi/8
    q_data_init[robot.J_RKP-1] = pi/4
    q_data_init[robot.J_RAP-1] = -pi/8
    q_init = q_data_init
    
    q_data_init[robot.J_LEP-1] = -3*pi/4
    q_data_init[robot.J_REP-1] = -3*pi/4
    q_data_init[robot.J_LSR-1] = pi/12
    q_data_init[robot.J_RSR-1] = -pi/12
    
    
#    q_data_init[robot.J_RSP-1] = -1.42
#    q_data_init[robot.J_RSR-1] = -1.3782
#    q_data_init[robot.J_RSY-1] = -2.1016
#    q_data_init[robot.J_REP-1] = -2.0999
#    q_data_init[robot.J_RWY-1] = 0
#    q_data_init[robot.J_RWP-1] = 0
#    q_data_init[robot.J_RWR-1] = -0.24   
#    q_data_init[robot.J_LSP-1] = -1.42
#    q_data_init[robot.J_LSR-1] = 1.3782
#    q_data_init[robot.J_LSY-1] = 2.1016
#    q_data_init[robot.J_LEP-1] = -2.0999
#    q_data_init[robot.J_LWY-1] = 0
#    q_data_init[robot.J_LWP-1] = 0
#    q_data_init[robot.J_LWR-1] = 0.24   
    
    Kp = 100
    tsamp = 0.01
    err_thresh = 0.001    
    
    q_tmp = q_data_init 
    ContactState = robot.LF
    
    # CoM of the init pose 
    CoM_init = robot.CoM(q_init,ContactState)
    
    ## generate init pose
    # from zero configuration to init pose
    (Q, Qd, Qdd) = jtraj(zeros((1,robot.NJ)),q_data_init,60)
    qdata = Q
    
    REF_FRAMES = robot.LF*ones((60,1))
    
    ## static backward staircase climbing motion generation
    q_tmp = q_data_init
    ## first step - left foot support
    ### parameters
    steplength = - 0.001*50
    stepheight = 0.001*75
    steplength_v = 0
    
    Nframes_step = 30
    Nframes_com = 40
    # contact state and swing foot
    ContactState = robot.LF
    F_swF = robot.F_RF
    
    ### current pose of the robot
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
    T_swF_ref[0,3] = T_swF_ref[0,3]# + 0.001*50;
    T_swF_ref[1,3] = T_swF_ref[1,3]
    T_swF_ref[2,3] = T_swF_ref[2,3]
    x_swF_ref = tr2diff(T_swF_ref)
    
    # right hand pose
    T_LH_ref = robot.FK_tree(q_tmp, robot.F_LH, ContactState)
    T_LH_init = T_LH_ref
    T_LH_ref[0,3] = T_LH_ref[0,3]# + 0.001*50;
    T_LH_ref[1,3] = T_LH_ref[1,3]
    T_LH_ref[2,3] = T_LH_ref[2,3]
    x_LH_ref = tr2diff(T_LH_ref)
    
    # left hand pose
    T_RH_ref = robot.FK_tree(q_tmp, robot.F_RH, ContactState)
    T_RH_init = T_RH_ref
    T_RH_ref[0,3] = T_RH_ref[0,3]# + 0.001*50;
    T_RH_ref[1,3] = T_RH_ref[1,3]
    T_RH_ref[2,3] = T_RH_ref[2,3]
    x_RH_ref = tr2diff(T_RH_ref)
    
    # CoM location
    x_com_ref = robot.CoM(q_tmp, ContactState)
    x_com_ref[0,0] = x_com_ref[0,0]
    x_com_ref[0,1] = x_com_ref[0,1]
    x_com_ref[0,2] = x_com_ref[0,2]
    
    # generate CoM trajectory
    margin_x = 0.001*0
    margin_y = 0.001*40
    com_data = com_motion_generation(robot, q_tmp, ContactState, Nframes_com, margin_x, margin_y)
       
    # generate swing foot stepping motion 
    stepcollision = 0.001*0
    Tdata_swF_tmp = stepping_motion_generation(robot, q_tmp, ContactState, steplength, stepheight, Nframes_step, steplength_v, 1, stepcollision)
    
    ### pad trajectories 
    # com trajectory
    com_tmp = com_data[-1:,]
    for i in range(Nframes_step):
        com_data = vstack((com_data,com_tmp))
    
    Tdata_swF = zeros((4,4,Nframes_com+Nframes_step))
    # swing foot trajectory
    for i in range(Nframes_com):
        Tdata_swF[:,:,i] = T_swF_init
    
    for i in range(Nframes_com, Nframes_com + Nframes_step):
        Tdata_swF[:,:,i] = Tdata_swF_tmp[:,:,i-Nframes_com]
           
    Nframes = Nframes_com + Nframes_step
    
    err_thresh = 0.001
    #qdata = vstack((qdata,q_tmp.T))
    
    #tic = time.clock()
    for I in range(Nframes):
        x_com_ref = mat(com_data[I,]).T
        
        T_swF_ref = Tdata_swF[:,:,I]
        x_swF_ref = tr2diff(T_swF_ref)
        
        error = 10000
        cnt = 1
        while norm(error) > err_thresh:
            x_W = tr2diff(robot.FK_tree(q_tmp, robot.F_HPY, ContactState))
            err_x_W = x_W_ref - x_W
            err_x_W = err_x_W[3:]
    
            x_swF = tr2diff(robot.FK_tree(q_tmp, F_swF, ContactState))
            err_x_swF = x_swF_ref - x_swF
            
            x_RH = tr2diff(robot.FK_tree(q_tmp, robot.F_RH, ContactState))
            err_x_RH = x_RH_ref - x_RH
            
            x_LH = tr2diff(robot.FK_tree(q_tmp, robot.F_LH, ContactState))
            err_x_LH = x_LH_ref - x_LH
            
            x_com = robot.CoM(q_tmp, ContactState)
            err_x_com = x_com_ref - x_com.T
    
            J_W = robot.J_tree(q_tmp, robot.F_HPY, ContactState)
            J_W = J_W[3:,]
            
            J_swF = robot.J_tree(q_tmp, F_swF, ContactState)
                        
            J_RH = robot.J_tree(q_tmp, robot.F_RH, ContactState)
            
            J_LH = robot.J_tree(q_tmp, robot.F_LH, ContactState)
            
            J_com = robot.J_com_tree(q_tmp, ContactState)
            J_com_legs = hstack((zeros((3,1)), J_com[:,robot.J_LHY-1:robot.J_RAR], zeros((3,14))))              
            
            Jacob = vstack((J_com_legs[:2,],J_swF,J_W))
            error = vstack((err_x_com[:2,],err_x_swF,err_x_W))
            
#            Jacob = vstack((J_com_legs[:2,],J_swF,J_W,J_LH,J_RH))
#            error = vstack((err_x_com[:2,],err_x_swF,err_x_W,err_x_LH,err_x_RH))
            
            qd_tmp = Kp*pinv(Jacob)*error
            q_tmp = q_tmp + qd_tmp*tsamp
            q_tmp = robot.joint_limit_check(q_tmp)
    
            cnt = cnt + 1
            
            print I
            print cnt
            print norm(error)
        #q_tmp[robot.J_RWR-1,0] = q_data_init[robot.J_RWR-1,0] + ang_data[0,I]
        qdata = vstack((qdata,q_tmp.T))
        REF_FRAMES = vstack((REF_FRAMES,[ContactState]))
        
    ## second step - right foot support
    ### parameters
    steplength = -0.001*50
    stepheight = 0.001*75
    steplength_v = 0
    
    Nframes_step = 30
    Nframes_com = 30
    # contact state and swing foot
    ContactState = robot.RF
    F_swF = robot.F_LF
    
    ### current pose of the robot
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
    T_swF_ref[0,3] = T_swF_ref[0,3]# + 0.001*50;
    T_swF_ref[1,3] = T_swF_ref[1,3]
    T_swF_ref[2,3] = T_swF_ref[2,3]
    x_swF_ref = tr2diff(T_swF_ref)
    
    # right hand pose
    T_LH_ref = robot.FK_tree(q_tmp, robot.F_LH, ContactState)
    T_LH_init = T_LH_ref
    T_LH_ref[0,3] = T_LH_ref[0,3]# + 0.001*50;
    T_LH_ref[1,3] = T_LH_ref[1,3]
    T_LH_ref[2,3] = T_LH_ref[2,3]
    x_LH_ref = tr2diff(T_LH_ref)
    
    # left hand pose
    T_RH_ref = robot.FK_tree(q_tmp, robot.F_RH, ContactState)
    T_RH_init = T_RH_ref
    T_RH_ref[0,3] = T_RH_ref[0,3]# + 0.001*50;
    T_RH_ref[1,3] = T_RH_ref[1,3]
    T_RH_ref[2,3] = T_RH_ref[2,3]
    x_RH_ref = tr2diff(T_RH_ref)
    
    # CoM location
    x_com_ref = robot.CoM(q_tmp, ContactState)
    x_com_ref[0,0] = x_com_ref[0,0]
    x_com_ref[0,1] = x_com_ref[0,1]
    x_com_ref[0,2] = x_com_ref[0,2]
    
    # generate CoM trajectory
    margin_x = 0.001*0
    margin_y = 0.001*40
    com_data = com_motion_generation(robot, q_tmp, ContactState, Nframes_com, margin_x, margin_y)
       
    # generate swing foot stepping motion 
    stepcollision = 0.001*0
    Tdata_swF_tmp = stepping_motion_generation(robot, q_tmp, ContactState, steplength, stepheight, Nframes_step, steplength_v, 1, stepcollision)
    
    ### pad trajectories 
    # com trajectory
    com_tmp = com_data[-1:,]
    for i in range(Nframes_step):
        com_data = vstack((com_data,com_tmp))
    
    Tdata_swF = zeros((4,4,Nframes_com+Nframes_step))
    # swing foot trajectory
    for i in range(Nframes_com):
        Tdata_swF[:,:,i] = T_swF_init
    
    for i in range(Nframes_com, Nframes_com + Nframes_step):
        Tdata_swF[:,:,i] = Tdata_swF_tmp[:,:,i-Nframes_com]
           
    Nframes = Nframes_com + Nframes_step
    
    err_thresh = 0.001
    #qdata = vstack((qdata,q_tmp.T))
    
    #tic = time.clock()
    for I in range(Nframes):
        x_com_ref = mat(com_data[I,]).T
        
        T_swF_ref = Tdata_swF[:,:,I]
        x_swF_ref = tr2diff(T_swF_ref)
        
        error = 10000
        cnt = 1
        while norm(error) > err_thresh:
            x_W = tr2diff(robot.FK_tree(q_tmp, robot.F_HPY, ContactState))
            err_x_W = x_W_ref - x_W
            err_x_W = err_x_W[3:]
    
            x_swF = tr2diff(robot.FK_tree(q_tmp, F_swF, ContactState))
            err_x_swF = x_swF_ref - x_swF
            
            x_RH = tr2diff(robot.FK_tree(q_tmp, robot.F_RH, ContactState))
            err_x_RH = x_RH_ref - x_RH
            
            x_LH = tr2diff(robot.FK_tree(q_tmp, robot.F_LH, ContactState))
            err_x_LH = x_LH_ref - x_LH
            
            x_com = robot.CoM(q_tmp, ContactState)
            err_x_com = x_com_ref - x_com.T
    
            J_W = robot.J_tree(q_tmp, robot.F_HPY, ContactState)
            J_W = J_W[3:,]
            
            J_swF = robot.J_tree(q_tmp, F_swF, ContactState)
                        
            J_RH = robot.J_tree(q_tmp, robot.F_RH, ContactState)
            
            J_LH = robot.J_tree(q_tmp, robot.F_LH, ContactState)
            
            J_com = robot.J_com_tree(q_tmp, ContactState)
            J_com_legs = hstack((zeros((3,1)), J_com[:,robot.J_LHY-1:robot.J_RAR], zeros((3,14))))              
            
            Jacob = vstack((J_com_legs[:2,],J_swF,J_W))
            error = vstack((err_x_com[:2,],err_x_swF,err_x_W))
            
#            Jacob = vstack((J_com_legs[:2,],J_swF,J_W,J_LH,J_RH))
#            error = vstack((err_x_com[:2,],err_x_swF,err_x_W,err_x_LH,err_x_RH))
                       
            qd_tmp = Kp*pinv(Jacob)*error
            q_tmp = q_tmp + qd_tmp*tsamp
            q_tmp = robot.joint_limit_check(q_tmp)
    
            cnt = cnt + 1
            
            print I
            print cnt
            print norm(error)
        #q_tmp[robot.J_RWR-1,0] = q_data_init[robot.J_RWR-1,0] + ang_data[0,I]
        qdata = vstack((qdata,q_tmp.T))
        REF_FRAMES = vstack((REF_FRAMES,[ContactState]))    
    
    ## third step - move the CoM to the center
    ### parameters
    #Nframes_step = 30
    Nframes_com = 30
    # contact state and swing foot
    ContactState = robot.RF
    F_swF = robot.F_LF
    
    ### current pose of the robot
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
    T_swF_ref[0,3] = T_swF_ref[0,3]# + 0.001*50;
    T_swF_ref[1,3] = T_swF_ref[1,3]
    T_swF_ref[2,3] = T_swF_ref[2,3]
    x_swF_ref = tr2diff(T_swF_ref)
    
    # right hand pose
    T_LH_ref = robot.FK_tree(q_tmp, robot.F_LH, ContactState)
    T_LH_init = T_LH_ref
    T_LH_ref[0,3] = T_LH_ref[0,3]# + 0.001*50;
    T_LH_ref[1,3] = T_LH_ref[1,3]
    T_LH_ref[2,3] = T_LH_ref[2,3]
    x_LH_ref = tr2diff(T_LH_ref)
    
    # left hand pose
    T_RH_ref = robot.FK_tree(q_tmp, robot.F_RH, ContactState)
    T_RH_init = T_RH_ref
    T_RH_ref[0,3] = T_RH_ref[0,3]# + 0.001*50;
    T_RH_ref[1,3] = T_RH_ref[1,3]
    T_RH_ref[2,3] = T_RH_ref[2,3]
    x_RH_ref = tr2diff(T_RH_ref)
    
    # CoM location
    x_com_ref = robot.CoM(q_tmp, ContactState)
    x_com_ref[0,0] = x_com_ref[0,0]
    x_com_ref[0,1] = x_com_ref[0,1]
    x_com_ref[0,2] = x_com_ref[0,2]
    
    # generate CoM trajectory
#    margin_x = 0.001*0
#    margin_y = 0.001*30
#    com_data = com_motion_generation(robot, q_tmp, ContactState, Nframes_com, margin_x, margin_y)
    x_com_des = robot.CoM(q_data_init,ContactState)       
    com_data = com_motion_des(robot,q_tmp,ContactState,Nframes_com,x_com_des)
    
    Nframes = Nframes_com
    
    # waist angle back to init
    q_waist = CosTraj(Nframes_com,q_tmp[robot.J_HPY,0],0)[0]
    
    err_thresh = 0.001
    #qdata = vstack((qdata,q_tmp.T))
    
    #tic = time.clock()
    for I in range(Nframes):
        x_com_ref = mat(com_data[I,]).T
        
        error = 10000
        cnt = 1
        while norm(error) > err_thresh:
            x_W = tr2diff(robot.FK_tree(q_tmp, robot.F_HPY, ContactState))
            err_x_W = x_W_ref - x_W
            err_x_W = err_x_W[3:]
    
            x_swF = tr2diff(robot.FK_tree(q_tmp, F_swF, ContactState))
            err_x_swF = x_swF_ref - x_swF
            
            x_RH = tr2diff(robot.FK_tree(q_tmp, robot.F_RH, ContactState))
            err_x_RH = x_RH_ref - x_RH
            
            x_LH = tr2diff(robot.FK_tree(q_tmp, robot.F_LH, ContactState))
            err_x_LH = x_LH_ref - x_LH
            
            x_com = robot.CoM(q_tmp, ContactState)
            err_x_com = x_com_ref - x_com.T
    
            J_W = robot.J_tree(q_tmp, robot.F_HPY, ContactState)
            J_W = J_W[3:,]
            
            J_swF = robot.J_tree(q_tmp, F_swF, ContactState)
                        
            J_RH = robot.J_tree(q_tmp, robot.F_RH, ContactState)
            
            J_LH = robot.J_tree(q_tmp, robot.F_LH, ContactState)
            
            J_com = robot.J_com_tree(q_tmp, ContactState)
            J_com_legs = hstack((zeros((3,1)), J_com[:,robot.J_LHY-1:robot.J_RAR], zeros((3,14))))              
            
            Jacob = vstack((J_com_legs,J_swF,J_W))
            error = vstack((err_x_com,err_x_swF,err_x_W))
            
#            Jacob = vstack((J_com_legs,J_swF,J_W,J_LH,J_RH))
#            error = vstack((err_x_com,err_x_swF,err_x_W,err_x_LH,err_x_RH))
                       
            qd_tmp = Kp*pinv(Jacob)*error
            q_tmp = q_tmp + qd_tmp*tsamp
            
            q_tmp[robot.J_HPY,0] = q_waist[I]
            q_tmp = robot.joint_limit_check(q_tmp)
    
            cnt = cnt + 1
            
            print I
            print cnt
            print norm(error)
        #q_tmp[robot.J_RWR-1,0] = q_data_init[robot.J_RWR-1,0] + ang_data[0,I]
        qdata = vstack((qdata,q_tmp.T))
        REF_FRAMES = vstack((REF_FRAMES,[ContactState]))    
        
    #toc = time.clock()
    
    #print toc - tic
    ## generate end pose
    (Q, Qd, Qdd) = jtraj(qdata[-1:,:],zeros((1,robot.NJ)),60)
    #Q = Qdata[0] # joint position trajectory
    qdata = vstack((qdata,Q))
    REF_FRAMES = vstack((REF_FRAMES,ContactState*ones((60,1))))
    
    ## interpolation
    qdata1 = spline_interp(qdata, 100)    
    #print qdata1
    
    ## send the motion to Hubo-Ach
    #print "sending data to Hubo-Ach"
    #moveRobot(qdata1,0.005)    
    
    ## plot qdata
#    from Ani_Robot import *
#    Ani_robot(robot,qdata,REF_FRAMES)  



## 07-30-13 Foot Step Correction
# footstep position + orientation
if select == 9:
    import time
    #import robot
    from copy import copy, deepcopy
    #from construct_DRC_Hubo_fast import *
    from construct_DRC_Hubo import *
    from motion2ach_update import *
    #from Ani_Robot import *
#    from motion2ach import *
    
    robot = construct_DRC_Hubo()
    q_data = mat(zeros((robot.NJ,1)))
    
    q_data_init = q_data
    q_data_init[robot.J_HPY-1] = 0
    q_data_init[robot.J_LHP-1] = -pi/8
    q_data_init[robot.J_LKP-1] = pi/4
    q_data_init[robot.J_LAP-1] = -pi/8
    q_data_init[robot.J_RHP-1] = -pi/8
    q_data_init[robot.J_RKP-1] = pi/4
    q_data_init[robot.J_RAP-1] = -pi/8
    q_init = q_data_init
    
    q_data_init[robot.J_LEP-1] = -3*pi/4
    q_data_init[robot.J_REP-1] = -3*pi/4
    q_data_init[robot.J_LSR-1] = pi/12
    q_data_init[robot.J_RSR-1] = -pi/12
    
#    q_data_init[robot.J_RSP-1] = -1.42
#    q_data_init[robot.J_RSR-1] = -1.3782
#    q_data_init[robot.J_RSY-1] = -2.1016
#    q_data_init[robot.J_REP-1] = -2.0999
#    q_data_init[robot.J_RWY-1] = 0
#    q_data_init[robot.J_RWP-1] = 0
#    q_data_init[robot.J_RWR-1] = -0.24   
#    q_data_init[robot.J_LSP-1] = -1.42
#    q_data_init[robot.J_LSR-1] = 1.3782
#    q_data_init[robot.J_LSY-1] = 2.1016
#    q_data_init[robot.J_LEP-1] = -2.0999
#    q_data_init[robot.J_LWY-1] = 0
#    q_data_init[robot.J_LWP-1] = 0
#    q_data_init[robot.J_LWR-1] = 0.24   
    
    Kp = 100
    tsamp = 0.01
    err_thresh = 0.001    
    
    q_tmp = q_data_init 
    ContactState = robot.LF
    
    # CoM of the init pose 
    CoM_init = robot.CoM(q_init,ContactState)
    
    ## generate init pose
    # from zero configuration to init pose
    (Q, Qd, Qdd) = jtraj(zeros((1,robot.NJ)),q_data_init,60)
    qdata = Q
    
    REF_FRAMES = robot.LF*ones((60,1))
    
    ## static backward staircase climbing motion generation
    q_tmp = q_data_init
    ## first step - left foot support
    ### parameters
    steplength = - 0.001*0
    steplength_y = -0.001*50   
    stepheight = 0.001*75
    steplength_v = 0
    rotation_z = 0
    
    Nframes_step = 30
    Nframes_com = 40
    # contact state and swing foot
    ContactState = robot.LF
    F_swF = robot.F_RF
    
    ### current pose of the robot
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
    T_swF_ref[0,3] = T_swF_ref[0,3]# + 0.001*50;
    
    T_swF_ref[1,3] = T_swF_ref[1,3]
    T_swF_ref[2,3] = T_swF_ref[2,3]
    x_swF_ref = tr2diff(T_swF_ref)
    
    # right hand pose
    T_LH_ref = robot.FK_tree(q_tmp, robot.F_LH, ContactState)
    T_LH_init = T_LH_ref
    T_LH_ref[0,3] = T_LH_ref[0,3]# + 0.001*50;
    T_LH_ref[1,3] = T_LH_ref[1,3]
    T_LH_ref[2,3] = T_LH_ref[2,3]
    x_LH_ref = tr2diff(T_LH_ref)
    
    # left hand pose
    T_RH_ref = robot.FK_tree(q_tmp, robot.F_RH, ContactState)
    T_RH_init = T_RH_ref
    T_RH_ref[0,3] = T_RH_ref[0,3]# + 0.001*50;
    T_RH_ref[1,3] = T_RH_ref[1,3]
    T_RH_ref[2,3] = T_RH_ref[2,3]
    x_RH_ref = tr2diff(T_RH_ref)
    
    # CoM location
    x_com_ref = robot.CoM(q_tmp, ContactState)
    x_com_ref[0,0] = x_com_ref[0,0]
    x_com_ref[0,1] = x_com_ref[0,1]
    x_com_ref[0,2] = x_com_ref[0,2]
    
    # generate CoM trajectory
    margin_x = 0.001*0
    margin_y = 0.001*40
    com_data = com_motion_generation(robot, q_tmp, ContactState, Nframes_com, margin_x, margin_y)

    # generate footstep angle data
    ang_data = hstack((zeros((Nframes_com)),CosTraj(Nframes_step,0,deg2rad(rotation_z))[0]))
    
    # generate swing foot stepping motion 
    stepcollision = 0.001*0
    #N_collision = 10
    Tdata_swF_tmp = stepping_motion_generation(robot, q_tmp, ContactState, steplength, stepheight, Nframes_step, steplength_v, 1, stepcollision, steplength_y)
    
    ### pad trajectories 
    # com trajectory
    com_tmp = com_data[-1:,]
    for i in range(Nframes_step):
        com_data = vstack((com_data,com_tmp))
    
    Tdata_swF = zeros((4,4,Nframes_com+Nframes_step))
    # swing foot trajectory
    for i in range(Nframes_com):
        Tdata_swF[:,:,i] = T_swF_init
    
    for i in range(Nframes_com, Nframes_com + Nframes_step):
        Tdata_swF[:,:,i] = Tdata_swF_tmp[:,:,i-Nframes_com]
           
    Nframes = Nframes_com + Nframes_step
    
    err_thresh = 0.01
    #qdata = vstack((qdata,q_tmp.T))
    
    #tic = time.clock()
    for I in range(Nframes):
        x_com_ref = mat(com_data[I,]).T
        
        T_swF_ref = Tdata_swF[:,:,I]
        x_swF_ref = tr2diff(T_swF_ref)
        
        error = 10000
        cnt = 1
        while norm(error) > err_thresh:
            x_W = tr2diff(robot.FK_tree(q_tmp, robot.F_HPY, ContactState))
            err_x_W = x_W_ref - x_W
            err_x_W = err_x_W[3:]
    
            x_swF = tr2diff(robot.FK_tree(q_tmp, F_swF, ContactState))
            err_x_swF = x_swF_ref - x_swF
            
            x_RH = tr2diff(robot.FK_tree(q_tmp, robot.F_RH, ContactState))
            err_x_RH = x_RH_ref - x_RH
            
            x_LH = tr2diff(robot.FK_tree(q_tmp, robot.F_LH, ContactState))
            err_x_LH = x_LH_ref - x_LH
            
            x_com = robot.CoM(q_tmp, ContactState)
            err_x_com = x_com_ref - x_com.T
    
            J_W = robot.J_tree(q_tmp, robot.F_HPY, ContactState)
            J_W = J_W[3:,]
            
            J_swF = robot.J_tree(q_tmp, F_swF, ContactState)
                        
            J_RH = robot.J_tree(q_tmp, robot.F_RH, ContactState)
            
            J_LH = robot.J_tree(q_tmp, robot.F_LH, ContactState)
            
            J_com = robot.J_com_tree(q_tmp, ContactState)
            J_com_legs = hstack((zeros((3,1)), J_com[:,robot.J_LHY-1:robot.J_RAR], zeros((3,14))))              
            
            Jacob = vstack((J_com_legs[:2,],J_swF,J_W))
            error = vstack((err_x_com[:2,],err_x_swF,err_x_W))
            
#            Jacob = vstack((J_com_legs[:2,],J_swF,J_W,J_LH,J_RH))
#            error = vstack((err_x_com[:2,],err_x_swF,err_x_W,err_x_LH,err_x_RH))
            
            qd_tmp = Kp*pinv(Jacob)*error
            q_tmp = q_tmp + qd_tmp*tsamp
            q_tmp = robot.joint_limit_check(q_tmp)
    
            cnt = cnt + 1
            
            print I
            #print cnt
            #print norm(error)
        q_tmp1 = deepcopy(q_tmp)
        q_tmp1[robot.J_RHY-1,0] = ang_data[I]
        qdata = vstack((qdata,q_tmp1.T))
        REF_FRAMES = vstack((REF_FRAMES,[ContactState]))
        
    ## second step - right foot support
    q_tmp[robot.J_RHY-1,0] = 0
    
    ### parameters
    steplength = -0.001*0
    steplength_y = -0.001*50    
    stepheight = 0.001*75
    steplength_v = 0
    
    Nframes_step = 30
    Nframes_com = 30
    # contact state and swing foot
    ContactState = robot.RF
    F_swF = robot.F_LF
    
    ### current pose of the robot
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
    T_swF_ref[0,3] = T_swF_ref[0,3]# + 0.001*50;
    T_swF_ref[1,3] = T_swF_ref[1,3]
    T_swF_ref[2,3] = T_swF_ref[2,3]
    x_swF_ref = tr2diff(T_swF_ref)
    
    # right hand pose
    T_LH_ref = robot.FK_tree(q_tmp, robot.F_LH, ContactState)
    T_LH_init = T_LH_ref
    T_LH_ref[0,3] = T_LH_ref[0,3]# + 0.001*50;
    T_LH_ref[1,3] = T_LH_ref[1,3]
    T_LH_ref[2,3] = T_LH_ref[2,3]
    x_LH_ref = tr2diff(T_LH_ref)
    
    # left hand pose
    T_RH_ref = robot.FK_tree(q_tmp, robot.F_RH, ContactState)
    T_RH_init = T_RH_ref
    T_RH_ref[0,3] = T_RH_ref[0,3]# + 0.001*50;
    T_RH_ref[1,3] = T_RH_ref[1,3]
    T_RH_ref[2,3] = T_RH_ref[2,3]
    x_RH_ref = tr2diff(T_RH_ref)
    
    # CoM location
    x_com_ref = robot.CoM(q_tmp, ContactState)
    x_com_ref[0,0] = x_com_ref[0,0]
    x_com_ref[0,1] = x_com_ref[0,1]
    x_com_ref[0,2] = x_com_ref[0,2]
    
    # generate CoM trajectory
    margin_x = 0.001*0
    margin_y = 0.001*40
    com_data = com_motion_generation(robot, q_tmp, ContactState, Nframes_com, margin_x, margin_y)
       
    # generate swing foot stepping motion 
    stepcollision = 0.001*0
    #N_collision = 10
    Tdata_swF_tmp = stepping_motion_generation(robot, q_tmp, ContactState, steplength, stepheight, Nframes_step, steplength_v, 1, stepcollision, steplength_y)
    
    # generate footstep angle data
    ang_data = hstack((deg2rad(rotation_z)*ones((Nframes_com)),CosTraj(Nframes_step,deg2rad(rotation_z),0)[0]))    
    
    ### pad trajectories 
    # com trajectory
    com_tmp = com_data[-1:,]
    for i in range(Nframes_step):
        com_data = vstack((com_data,com_tmp))
    
    Tdata_swF = zeros((4,4,Nframes_com+Nframes_step))
    # swing foot trajectory
    for i in range(Nframes_com):
        Tdata_swF[:,:,i] = T_swF_init
    
    for i in range(Nframes_com, Nframes_com + Nframes_step):
        Tdata_swF[:,:,i] = Tdata_swF_tmp[:,:,i-Nframes_com]
           
    Nframes = Nframes_com + Nframes_step
    
    err_thresh = 0.01
    #qdata = vstack((qdata,q_tmp.T))
    
    #tic = time.clock()
    for I in range(Nframes):
        x_com_ref = mat(com_data[I,]).T
        
        T_swF_ref = Tdata_swF[:,:,I]
        x_swF_ref = tr2diff(T_swF_ref)
        
        error = 10000
        cnt = 1
        while norm(error) > err_thresh:
            x_W = tr2diff(robot.FK_tree(q_tmp, robot.F_HPY, ContactState))
            err_x_W = x_W_ref - x_W
            err_x_W = err_x_W[3:]
    
            x_swF = tr2diff(robot.FK_tree(q_tmp, F_swF, ContactState))
            err_x_swF = x_swF_ref - x_swF
            
            x_RH = tr2diff(robot.FK_tree(q_tmp, robot.F_RH, ContactState))
            err_x_RH = x_RH_ref - x_RH
            
            x_LH = tr2diff(robot.FK_tree(q_tmp, robot.F_LH, ContactState))
            err_x_LH = x_LH_ref - x_LH
            
            x_com = robot.CoM(q_tmp, ContactState)
            err_x_com = x_com_ref - x_com.T
    
            J_W = robot.J_tree(q_tmp, robot.F_HPY, ContactState)
            J_W = J_W[3:,]
            
            J_swF = robot.J_tree(q_tmp, F_swF, ContactState)
                        
            J_RH = robot.J_tree(q_tmp, robot.F_RH, ContactState)
            
            J_LH = robot.J_tree(q_tmp, robot.F_LH, ContactState)
            
            J_com = robot.J_com_tree(q_tmp, ContactState)
            J_com_legs = hstack((zeros((3,1)), J_com[:,robot.J_LHY-1:robot.J_RAR], zeros((3,14))))              
            
            Jacob = vstack((J_com_legs[:2,],J_swF,J_W))
            error = vstack((err_x_com[:2,],err_x_swF,err_x_W))
            
#            Jacob = vstack((J_com_legs[:2,],J_swF,J_W,J_LH,J_RH))
#            error = vstack((err_x_com[:2,],err_x_swF,err_x_W,err_x_LH,err_x_RH))
                       
            qd_tmp = Kp*pinv(Jacob)*error
            q_tmp = q_tmp + qd_tmp*tsamp
            q_tmp = robot.joint_limit_check(q_tmp)
    
            cnt = cnt + 1
            
            print I
            #print cnt
            #print norm(error)
        q_tmp1 = deepcopy(q_tmp)
        q_tmp1[robot.J_RHY-1,0] = ang_data[I]
        qdata = vstack((qdata,q_tmp1.T))  
        REF_FRAMES = vstack((REF_FRAMES,[ContactState]))    
    
    ## third step - move the CoM to the center
    ### parameters
    #Nframes_step = 30
    Nframes_com = 30
    # contact state and swing foot
    ContactState = robot.RF
    F_swF = robot.F_LF
    
    ### current pose of the robot
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
    T_swF_ref[0,3] = T_swF_ref[0,3]# + 0.001*50;
    T_swF_ref[1,3] = T_swF_ref[1,3]
    T_swF_ref[2,3] = T_swF_ref[2,3]
    x_swF_ref = tr2diff(T_swF_ref)
    
    # right hand pose
    T_LH_ref = robot.FK_tree(q_tmp, robot.F_LH, ContactState)
    T_LH_init = T_LH_ref
    T_LH_ref[0,3] = T_LH_ref[0,3]# + 0.001*50;
    T_LH_ref[1,3] = T_LH_ref[1,3]
    T_LH_ref[2,3] = T_LH_ref[2,3]
    x_LH_ref = tr2diff(T_LH_ref)
    
    # left hand pose
    T_RH_ref = robot.FK_tree(q_tmp, robot.F_RH, ContactState)
    T_RH_init = T_RH_ref
    T_RH_ref[0,3] = T_RH_ref[0,3]# + 0.001*50;
    T_RH_ref[1,3] = T_RH_ref[1,3]
    T_RH_ref[2,3] = T_RH_ref[2,3]
    x_RH_ref = tr2diff(T_RH_ref)
    
    # CoM location
    x_com_ref = robot.CoM(q_tmp, ContactState)
    x_com_ref[0,0] = x_com_ref[0,0]
    x_com_ref[0,1] = x_com_ref[0,1]
    x_com_ref[0,2] = x_com_ref[0,2]
    
    # generate CoM trajectory
#    margin_x = 0.001*0
#    margin_y = 0.001*30
#    com_data = com_motion_generation(robot, q_tmp, ContactState, Nframes_com, margin_x, margin_y)
    x_com_des = robot.CoM(q_data_init,ContactState)       
    com_data = com_motion_des(robot,q_tmp,ContactState,Nframes_com,x_com_des)
    
    Nframes = Nframes_com
    
    # waist angle back to init
    q_waist = CosTraj(Nframes_com,q_tmp[robot.J_HPY,0],0)[0]
    
    err_thresh = 0.01
    #qdata = vstack((qdata,q_tmp.T))
    
    #tic = time.clock()
    for I in range(Nframes):
        x_com_ref = mat(com_data[I,]).T
        
        error = 10000
        cnt = 1
        while norm(error) > err_thresh:
            x_W = tr2diff(robot.FK_tree(q_tmp, robot.F_HPY, ContactState))
            err_x_W = x_W_ref - x_W
            err_x_W = err_x_W[3:]
    
            x_swF = tr2diff(robot.FK_tree(q_tmp, F_swF, ContactState))
            err_x_swF = x_swF_ref - x_swF
            
            x_RH = tr2diff(robot.FK_tree(q_tmp, robot.F_RH, ContactState))
            err_x_RH = x_RH_ref - x_RH
            
            x_LH = tr2diff(robot.FK_tree(q_tmp, robot.F_LH, ContactState))
            err_x_LH = x_LH_ref - x_LH
            
            x_com = robot.CoM(q_tmp, ContactState)
            err_x_com = x_com_ref - x_com.T
    
            J_W = robot.J_tree(q_tmp, robot.F_HPY, ContactState)
            J_W = J_W[3:,]
            
            J_swF = robot.J_tree(q_tmp, F_swF, ContactState)
                        
            J_RH = robot.J_tree(q_tmp, robot.F_RH, ContactState)
            
            J_LH = robot.J_tree(q_tmp, robot.F_LH, ContactState)
            
            J_com = robot.J_com_tree(q_tmp, ContactState)
            J_com_legs = hstack((zeros((3,1)), J_com[:,robot.J_LHY-1:robot.J_RAR], zeros((3,14))))              
            
            Jacob = vstack((J_com_legs,J_swF,J_W))
            error = vstack((err_x_com,err_x_swF,err_x_W))
            
#            Jacob = vstack((J_com_legs,J_swF,J_W,J_LH,J_RH))
#            error = vstack((err_x_com,err_x_swF,err_x_W,err_x_LH,err_x_RH))
                       
            qd_tmp = Kp*pinv(Jacob)*error
            q_tmp = q_tmp + qd_tmp*tsamp
            
            q_tmp[robot.J_HPY,0] = q_waist[I]
            q_tmp = robot.joint_limit_check(q_tmp)
    
            cnt = cnt + 1
            
            print I
            #print cnt
            #print norm(error)
        #q_tmp[robot.J_RWR-1,0] = q_data_init[robot.J_RWR-1,0] + ang_data[0,I]
        qdata = vstack((qdata,q_tmp.T))
        REF_FRAMES = vstack((REF_FRAMES,[ContactState]))    
        
    #toc = time.clock()
    
    #print toc - tic
    ## generate end pose
    (Q, Qd, Qdd) = jtraj(qdata[-1:,:],zeros((1,robot.NJ)),60)
    #Q = Qdata[0] # joint position trajectory
    qdata = vstack((qdata,Q))
    REF_FRAMES = vstack((REF_FRAMES,ContactState*ones((60,1))))
    
    ## interpolation
    qdata1 = spline_interp(qdata, 100)    
    #print qdata1
    
    ## send the motion to Hubo-Ach
    #print "sending data to Hubo-Ach"
    #moveRobot(qdata1,0.005)    

#    ### store data    
#    import scipy.io
#    data = {}
#    data['qdata1'] = qdata1
#    scipy.io.savemat('backward_walking_080113.mat',data)
#    
#    ### read from mat file
#    data = scipy.io.loadmat('backward_walking_080113.mat')
#    qdata = data['qdata']
    
    ## plot qdata
#    from Ani_Robot import *
#    Ani_robot(robot,qdata,REF_FRAMES)  

## 08-02-13 Foot Step Correction
# footstep position + orientation
if select == 10:
    import time
    #import robot
    from copy import copy, deepcopy
    from construct_DRC_Hubo_fast import *
    from motion2ach_update import *
    #from Ani_Robot import *
#    from motion2ach import *
    
    robot = construct_DRC_Hubo()
    q_data = mat(zeros((robot.NJ,1)))
    
    q_data_init = q_data
    q_data_init[robot.J_HPY-1] = 0
    q_data_init[robot.J_LHP-1] = -pi/8
    q_data_init[robot.J_LKP-1] = pi/4
    q_data_init[robot.J_LAP-1] = -pi/8
    q_data_init[robot.J_RHP-1] = -pi/8
    q_data_init[robot.J_RKP-1] = pi/4
    q_data_init[robot.J_RAP-1] = -pi/8
    q_init = q_data_init
    
    q_data_init[robot.J_LEP-1] = -3*pi/4
    q_data_init[robot.J_REP-1] = -3*pi/4
    q_data_init[robot.J_LSR-1] = pi/12
    q_data_init[robot.J_RSR-1] = -pi/12
    
#    q_data_init[robot.J_RSP-1] = -1.42
#    q_data_init[robot.J_RSR-1] = -1.3782
#    q_data_init[robot.J_RSY-1] = -2.1016
#    q_data_init[robot.J_REP-1] = -2.0999
#    q_data_init[robot.J_RWY-1] = 0
#    q_data_init[robot.J_RWP-1] = 0
#    q_data_init[robot.J_RWR-1] = -0.24   
#    q_data_init[robot.J_LSP-1] = -1.42
#    q_data_init[robot.J_LSR-1] = 1.3782
#    q_data_init[robot.J_LSY-1] = 2.1016
#    q_data_init[robot.J_LEP-1] = -2.0999
#    q_data_init[robot.J_LWY-1] = 0
#    q_data_init[robot.J_LWP-1] = 0
#    q_data_init[robot.J_LWR-1] = 0.24   
    
    Kp = 100
    tsamp = 0.01
    err_thresh = 0.001    
    
    q_tmp = q_data_init 
    ContactState = robot.LF
    
    # CoM of the init pose 
    CoM_init = robot.CoM(q_init,ContactState)
    
    ## generate init pose
    # from zero configuration to init pose
    (Q, Qd, Qdd) = jtraj(zeros((1,robot.NJ)),q_data_init,60)
    qdata = Q
    
    REF_FRAMES = robot.LF*ones((60,1))
    
    ## static backward staircase climbing motion generation
    q_tmp = q_data_init
    ## first step - left foot support
    ### parameters
    steplength = - 0.001*50
    steplength_y = -0.001*50   
    stepheight = 0.001*75
    steplength_v = 0
    rotation_z = 0
    
    Nframes_step = 30
    Nframes_com = 40
    # contact state and swing foot
    ContactState = robot.LF
    F_swF = robot.F_RF
    
    ### current pose of the robot
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
    T_swF_ref[0,3] = T_swF_ref[0,3]# + 0.001*50;
    
    T_swF_ref[1,3] = T_swF_ref[1,3]
    T_swF_ref[2,3] = T_swF_ref[2,3]
    x_swF_ref = tr2diff(T_swF_ref)
    
    # right hand pose
    T_LH_ref = robot.FK_tree(q_tmp, robot.F_LH, ContactState)
    T_LH_init = T_LH_ref
    T_LH_ref[0,3] = T_LH_ref[0,3]# + 0.001*50;
    T_LH_ref[1,3] = T_LH_ref[1,3]
    T_LH_ref[2,3] = T_LH_ref[2,3]
    x_LH_ref = tr2diff(T_LH_ref)
    
    # left hand pose
    T_RH_ref = robot.FK_tree(q_tmp, robot.F_RH, ContactState)
    T_RH_init = T_RH_ref
    T_RH_ref[0,3] = T_RH_ref[0,3]# + 0.001*50;
    T_RH_ref[1,3] = T_RH_ref[1,3]
    T_RH_ref[2,3] = T_RH_ref[2,3]
    x_RH_ref = tr2diff(T_RH_ref)
    
    # CoM location
    x_com_ref = robot.CoM(q_tmp, ContactState)
    x_com_ref[0,0] = x_com_ref[0,0]
    x_com_ref[0,1] = x_com_ref[0,1]
    x_com_ref[0,2] = x_com_ref[0,2]
    
    # generate CoM trajectory
    margin_x = 0.001*0
    margin_y = 0.001*20
    com_data = com_motion_generation(robot, q_tmp, ContactState, Nframes_com, margin_x, margin_y)

    # generate footstep angle data
    ang_data = hstack((zeros((Nframes_com)),CosTraj(Nframes_step,0,deg2rad(rotation_z))[0]))
    
    # generate swing foot stepping motion 
    stepcollision = 0.001*0
    N_col = 10
    Tdata_swF_tmp = stepping_motion_generation(robot, q_tmp, ContactState, steplength, stepheight, Nframes_step, steplength_v, 1, stepcollision, steplength_y, N_col, rotation_z)
    
    ### pad trajectories 
    # com trajectory
    com_tmp = com_data[-1:,]
    for i in range(Nframes_step):
        com_data = vstack((com_data,com_tmp))
    
    Tdata_swF = zeros((4,4,Nframes_com+Nframes_step))
    # swing foot trajectory
    for i in range(Nframes_com):
        Tdata_swF[:,:,i] = T_swF_init
    
    for i in range(Nframes_com, Nframes_com + Nframes_step):
        Tdata_swF[:,:,i] = Tdata_swF_tmp[:,:,i-Nframes_com]
           
    Nframes = Nframes_com + Nframes_step
    
    err_thresh = 0.01
    #qdata = vstack((qdata,q_tmp.T))
    
    #tic = time.clock()
    for I in range(Nframes):
        x_com_ref = mat(com_data[I,]).T
        
        T_swF_ref = Tdata_swF[:,:,I]
        x_swF_ref = tr2diff(T_swF_ref)
        
        error = 10000
        cnt = 1
        while norm(error) > err_thresh:
            x_W = tr2diff(robot.FK_tree(q_tmp, robot.F_HPY, ContactState))
            err_x_W = x_W_ref - x_W
            err_x_W = err_x_W[3:]
    
            x_swF = tr2diff(robot.FK_tree(q_tmp, F_swF, ContactState))
            err_x_swF = x_swF_ref - x_swF
            
            x_RH = tr2diff(robot.FK_tree(q_tmp, robot.F_RH, ContactState))
            err_x_RH = x_RH_ref - x_RH
            
            x_LH = tr2diff(robot.FK_tree(q_tmp, robot.F_LH, ContactState))
            err_x_LH = x_LH_ref - x_LH
            
            x_com = robot.CoM(q_tmp, ContactState)
            err_x_com = x_com_ref - x_com.T
    
            J_W = robot.J_tree(q_tmp, robot.F_HPY, ContactState)
            J_W = J_W[3:,]
            
            J_swF = robot.J_tree(q_tmp, F_swF, ContactState)
                        
            J_RH = robot.J_tree(q_tmp, robot.F_RH, ContactState)
            
            J_LH = robot.J_tree(q_tmp, robot.F_LH, ContactState)
            
            J_com = robot.J_com_tree(q_tmp, ContactState)
            J_com_legs = hstack((zeros((3,1)), J_com[:,robot.J_LHY-1:robot.J_RAR], zeros((3,14))))              
            
            Jacob = vstack((J_com_legs[:2,],J_swF,J_W[:2,]))
            error = vstack((err_x_com[:2,],err_x_swF,err_x_W[:2]))
            
#            Jacob = vstack((J_com_legs[:2,],J_swF,J_W,J_LH,J_RH))
#            error = vstack((err_x_com[:2,],err_x_swF,err_x_W,err_x_LH,err_x_RH))
            
            qd_tmp = Kp*pinv(Jacob)*error
            q_tmp = q_tmp + qd_tmp*tsamp
            q_tmp = robot.joint_limit_check(q_tmp)
    
            cnt = cnt + 1
            
            print I
            #print cnt
            #print norm(error)
        q_tmp1 = deepcopy(q_tmp)
        #q_tmp1[robot.J_RHY-1,0] = ang_data[I]
        qdata = vstack((qdata,q_tmp1.T))
        REF_FRAMES = vstack((REF_FRAMES,[ContactState]))
        
    ## second step - right foot support
    #q_tmp[robot.J_RHY-1,0] = 0
    
    ### parameters
    steplength = -0.001*50
    steplength_y = -0.001*50    
    stepheight = 0.001*75
    steplength_v = 0
    rotation_z = 0
    
    Nframes_step = 30
    Nframes_com = 30
    # contact state and swing foot
    ContactState = robot.RF
    F_swF = robot.F_LF
    
    ### current pose of the robot
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
    T_swF_ref[0,3] = T_swF_ref[0,3]# + 0.001*50;
    T_swF_ref[1,3] = T_swF_ref[1,3]
    T_swF_ref[2,3] = T_swF_ref[2,3]
    x_swF_ref = tr2diff(T_swF_ref)
    
    # right hand pose
    T_LH_ref = robot.FK_tree(q_tmp, robot.F_LH, ContactState)
    T_LH_init = T_LH_ref
    T_LH_ref[0,3] = T_LH_ref[0,3]# + 0.001*50;
    T_LH_ref[1,3] = T_LH_ref[1,3]
    T_LH_ref[2,3] = T_LH_ref[2,3]
    x_LH_ref = tr2diff(T_LH_ref)
    
    # left hand pose
    T_RH_ref = robot.FK_tree(q_tmp, robot.F_RH, ContactState)
    T_RH_init = T_RH_ref
    T_RH_ref[0,3] = T_RH_ref[0,3]# + 0.001*50;
    T_RH_ref[1,3] = T_RH_ref[1,3]
    T_RH_ref[2,3] = T_RH_ref[2,3]
    x_RH_ref = tr2diff(T_RH_ref)
    
    # CoM location
    x_com_ref = robot.CoM(q_tmp, ContactState)
    x_com_ref[0,0] = x_com_ref[0,0]
    x_com_ref[0,1] = x_com_ref[0,1]
    x_com_ref[0,2] = x_com_ref[0,2]
    
    # generate CoM trajectory
    margin_x = 0.001*0
    margin_y = 0.001*20
    com_data = com_motion_generation(robot, q_tmp, ContactState, Nframes_com, margin_x, margin_y)
       
    # generate swing foot stepping motion 
    stepcollision = 0.001*0
    N_col = 10
    Tdata_swF_tmp = stepping_motion_generation(robot, q_tmp, ContactState, steplength, stepheight, Nframes_step, steplength_v, 1, stepcollision, steplength_y, N_col, rotation_z)
    
    # generate footstep angle data
    ang_data = hstack((deg2rad(rotation_z)*ones((Nframes_com)),CosTraj(Nframes_step,deg2rad(rotation_z),0)[0]))    
    
    ### pad trajectories 
    # com trajectory
    com_tmp = com_data[-1:,]
    for i in range(Nframes_step):
        com_data = vstack((com_data,com_tmp))
    
    Tdata_swF = zeros((4,4,Nframes_com+Nframes_step))
    # swing foot trajectory
    for i in range(Nframes_com):
        Tdata_swF[:,:,i] = T_swF_init
    
    for i in range(Nframes_com, Nframes_com + Nframes_step):
        Tdata_swF[:,:,i] = Tdata_swF_tmp[:,:,i-Nframes_com]
           
    Nframes = Nframes_com + Nframes_step
    
    err_thresh = 0.01
    #qdata = vstack((qdata,q_tmp.T))
    
    #tic = time.clock()
    for I in range(Nframes):
        x_com_ref = mat(com_data[I,]).T
        
        T_swF_ref = Tdata_swF[:,:,I]
        x_swF_ref = tr2diff(T_swF_ref)
        
        error = 10000
        cnt = 1
        while norm(error) > err_thresh:
            x_W = tr2diff(robot.FK_tree(q_tmp, robot.F_HPY, ContactState))
            err_x_W = x_W_ref - x_W
            err_x_W = err_x_W[3:]
    
            x_swF = tr2diff(robot.FK_tree(q_tmp, F_swF, ContactState))
            err_x_swF = x_swF_ref - x_swF
            
            x_RH = tr2diff(robot.FK_tree(q_tmp, robot.F_RH, ContactState))
            err_x_RH = x_RH_ref - x_RH
            
            x_LH = tr2diff(robot.FK_tree(q_tmp, robot.F_LH, ContactState))
            err_x_LH = x_LH_ref - x_LH
            
            x_com = robot.CoM(q_tmp, ContactState)
            err_x_com = x_com_ref - x_com.T
    
            J_W = robot.J_tree(q_tmp, robot.F_HPY, ContactState)
            J_W = J_W[3:,]
            
            J_swF = robot.J_tree(q_tmp, F_swF, ContactState)
                        
            J_RH = robot.J_tree(q_tmp, robot.F_RH, ContactState)
            
            J_LH = robot.J_tree(q_tmp, robot.F_LH, ContactState)
            
            J_com = robot.J_com_tree(q_tmp, ContactState)
            J_com_legs = hstack((zeros((3,1)), J_com[:,robot.J_LHY-1:robot.J_RAR], zeros((3,14))))              
            
            Jacob = vstack((J_com_legs[:2,],J_swF,J_W[:2,]))
            error = vstack((err_x_com[:2,],err_x_swF,err_x_W[:2]))
            
#            Jacob = vstack((J_com_legs[:2,],J_swF,J_W,J_LH,J_RH))
#            error = vstack((err_x_com[:2,],err_x_swF,err_x_W,err_x_LH,err_x_RH))
                       
            qd_tmp = Kp*pinv(Jacob)*error
            q_tmp = q_tmp + qd_tmp*tsamp
            q_tmp = robot.joint_limit_check(q_tmp)
    
            cnt = cnt + 1
            
            print I
            #print cnt
            #print norm(error)
        q_tmp1 = deepcopy(q_tmp)
        #q_tmp1[robot.J_RHY-1,0] = ang_data[I]
        qdata = vstack((qdata,q_tmp1.T))  
        REF_FRAMES = vstack((REF_FRAMES,[ContactState]))    
    
    ## third step - move the CoM to the center
    ### parameters
    #Nframes_step = 30
    Nframes_com = 30
    # contact state and swing foot
    ContactState = robot.RF
    F_swF = robot.F_LF
    
    ### current pose of the robot
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
    T_swF_ref[0,3] = T_swF_ref[0,3]# + 0.001*50;
    T_swF_ref[1,3] = T_swF_ref[1,3]
    T_swF_ref[2,3] = T_swF_ref[2,3]
    x_swF_ref = tr2diff(T_swF_ref)
    
    # right hand pose
    T_LH_ref = robot.FK_tree(q_tmp, robot.F_LH, ContactState)
    T_LH_init = T_LH_ref
    T_LH_ref[0,3] = T_LH_ref[0,3]# + 0.001*50;
    T_LH_ref[1,3] = T_LH_ref[1,3]
    T_LH_ref[2,3] = T_LH_ref[2,3]
    x_LH_ref = tr2diff(T_LH_ref)
    
    # left hand pose
    T_RH_ref = robot.FK_tree(q_tmp, robot.F_RH, ContactState)
    T_RH_init = T_RH_ref
    T_RH_ref[0,3] = T_RH_ref[0,3]# + 0.001*50;
    T_RH_ref[1,3] = T_RH_ref[1,3]
    T_RH_ref[2,3] = T_RH_ref[2,3]
    x_RH_ref = tr2diff(T_RH_ref)
    
    # CoM location
    x_com_ref = robot.CoM(q_tmp, ContactState)
    x_com_ref[0,0] = x_com_ref[0,0]
    x_com_ref[0,1] = x_com_ref[0,1]
    x_com_ref[0,2] = x_com_ref[0,2]
    
    # generate CoM trajectory
#    margin_x = 0.001*0
#    margin_y = 0.001*30
#    com_data = com_motion_generation(robot, q_tmp, ContactState, Nframes_com, margin_x, margin_y)
    x_com_des = robot.CoM(q_data_init,ContactState)       
    com_data = com_motion_des(robot,q_tmp,ContactState,Nframes_com,x_com_des)
    
    Nframes = Nframes_com
    
    # waist angle back to init
    q_waist = CosTraj(Nframes_com,q_tmp[robot.J_HPY,0],0)[0]
    
    err_thresh = 0.01
    #qdata = vstack((qdata,q_tmp.T))
    
    #tic = time.clock()
    for I in range(Nframes):
        x_com_ref = mat(com_data[I,]).T
        
        error = 10000
        cnt = 1
        while norm(error) > err_thresh:
            x_W = tr2diff(robot.FK_tree(q_tmp, robot.F_HPY, ContactState))
            err_x_W = x_W_ref - x_W
            err_x_W = err_x_W[3:]
    
            x_swF = tr2diff(robot.FK_tree(q_tmp, F_swF, ContactState))
            err_x_swF = x_swF_ref - x_swF
            
            x_RH = tr2diff(robot.FK_tree(q_tmp, robot.F_RH, ContactState))
            err_x_RH = x_RH_ref - x_RH
            
            x_LH = tr2diff(robot.FK_tree(q_tmp, robot.F_LH, ContactState))
            err_x_LH = x_LH_ref - x_LH
            
            x_com = robot.CoM(q_tmp, ContactState)
            err_x_com = x_com_ref - x_com.T
    
            J_W = robot.J_tree(q_tmp, robot.F_HPY, ContactState)
            J_W = J_W[3:,]
            
            J_swF = robot.J_tree(q_tmp, F_swF, ContactState)
                        
            J_RH = robot.J_tree(q_tmp, robot.F_RH, ContactState)
            
            J_LH = robot.J_tree(q_tmp, robot.F_LH, ContactState)
            
            J_com = robot.J_com_tree(q_tmp, ContactState)
            J_com_legs = hstack((zeros((3,1)), J_com[:,robot.J_LHY-1:robot.J_RAR], zeros((3,14))))              
            
            Jacob = vstack((J_com_legs,J_swF,J_W))
            error = vstack((err_x_com,err_x_swF,err_x_W))
            
#            Jacob = vstack((J_com_legs,J_swF,J_W,J_LH,J_RH))
#            error = vstack((err_x_com,err_x_swF,err_x_W,err_x_LH,err_x_RH))
                       
            qd_tmp = Kp*pinv(Jacob)*error
            q_tmp = q_tmp + qd_tmp*tsamp
            
            q_tmp[robot.J_HPY,0] = q_waist[I]
            q_tmp = robot.joint_limit_check(q_tmp)
    
            cnt = cnt + 1
            
            print I
            #print cnt
            #print norm(error)
        #q_tmp[robot.J_RWR-1,0] = q_data_init[robot.J_RWR-1,0] + ang_data[0,I]
        qdata = vstack((qdata,q_tmp.T))
        REF_FRAMES = vstack((REF_FRAMES,[ContactState]))    
        
    #toc = time.clock()
    
    #print toc - tic
    ## generate end pose
    (Q, Qd, Qdd) = jtraj(qdata[-1:,:],zeros((1,robot.NJ)),60)
    #Q = Qdata[0] # joint position trajectory
    qdata = vstack((qdata,Q))
    REF_FRAMES = vstack((REF_FRAMES,ContactState*ones((60,1))))
    
    ## interpolation
    qdata1 = spline_interp(qdata, 100)    
    #print qdata1
    
    ## send the motion to Hubo-Ach
    print "sending data to Hubo-Ach"
    moveRobot(qdata1,0.004)    

#    ### store data    
#    import scipy.io
#    data = {}
#    data['qdata1'] = qdata1
#    scipy.io.savemat('backward_walking_080113.mat',data)
#    
#    ### read from mat file
#    data = scipy.io.loadmat('backward_walking_080113.mat')
#    qdata = data['qdata']
    
    ## plot qdata
#    from Ani_Robot import *
#    Ani_robot(robot,qdata,REF_FRAMES)  