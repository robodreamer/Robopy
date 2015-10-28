# -*- coding: utf-8 -*-
"""
07-16-2013 Construct a DRC-Hubo model
Written by Andy Park (Purdue University) on 07-23-13
"""
from numpy import *
from numpy.linalg import *
from sets import Set
from scipy.interpolate import interp1d
from matplotlib.mlab import find
from copy import copy, deepcopy



## Raised Cosine Interpolation
# written on 07-22-13
def CosTraj(N, ri, rf):
    data = linspace(0,N-1,N)
    N = float(N)    
    f = 0.5 - 0.5*cos(data*pi/(N-1))
    Traj = ri*ones((1,N)) + (rf-ri)*f
    return Traj


## Swing Leg Trajectory 
# given initial pos, maximum height, final height, generate a cycloid trajectory
# written on 07-30-13
def CycloidTraj(N, ri, r1, r2):
# r1: stepheight
# r2: steplength_vertical    
    
    Traj = zeros((N))
    Nmid = int(N/2)
    for i in range(Nmid):
#        n = i
        f = r1/2*(1 - cos(i*2*pi/(N-1)))
        
        Traj[i] = ri + f
        
    for i in range(Nmid,N):
#        n = i
        f = (float((ri+r1)-r2)/2)*(1 - cos(i*2*pi/(N-1)))
        
        Traj[i] = r2 + f 
    
    return Traj
    
## CoM motion generation
# written on 07-30-13
def com_motion_generation(robot, q_data, ContactState, N, margin_x, margin_y, mode = 0):
    CoM_init = robot.CoM(q_data, ContactState)
    CoM_des = deepcopy(CoM_init)
    CoM_des[0,0] = margin_x
    
    if ContactState == robot.LF:
        CoM_des[0,1] = -margin_y
    else:
        CoM_des[0,1] = +margin_y
        
    if mode == 0:
        xdata = CosTraj(N, CoM_init[0,0], CoM_des[0,0])        
    elif mode == 1: ## no movement in x direction
        xdata = CosTraj(N, CoM_init[0,0], CoM_init[0,0])
        
    # y direction
    ydata = CosTraj(N, CoM_init[0,1], CoM_des[0,1])
    
    # generate CoM trajectory
    com_data = zeros((N,3))
    
    for I in range(N):
        com_tmp = CoM_init
        com_tmp[0,0] = xdata[0,I]
        com_tmp[0,1] = ydata[0,I]
        com_data[I,] = com_tmp
    
    return com_data
    
## com_motion_desired
# written on 07-30-13
def com_motion_des(robot,q_data,ContactState,N,CoM_des):
    CoM_init = robot.CoM(q_data, ContactState)
    
    # xdata
    xdata = CosTraj(N,CoM_init[0,0],CoM_des[0,0])[0]
    
    # ydata
    ydata = CosTraj(N,CoM_init[0,1],CoM_des[0,1])[0]
    
    # zdata
    zdata = CosTraj(N,CoM_init[0,2],CoM_des[0,2])[0]
    
    # generate CoM trajectory
    com_data = zeros((N,3))
    
    for I in range(N):
        com_tmp = CoM_init
        com_tmp[0,0] = xdata[I]
        com_tmp[0,1] = ydata[I]
        com_tmp[0,2] = zdata[I]
        com_data[I,] = com_tmp
        
    return com_data
        
## Stepping motion generation
# written on 07-30-2013
def stepping_motion_generation(robot, q_data, ContactState, stepsize_h, stepheight, N, stepsize_v = 0, climbing_mode = 0, stepcollision = 0.001*50,  stepsize_y = 0.0,  N_col = 10, rotation_z = 0):
    Tdata = zeros((4,4,N))
        
    if ContactState == robot.LF:
        T_F_init = robot.FK_tree(q_data, robot.F_RF, ContactState)
    elif ContactState == robot.RF:
        T_F_init = robot.FK_tree(q_data, robot.F_LF, ContactState)
    
    xdata = zeros((N))
    if climbing_mode == 0:
        xdata = CosTraj(N,T_F_init[0,3],T_F_init[0,3] + stepsize_h)[0]
    else:
        # generate stepping motion in x direction avoiding the collision with the rung
        xdata[:N-(N_col)] = CycloidTraj(N-N_col, T_F_init[0,3], stepcollision, T_F_init[0,3] + stepsize_h)
        xdata[N-N_col:] = xdata[N-N_col-1]*ones((N_col))
     
    # side direction
    ydata = CosTraj(N,T_F_init[1,3],T_F_init[1,3] + stepsize_y)[0]
        
    # z direction
    zdata = CycloidTraj(N, T_F_init[2,3], stepheight, stepsize_v)

    # rotation data    
    ang_data = CosTraj(N,0,rotation_z)[0]
    
    # generate swing foot trajectory
    for I in range(N):
        T_F_tmp = deepcopy(T_F_init)
        R_F_tmp = t2r(T_F_tmp)
        T_F_tmp[0,3] = xdata[I]
        T_F_tmp[1,3] = ydata[I]
        T_F_tmp[2,3] = zdata[I]
        R_F_tmp = R_F_tmp*rotz(deg2rad(rotation_z))
        T_F_tmp[:3,:3] = R_F_tmp
        Tdata[:,:,I] = T_F_tmp
    
    return Tdata

## Cubic Spline Interpolation
# written on 07-22-13    
def spline_interp(input_data, N):
    row = size(input_data,0)
    col = size(input_data,1)
    #data = mat([])
    t = linspace(0,row-1,row)
    tnew = linspace(0,row-1,row*N)
    #for I in range(col):
    #    data_tmp = input_data[:,I]
    f = interp1d(t, input_data.T, kind = 'cubic')
    data = f(tnew)
    return data.T



def ishomog(tr):
    """
    True if C{tr} is a 4x4 homogeneous transform.
    
    @note: Only the dimensions are tested, not whether the rotation submatrix
    is orthonormal.
    
    @rtype: boolean
    """
    
    return tr.shape == (4,4)



def isrot(r):
    """
    True if C{tr} is a 3x3 matrix.
    
    @note: Only the dimensions are tested, not whether the matrix
    is orthonormal.
    
    @rtype: boolean
    """
    return r.shape == (3,3)


def isvec(v, l=3):
    """
    True if C{tr} is an l-vector.  
    
    @param v: object to test
    @type l: integer
    @param l: length of vector (default 3)
   
    @rtype: boolean
    """
    return v.shape == (l,1) or v.shape == (1,l) or v.shape == (l,)


def numcols(m):
    """
    Number of columns in a matrix.
    
    @type m: matrix
    @return: the number of columns in the matrix.
    return m.shape[1];
    """
    return m.shape[1];
    
def numrows(m):
    """
    Number of rows in a matrix.
    
    @type m: matrix
    @return: the number of rows in the matrix.
    return m.shape[1];
    """
    return m.shape[0];

################ vector operations

def unit(v):
    """
    Unit vector.
    
    @type v: vector
    @rtype: vector
    @return: unit-vector parallel to C{v}
    """
    return mat(v / linalg.norm(v))
    
def crossp(v1, v2):
    """
    Vector cross product.
    
    @note Differs from L{numpy.cross} in that vectors can be row or
    column.
    
    @type v1: 3-vector
    @type v2: 3-vector
    @rtype: 3-vector
    @return: Cross product M{v1 x v2}
    """
    v1=mat(v1)
    v2=mat(v2)
    v1=v1.reshape(3,1)
    v2=v2.reshape(3,1)
    v = matrix(zeros( (3,1) ))
    v[0] = v1[1]*v2[2] - v1[2]*v2[1]
    v[1] = v1[2]*v2[0] - v1[0]*v2[2]
    v[2] = v1[0]*v2[1] - v1[1]*v2[0]
    return v

################ misc support functions

def arg2array(arg):
    """
    Convert a 1-dimensional argument that is either a list, array or matrix to an 
    array.
    
    Useful for functions where the argument might be in any of these formats:::
            func(a)
            func(1,2,3)
            
            def func(*args):
                if len(args) == 1:
                    v = arg2array(arg[0]);
                elif len(args) == 3:
                    v = arg2array(args);
             .
             .
             .
    
    @rtype: array
    @return: Array equivalent to C{arg}.
    """
    if isinstance(arg, (matrix, ndarray)):
        s = arg.shape;
        if len(s) == 1:
            return array(arg);
        if min(s) == 1:
            return array(arg).flatten();

    elif isinstance(arg, list):
        return array(arg);

    elif isinstance(arg, (int, float, float32, float64)):
        return array([arg]);
        
    raise ValueError;
    
def rotx(theta):
    """
    Rotation about X-axis
    
    @type theta: number
    @param theta: the rotation angle
    @rtype: 3x3 orthonormal matrix
    @return: rotation about X-axis

    @see: L{roty}, L{rotz}, L{rotvec}
    """
    
    ct = cos(theta)
    st = sin(theta)
    return mat([[1,  0,    0],
            [0,  ct, -st],
            [0,  st,  ct]])

def roty(theta):
    """
    Rotation about Y-axis
    
    @type theta: number
    @param theta: the rotation angle
    @rtype: 3x3 orthonormal matrix
    @return: rotation about Y-axis

    @see: L{rotx}, L{rotz}, L{rotvec}
    """
    
    ct = cos(theta)
    st = sin(theta)

    return mat([[ct,   0,   st],
            [0,    1,    0],
            [-st,  0,   ct]])

def rotz(theta):
    """
    Rotation about Z-axis
    
    @type theta: number
    @param theta: the rotation angle
    @rtype: 3x3 orthonormal matrix
    @return: rotation about Z-axis

    @see: L{rotx}, L{roty}, L{rotvec}
    """
    
    ct = cos(theta)
    st = sin(theta)

    return mat([[ct,      -st,  0],
            [st,       ct,  0],
            [ 0,    0,  1]])

def trotx(theta):
    """
    Rotation about X-axis
    
    @type theta: number
    @param theta: the rotation angle
    @rtype: 4x4 homogeneous matrix
    @return: rotation about X-axis

    @see: L{troty}, L{trotz}, L{rotx}
    """
    return r2t(rotx(theta))

def troty(theta):
    """
    Rotation about Y-axis
    
    @type theta: number
    @param theta: the rotation angle
    @rtype: 4x4 homogeneous matrix
    @return: rotation about Y-axis

    @see: L{troty}, L{trotz}, L{roty}
    """
    return r2t(roty(theta))

def trotz(theta):
    """
    Rotation about Z-axis
    
    @type theta: number
    @param theta: the rotation angle
    @rtype: 4x4 homogeneous matrix
    @return: rotation about Z-axis

    @see: L{trotx}, L{troty}, L{rotz}
    """
    return r2t(rotz(theta))

def rotvec2r(theta, v):
    """
    Rotation about arbitrary axis.  Compute a rotation matrix representing
    a rotation of C{theta} about the vector C{v}.
    
    @type v: 3-vector
    @param v: rotation vector
    @type theta: number
    @param theta: the rotation angle
    @rtype: 3x3 orthonormal matrix
    @return: rotation

    @see: L{rotx}, L{roty}, L{rotz}
    """
    #v = arg2array(v);
    cth = cos(theta)
    sth = sin(theta)
    vth = 1-cth
    kx = v[0]
    ky = v[1]
    kz = v[2]
    R = mat([[kx*kx*vth+cth,      ky*kx*vth-kz*sth,   kz*kx*vth+ky*sth],\
        [kx*ky*vth+kz*sth,   ky*ky*vth+cth,         kz*ky*vth-kx*sth],\
        [kx*kz*vth-ky*sth,   ky*kz*vth+kx*sth,   kz*kz*vth+cth]])    
    return R    
    
#    r = mat([[ct,         -v[2]*st,    v[1]*st],\
#             [v[2]*st,          ct,   -v[0]*st],\
#             [-v[1]*st,  v[0]*st,           ct]])
#    return v*v.T*vt+r
#    kx = k(1); ky = k(2); kz = k(3);
#
#	R = [
#kx*kx*vth+cth      ky*kx*vth-kz*sth   kz*kx*vth+ky*sth
#kx*ky*vth+kz*sth   ky*ky*vth+cth          kz*ky*vth-kx*sth
#kx*kz*vth-ky*sth   ky*kz*vth+kx*sth   kz*kz*vth+cth
#	]

def rotvec2tr(theta, v):
    """
    Rotation about arbitrary axis.  Compute a rotation matrix representing
    a rotation of C{theta} about the vector C{v}.
    
    @type v: 3-vector
    @param v: rotation vector
    @type theta: number
    @param theta: the rotation angle
    @rtype: 4x4 homogeneous matrix
    @return: rotation

    @see: L{trotx}, L{troty}, L{trotz}
    """
    return r2t(rotvec2r(theta, v))


###################################### translational transform

def transl(x, y=None, z=None):
    """
    Create or decompose translational homogeneous transformations.
    
    Create a homogeneous transformation
    ===================================
    
        - T = transl(v)
        - T = transl(vx, vy, vz)
        
        The transformation is created with a unit rotation submatrix.
        The translational elements are set from elements of v which is
        a list, array or matrix, or from separate passed elements.
    
    Decompose a homogeneous transformation
    ======================================
    

        - v = transl(T)   
    
        Return the translation vector
    """
           
    if y==None and z==None:
            x=mat(x)
            try:
                    if ishomog(x):
                            return x[0:3,3].reshape(3,1)
                    else:
                            return concatenate((concatenate((eye(3),x.reshape(3,1)),1),mat([0,0,0,1])))
            except AttributeError:
                    n=len(x)
                    r = [[],[],[]]
                    for i in range(n):
                            r = concatenate((r,x[i][0:3,3]),1)
                    return r
    elif y!=None and z!=None:
            return concatenate((concatenate((eye(3),mat([x,y,z]).T),1),mat([0,0,0,1])))

def tr2diff(t1):
    """
    Convert a transform matrix to differential representation.
    Returns the 6-element differential motion required to move
    to T1 in base coordinates.
    
    @type t1: 4x4 homogeneous transform
    @param t1: final value
    @rtype: 6-vector
    @return: Differential motion [dx dy dz drx dry drz]
    @see: L{skew}
    """
    
    t1 = mat(t1)
    
    d = vstack((t1[0:3,3],
             0.5*(t1[2,1] - t1[1,2]),
             0.5*(t1[0,2] - t1[2,0]),
             0.5*(t1[1,0] - t1[0,1])))
    return d

def t2r(T):
    """
    Return rotational submatrix of a homogeneous transformation.
    @type T: 4x4 homogeneous transformation
    @param T: the transform matrix to convert
    @rtype: 3x3 orthonormal rotation matrix
    @return: rotation submatrix
    """    
    
    if ishomog(T)==False:
        error( 'input must be a homogeneous transform')
    return T[0:3,0:3]


def r2t(R):
    """
    Convert a 3x3 orthonormal rotation matrix to a 4x4 homogeneous transformation::
    
        T = | R 0 |
            | 0 1 |
            
    @type R: 3x3 orthonormal rotation matrix
    @param R: the rotation matrix to convert
    @rtype: 4x4 homogeneous matrix
    @return: homogeneous equivalent
    """
    
    return concatenate( (concatenate( (R, zeros((3,1))),1), mat([0,0,0,1])) )

def jtraj(q0, q1, tv, qd0=None, qd1=None):
    """
    Compute a joint space trajectory between points C{q0} and C{q1}.
    The number of points is the length of the given time vector C{tv}.  If
    {tv} is a scalar it is taken as the number of points.
    
    A 7th order polynomial is used with default zero boundary conditions for
    velocity and acceleration.  Non-zero boundary velocities can be
    optionally specified as C{qd0} and C{qd1}.
    
    As well as the trajectory, M{q{t}}, its first and second derivatives
    M{qd(t)} and M{qdd(t)} are also computed.  All three are returned as a tuple.
    Each of these is an M{m x n} matrix, with one row per time step, and
    one column per joint parameter.

    @type q0: m-vector
    @param q0: initial state
    @type q1: m-vector
    @param q1: final state
    @type tv: n-vector or scalar
    @param tv: time step vector or number of steps
    @type qd0: m-vector
    @param qd0: initial velocity (default 0)
    @type qd1: m-vector
    @param qd1: final velocity (default 0)
    @rtype: tuple
    @return: (q, qd, qdd), a tuple of M{m x n} matrices
    @see: L{ctraj}

    """

    if isinstance(tv,(int,int32,float,float64)):
        tscal = float(1)
        t = mat(range(0,tv)).T/(tv-1.) # Normalized time from 0 -> 1
    else:
        tv = arg2array(tv);
        tscal = float(max(tv))
        t = mat(tv).T / tscal
    
    q0 = arg2array(q0)
    q1 = arg2array(q1)
    
    if qd0 == None:
        qd0 = zeros((shape(q0)))
    else:
        qd0 = arg2array(qd0);
    if qd1 == None:
        qd1 = zeros((shape(q1)))
    else:
        qd1 = arg2array(qd1)

    #print qd0
    #print qd1
    
    # compute the polynomial coefficients
    A = 6*(q1 - q0) - 3*(qd1 + qd0)*tscal
    B = -15*(q1 - q0) + (8*qd0 + 7*qd1)*tscal
    C = 10*(q1 - q0) - (6*qd0 + 4*qd1)*tscal
    E = qd0*tscal # as the t vector has been normalized
    F = q0

    tt = concatenate((power(t,5),power(t,4),power(t,3),power(t,2),t,ones(shape(t))),1)
    c = vstack((A, B, C, zeros(shape(A)), E, F))
    qt = tt * c

    # compute velocity
    c = vstack((zeros(shape(A)),5*A,4*B,3*C,zeros(shape(A)),E))
    qdt = tt * c / tscal


    # compute acceleration
    c = vstack((zeros(shape(A)),zeros(shape(A)),20*A,12*B,6*C,zeros(shape(A))))
    qddt = tt * c / (tscal**2)

    return qt,qdt,qddt

## Compute Coordinate Frames 
# written on 07-23-13
def CoordinateFrame(DH, K):
    TR = DH[K*4:K*4+3,0:4]
    #TR = DH[0:3,0:4]        
    #TR = array([[ 1.,  0.,  0.,  1.],[ 0.,  1.,  0.,  2.],[ 0.,  0.,  1.,  3.],[ 0.,  0.,  0.,  1.]])
    length = 0.1
    origin = TR[0:3, 3] # first three elements
    X = origin + length*TR[0:3, 0] # point 'len' units out along x axis
    Y = origin + length*TR[0:3, 1] # point 'len' units out along y axis
    Z = origin + length*TR[0:3, 2] # point 'len' units out along z axis
    #pts = np.vstack([x,x,x]).transpose()
    x_pts = vstack(([origin[0],X[0]],[origin[1],X[1]],[origin[2],X[2]])).T
    y_pts = vstack(([origin[0],Y[0]],[origin[1],Y[1]],[origin[2],Y[2]])).T
    z_pts = vstack(([origin[0],Z[0]],[origin[1],Z[1]],[origin[2],Z[2]])).T
    return x_pts,y_pts,z_pts

## Compute Origin of Coordinate Frames
# written on 07-23-13
def Origin_Frame(DH, K1, K2):
    TR1 = DH[K1*4:K1*4+3,0:4]
    origin1 = TR1[0:3, 3] # first three elements
    TR2 = DH[K2*4:K2*4+3,0:4]
    origin2 = TR2[0:3, 3] # first three elements  
    
    pts = vstack(([origin1[0],origin2[0]],[origin1[1],origin2[1]],[origin1[2],origin2[2]])).T
    return pts

class construct_DRC_Hubo:
    NB = 28
    NJ = NB - 1
    NF = 4
    NLimb = 4
    grav = [0, 0, 9.81]

    # Contact State
    BODY = 0
    LF = 2
    RF = 3
    
    # Joint Index
    J_HPY = 1

    J_LHY = 2
    J_LHR = 3
    J_LHP = 4
    J_LKP = 5
    J_LAP = 6
    J_LAR = 7
    J_LF = J_LAR

    J_RHY = 8
    J_RHR = 9
    J_RHP = 10
    J_RKP = 11
    J_RAP = 12
    J_RAR = 13
    J_RF = J_RAR

    J_LSP = 14
    J_LSR = 15
    J_LSY = 16
    J_LEP = 17
    J_LWY = 18
    J_LWP = 19
    J_LWR = 20
    J_LH = J_LWR

    J_RSP = 21
    J_RSR = 22
    J_RSY = 23
    J_REP = 24
    J_RWY = 25
    J_RWP = 26
    J_RWR = 27
    J_RH = J_RWR

    Frame_LF = 101
    Frame_RF = 102
    Frame_LH = 103
    Frame_RH = 104

    # Frame Index
    F_HPY = 1

    F_LHY = 3
    F_LHR = 4
    F_LHP = 5
    F_LKP = 6
    F_LAP = 7
    F_LAR = 8
    F_LF = F_LAR

    F_RHY = 10
    F_RHR = 11
    F_RHP = 12
    F_RKP = 13
    F_RAP = 14
    F_RAR = 15
    F_RF = F_RAR

    F_LSP = 17
    F_LSR = 18
    F_LSY = 19
    F_LEP = 20
    F_LWY = 21
    F_LWP = 22
    F_LWR = 23
    F_LH = F_LWR

    F_RSP = 25
    F_RSR = 26
    F_RSY = 27
    F_REP = 28
    F_RWY = 29
    F_RWP = 30
    F_RWR = 31
    F_RH = F_RWR

    ## Bodies

    body_com = zeros((NB,3))
    body_mass = zeros((NB,1))
    body_I = zeros((NB,6))
    body_parent = zeros((NB,1))
    body_no_child_frames = zeros((NB,1))
    body_child_frame = -100*ones((NB,3))
    body_name = []

    K = 1 # Body 1
    body_name.append('Body Torso')
    body_com[K-1,] = [0.00186303110078, -0.000700932863028, 0.144864941081]
    body_mass[K-1] = 8.95351612284
    body_I[K-1,] = [0.0175077067002, 0.0101292698627, 0.0117063035943, -8.66965202242e-06, -6.34050266241e-06, -0.00105700122136] #Inertia Ixx Iyy Izz Ixy Iyz Izx
    body_parent[K-1] = -1
    body_no_child_frames[K-1] = 3
    body_child_frame[K-1,] = [J_HPY, J_LSP, J_RSP]

    K = K + 1 # Body 2
    body_name.append('Body Hip')
    body_com[K-1,] = [-0.0160702033923, -0.00217130156789, -0.0329438544914]
    body_mass[K-1] = 3.85811444191
    body_I[K-1,] = [0.0147101365201, 0.0049150553305, 0.0165336604409, -3.18965311766e-07, 6.91609383779e-08, 0.000204985841349] #Inertia Ixx Iyy Izz Ixy Iyz Izx
    body_parent[K-1] = body_name.index('Body Torso')
    body_no_child_frames[K-1] = 2
    body_child_frame[K-1,:2] = [J_LHY, J_RHY]

    K = K + 1 # Body 3
    body_name.append('Body LHY')
    body_com[K-1,] = [0.0378123600719, 2.54097696217e-08, -0.116128242308]
    body_mass[K-1] = 1.02665437744
    body_I[K-1,] = [0.000939663257124, 0.00165966908335, 0.000936466435958, -2.86257426328e-10, -4.52230061997e-09, -0.000211510302662] #Inertia Ixx Iyy Izz Ixy Iyz Izx
    body_parent[K-1] = body_name.index('Body Hip')
    body_no_child_frames[K-1] = 1
    body_child_frame[K-1,:1] = [J_LHR]

    K = K + 1 # Body 4
    body_name.append('Body LHR')
    body_com[K-1,] = [0.00473723996484, -0.00482931760303, -0.032556369283]
    body_mass[K-1] = 1.37473370901
    body_I[K-1,] = [0.00487691965138, 0.00495611046912, 0.00515522620781, -0.000379137788702, 0.000648224499068, 0.000631094588081] #Inertia Ixx Iyy Izz Ixy Iyz Izx
    body_parent[K-1] = body_name.index('Body LHY')
    body_no_child_frames[K-1] = 1
    body_child_frame[K-1,:1] = [J_LHP]

    K = K + 1 # Body 5
    body_name.append('Body LHP')
    body_com[K-1,] = [0.0139820973756, 0.0119128547059, -0.190050892708]
    body_mass[K-1] = 3.39708806381
    body_I[K-1,] = [0.0313848573081, 0.0297969550958, 0.00544518516521, -0.000257888030489, -7.62276306711e-05, 0.000464519122139] #Inertia Ixx Iyy Izz Ixy Iyz Izx
    body_parent[K-1] = body_name.index('Body LHR')
    body_no_child_frames[K-1] = 1
    body_child_frame[K-1,:1] = [J_LKP]

    K = K + 1 # Body 6
    body_name.append('Body LKP')
    body_com[K-1,] = [0.0167128425968, -0.00423174064898, -0.163547981621]
    body_mass[K-1] = 1.25133149222
    body_I[K-1,] = [0.0140834614744, 0.0125968551173, 0.00257590331692, 3.80466382642e-08, -0.000498008689825, -5.82548390175e-05] #Inertia Ixx Iyy Izz Ixy Iyz Izx
    body_parent[K-1] = body_name.index('Body LHP')
    body_no_child_frames[K-1] = 1
    body_child_frame[K-1,:1] = [J_LAP]

    K = K + 1 # Body 7
    body_name.append('Body LAP')
    body_com[K-1,] = [0.0148223972691, 0.0100562738748, 0.00997467775305]
    body_mass[K-1] = 2.00597218063
    body_I[K-1,] = [0.00327242471395, 0.00353130419741, 0.00382005435653, -0.000200723929097, -0.000293646277561, -0.000256255896565] #Inertia Ixx Iyy Izz Ixy Iyz Izx
    body_parent[K-1] = body_name.index('Body LKP')
    body_no_child_frames[K-1] = 1
    body_child_frame[K-1,:1] = [J_LAR]

    K = K + 1 # Body 8
    body_name.append('Body LAR')
    body_com[K-1,] = [0.0165455008794, 0.0021608598221, -0.095291506213]
    body_mass[K-1] = 1.06815376654
    body_I[K-1,] = [0.00267469460888, 0.00595096647226, 0.00656595176302, -8.4864864367e-06, -3.69090743362e-05, 0.000234342536736] #Inertia Ixx Iyy Izz Ixy Iyz Izx
    body_parent[K-1] = body_name.index('Body LAP')
    body_no_child_frames[K-1] = 1
    body_child_frame[K-1,:1] = [Frame_LF]

    K = K + 1 # Body 9
    body_name.append('Body RHY')
    body_com[K-1,] = [0.0378123600719, -2.54097696217e-08, -0.116128242308]
    body_mass[K-1] = 1.02665437744
    body_I[K-1,] = [0.000939663257124, 0.00165966908335, 0.000936466435958, 2.86257426328e-10, 4.52230061997e-09, -0.000211510302662] #Inertia Ixx Iyy Izz Ixy Iyz Izx
    body_parent[K-1] = body_name.index('Body Hip')
    body_no_child_frames[K-1] = 1
    body_child_frame[K-1,:1] = [J_RHR]

    K = K + 1 # Body 10
    body_name.append('Body RHR')
    body_com[K-1,] = [0.00473723996484, 0.00482931760303, -0.032556369283]
    body_mass[K-1] = 1.37473370901
    body_I[K-1,] = [0.00487691965138, 0.00495611046912, 0.00515522620781, 0.000379137788702, -0.000648224499068, 0.000631094588081] #Inertia Ixx Iyy Izz Ixy Iyz Izx
    body_parent[K-1] = body_name.index('Body RHY')
    body_no_child_frames[K-1] = 1
    body_child_frame[K-1,:1] = [J_RHP]

    K = K + 1 # Body 11
    body_name.append('Body RHP')
    body_com[K-1,] = [0.0139820973756, -0.0119128547059, -0.190050892708]
    body_mass[K-1] = 3.39708806381
    body_I[K-1,] = [0.0313848573081, 0.0297969550958, 0.00544518516521, 0.000257888030489, 7.62276306711e-05, 0.000464519122139] #Inertia Ixx Iyy Izz Ixy Iyz Izx
    body_parent[K-1] = body_name.index('Body RHR')
    body_no_child_frames[K-1] = 1
    body_child_frame[K-1,:1] = [J_RKP]

    K = K + 1 # Body 12
    body_name.append('Body RKP')
    body_com[K-1,] = [0.0167128425968, 0.00423174064898, -0.163547981621]
    body_mass[K-1] = 1.25133149222
    body_I[K-1,] = [0.0140834614744, 0.0125968551173, 0.00257590331692, -3.80466382642e-08, 0.000498008689825, -5.82548390175e-05] #Inertia Ixx Iyy Izz Ixy Iyz Izx
    body_parent[K-1] = body_name.index('Body RHP')
    body_no_child_frames[K-1] = 1
    body_child_frame[K-1,:1] = [J_RAP]

    K = K + 1 # Body 13
    body_name.append('Body RAP')
    body_com[K-1,] = [0.0148223972691, -0.0100562738748, 0.00997467775305]
    body_mass[K-1] = 2.00597218063
    body_I[K-1,] = [0.00327242471395, 0.00353130419741, 0.00382005435653, 0.000200723929097, 0.000293646277561, -0.000256255896565] #Inertia Ixx Iyy Izz Ixy Iyz Izx
    body_parent[K-1] = body_name.index('Body RKP')
    body_no_child_frames[K-1] = 1
    body_child_frame[K-1,:1] = [J_RAR]

    K = K + 1 # Body 14
    body_name.append('Body RAR')
    body_com[K-1,] = [0.0165455008794, -0.0021608598221, -0.095291506213]
    body_mass[K-1] = 1.06815376654
    body_I[K-1,] = [0.00267469460888, 0.00595096647226, 0.00656595176302, 8.4864864367e-06, 3.69090743362e-05, 0.000234342536736] #Inertia Ixx Iyy Izz Ixy Iyz Izx
    body_parent[K-1] = body_name.index('Body RAP')
    body_no_child_frames[K-1] = 1
    body_child_frame[K-1,:1] = [Frame_RF]

    K = K + 1 # Body 15
    body_name.append('Body LSP')
    body_com[K-1,] = [0.0125095477278, 0.0682567187642, -0.00010129669296]
    body_mass[K-1] = 0.512814669817
    body_I[K-1,] = [0.00111111449812, 0.00102933875391, 0.00154859148377, 0.00025913285553, -4.71727180543e-06, -1.56634343219e-06] #Inertia Ixx Iyy Izz Ixy Iyz Izx
    body_parent[K-1] = body_name.index('Body Torso')
    body_no_child_frames[K-1] = 1
    body_child_frame[K-1,:1] = [J_LSR]

    K = K + 1 # Body 16
    body_name.append('Body LSR')
    body_com[K-1,] = [-0.0353989625514, -0.000442459951968, -0.0177626332999]
    body_mass[K-1] = 0.478101234588
    body_I[K-1,] = [0.000352000426975, 0.000402774537852, 0.00029532793293, 1.27541844877e-08, -1.05628830152e-06, 4.69390949523e-06] #Inertia Ixx Iyy Izz Ixy Iyz Izx
    body_parent[K-1] = body_name.index('Body LSP')
    body_no_child_frames[K-1] = 1
    body_child_frame[K-1,:1] = [J_LSY]

    K = K + 1 # Body 17
    body_name.append('Body LSY')
    body_com[K-1,] = [0.00596307108671, -0.0022316583599, -0.162009947411]
    body_mass[K-1] = 1.3520653945
    body_I[K-1,] = [0.00482614381092, 0.00476185907752, 0.000629421244474, 9.29900141905e-06, -7.76901327191e-05, -0.000535313792861] #Inertia Ixx Iyy Izz Ixy Iyz Izx
    body_parent[K-1] = body_name.index('Body LSR')
    body_no_child_frames[K-1] = 1
    body_child_frame[K-1,:1] = [J_LEP]

    K = K + 1 # Body 18
    body_name.append('Body LEP')
    body_com[K-1,] = [-0.0257144533591, 0.000806178520289, -0.0417853587165]
    body_mass[K-1] = 0.267758871202
    body_I[K-1,] = [0.000270994112009, 0.000232506396573, 0.000178996997312, 5.10319022379e-0, -6.66148004212e-06, 4.85529326042e-05] #Inertia Ixx Iyy Izz Ixy Iyz Izx
    body_parent[K-1] = body_name.index('Body LSY')
    body_no_child_frames[K-1] = 1
    body_child_frame[K-1,:1] = [J_LWY]

    K = K + 1 # Body 19
    body_name.append('Body LWY')
    body_com[K-1,] = [3.99379011693e-05, 0.0577379856358, -0.151718765023]
    body_mass[K-1] = 1.37072161762
    body_I[K-1,] = [0.00354953016159, 0.00320829277392, 0.000522002143586, -6.67148734149e-07, -0.000697731286306, 2.34544677235e-06] #Inertia Ixx Iyy Izz Ixy Iyz Izx
    body_parent[K-1] = body_name.index('Body LEP')
    body_no_child_frames[K-1] = 1
    body_child_frame[K-1,:1] = [J_LWP]

    K = K + 1 # Body 20
    body_name.append('Body LWP')
    body_com[K-1,] = [-0.000378739071218, 0.0108882434913, 0.0170093706107]
    body_mass[K-1] = 0.294902574333
    body_I[K-1,] = [0.000536599378762, 0.000445108934527, 0.000148767913029, -1.09317989334e-06, -8.97810953897e-05, 6.1983392933e-07] #Inertia Ixx Iyy Izz Ixy Iyz Izx
    body_parent[K-1] = body_name.index('Body LWY')
    body_no_child_frames[K-1] = 1
    body_child_frame[K-1,:1] = [J_LWR]

    K = K + 1 # Body 21
    body_name.append('Body LWR')
    body_com[K-1,] = [0.00526734472243, -0.00165481505248, -0.0476967183847]
    body_mass[K-1] = 0.474006924727
    body_I[K-1,] = [0.000237313165292, 0.000185745961103, 0.000116818847121, 5.43644271427e-07, 5.88365761098e-06, -8.57164775663e-07] #Inertia Ixx Iyy Izz Ixy Iyz Izx
    body_parent[K-1] = body_name.index('Body LWP')
    body_no_child_frames[K-1] = 1
    body_child_frame[K-1,:1] = [Frame_LH]

    K = K + 1 # Body 22
    body_name.append('Body RSP')
    body_com[K-1,] = [0.0125095477278, -0.0682567187642, -0.00010129669296]
    body_mass[K-1] = 0.512814669817
    body_I[K-1,] = [0.00111111449812, 0.00102933875391, 0.00154859148377, -0.00025913285553, 4.71727180543e-06, -1.56634343219e-06] #Inertia Ixx Iyy Izz Ixy Iyz Izx
    body_parent[K-1] = body_name.index('Body Torso')
    body_no_child_frames[K-1] = 1
    body_child_frame[K-1,:1] = [J_RSR]

    K = K + 1 # Body 23
    body_name.append('Body RSR')
    body_com[K-1,] = [-0.0353989625514, 0.000442459951968, -0.0177626332999]
    body_mass[K-1] = 0.478101234588
    body_I[K-1,] = [0.000352000426975, 0.000402774537852, 0.00029532793293, -1.27541844877e-08, 1.05628830152e-06, 4.69390949523e-06] #Inertia Ixx Iyy Izz Ixy Iyz Izx
    body_parent[K-1] = body_name.index('Body RSP')
    body_no_child_frames[K-1] = 1
    body_child_frame[K-1,:1] = [J_RSY]

    K = K + 1 # Body 24
    body_name.append('Body RSY')
    body_com[K-1,] = [0.00596307108671, 0.0022316583599, -0.162009947411]
    body_mass[K-1] = 1.3520653945
    body_I[K-1,] = [0.00482614381092, 0.00476185907752, 0.000629421244474, -9.29900141905e-06, 7.76901327191e-05, -0.000535313792861] #Inertia Ixx Iyy Izz Ixy Iyz Izx
    body_parent[K-1] = body_name.index('Body RSR')
    body_no_child_frames[K-1] = 1
    body_child_frame[K-1,:1] = [J_REP]

    K = K + 1 # Body 25
    body_name.append('Body REP')
    body_com[K-1,] = [-0.0257144533591, -0.000806178520289, -0.0417853587165]
    body_mass[K-1] = 0.267758871202
    body_I[K-1,] = [0.000270994112009, 0.000232506396573, 0.000178996997312, -5.10319022379e-06, 6.66148004212e-06, 4.85529326042e-05] #Inertia Ixx Iyy Izz Ixy Iyz Izx
    body_parent[K-1] = body_name.index('Body RSY')
    body_no_child_frames[K-1] = 1
    body_child_frame[K-1,:1] = [J_RWY]

    K = K + 1 # Body 26
    body_name.append('Body RWY')
    body_com[K-1,] = [3.99379011693e-05, -0.0577379856358, -0.151718765023]
    body_mass[K-1] = 1.37072161762
    body_I[K-1,] = [0.00354953016159, 0.00320829277392, 0.000522002143586, 6.67148734149e-07, 0.000697731286306, 2.34544677235e-06] #Inertia Ixx Iyy Izz Ixy Iyz Izx
    body_parent[K-1] = body_name.index('Body REP')
    body_no_child_frames[K-1] = 1
    body_child_frame[K-1,:1] = [J_RWP]

    K = K + 1 # Body 27
    body_name.append('Body RWP')
    body_com[K-1,] = [-0.000378739071218, -0.0108882434913, 0.0170093706107]
    body_mass[K-1] = 0.294902574333
    body_I[K-1,] = [0.000536599378762, 0.000445108934527, 0.000148767913029, 1.09317989334e-06, 8.97810953897e-05, 6.1983392933e-07] #Inertia Ixx Iyy Izz Ixy Iyz Izx
    body_parent[K-1] = body_name.index('Body RWY')
    body_no_child_frames[K-1] = 1
    body_child_frame[K-1,:1] = [J_RWR]

    K = K + 1 # Body 28
    body_name.append('Body RWR')
    body_com[K-1,] = [0.00526734472243, 0.00165481505248, -0.0476967183847]
    body_mass[K-1] = 0.474006924727
    body_I[K-1,] = [0.000237313165292, 0.000185745961103, 0.000116818847121, -5.43644271427e-07, -5.88365761098e-06, -8.57164775663e-07] #Inertia Ixx Iyy Izz Ixy Iyz Izx
    body_parent[K-1] = body_name.index('Body RWP')
    body_no_child_frames[K-1] = 1
    body_child_frame[K-1,:1] = [Frame_RH]

    ### Joints
    joint_name = []
    joint_axis = zeros((NJ,3))
    joint_translation = zeros((NJ,3))
    joint_rotation = zeros((NJ,3))
    joint_q_min = zeros((NJ,1))
    joint_q_max = zeros((NJ,1))

    #Joint 1
    K = 1
    joint_name.append('Joint HPY')
    joint_axis[K-1,] = [0, 0, 1]
    joint_translation[K-1,] = [0, 0, 0]
    joint_rotation[K-1,] = [0, 0, 0]
    joint_q_min[K-1] = -3.14158
    joint_q_max[K-1] = 3.14158

    #Joint 2
    K = K + 1
    joint_name.append('Joint LHY')
    joint_axis[K-1,] = [0, 0, 1]
    joint_translation[K-1,] = [0.0, 0.0884999999999, -0.0230012618128]
    joint_rotation[K-1,] = [0, 0, 0]
    joint_q_min[K-1] = -1.7453
    joint_q_max[K-1] = 1.7453

    #Joint 3
    K = K + 1
    joint_name.append('Joint LHR')
    joint_axis[K-1,] = [1, 0, 0]
    joint_translation[K-1,] = [0.0, 0.0, -0.14098410676]
    joint_rotation[K-1,] = [0, 0, 0]
    joint_q_min[K-1] = -0.174
    joint_q_max[K-1] = 0.785

    #Joint 4
    K = K + 1
    joint_name.append('Joint LHP')
    joint_axis[K-1,] = [0, 1, 0]
    joint_translation[K-1,] = [0.0, 0.0, 0.0]
    joint_rotation[K-1,] = [0, 0, 0]
    joint_q_min[K-1] = -1.5708
    joint_q_max[K-1] = 1.5708

    #Joint 5
    K = K + 1
    joint_name.append('Joint LKP')
    joint_axis[K-1,] = [0, 1, 0]
    joint_translation[K-1,] = [0.0, 0.0, -0.329885]
    joint_rotation[K-1,] = [0, 0, 0]
    joint_q_min[K-1] = -0.174
    joint_q_max[K-1] = 2.61

    #Joint 6
    K = K + 1
    joint_name.append('Joint LAP')
    joint_axis[K-1,] = [0, 1, 0]
    joint_translation[K-1,] = [0.0, 0.0, -0.33001444503]
    joint_rotation[K-1,] = [0, 0, 0]
    joint_q_min[K-1] = -1.5708
    joint_q_max[K-1] = 1.5708

    #Joint 7
    K = K + 1
    joint_name.append('Joint LAR')
    joint_axis[K-1,] = [1, 0, 0]
    joint_translation[K-1,] = [0.0, 0.0, 0.0]
    joint_rotation[K-1,] = [0, 0, 0]
    joint_q_min[K-1] = -1.5708
    joint_q_max[K-1] = 1.5708

    #Joint 8
    K = K + 1
    joint_name.append('Joint RHY')
    joint_axis[K-1,] = [0, 0, 1]
    joint_translation[K-1,] = [0.0, -0.0884999999999, -0.0230012618128]
    joint_rotation[K-1,] = [0, 0, 0]
    joint_q_min[K-1] = -1.7453
    joint_q_max[K-1] = 1.7453

    #Joint 9
    K = K + 1
    joint_name.append('Joint RHR')
    joint_axis[K-1,] = [1, 0, 0]
    joint_translation[K-1,] = [0.0, -0.0, -0.14098410676]
    joint_rotation[K-1,] = [0, 0, 0]
    joint_q_min[K-1] = -0.785
    joint_q_max[K-1] = 0.174

    #Joint 10
    K = K + 1
    joint_name.append('Joint RHP')
    joint_axis[K-1,] = [0, 1, 0]
    joint_translation[K-1,] = [0.0, -0.0, 0.0]
    joint_rotation[K-1,] = [0, 0, 0]
    joint_q_min[K-1] = -1.5708
    joint_q_max[K-1] = 1.5708

    #Joint 11
    K = K + 1
    joint_name.append('Joint RKP')
    joint_axis[K-1,] = [0, 1, 0]
    joint_translation[K-1,] = [0.0, -0.0, -0.329885]
    joint_rotation[K-1,] = [0, 0, 0]
    joint_q_min[K-1] = -0.174
    joint_q_max[K-1] = 2.61

    #Joint 12
    K = K + 1
    joint_name.append('Joint RAP')
    joint_axis[K-1,] = [0, 1, 0]
    joint_translation[K-1,] = [0.0, -0.0, -0.33001444503]
    joint_rotation[K-1,] = [0, 0, 0]
    joint_q_min[K-1] = -1.5708
    joint_q_max[K-1] = 1.5708

    #Joint 13
    K = K + 1
    joint_name.append('Joint RAR')
    joint_axis[K-1,] = [1, 0, 0]
    joint_translation[K-1,] = [0.0, -0.0, 0.0]
    joint_rotation[K-1,] = [0, 0, 0]
    joint_q_min[K-1] = -1.5708
    joint_q_max[K-1] = 1.5708

    #Joint 14
    K = K + 1
    joint_name.append('Joint LSP')
    joint_axis[K-1,] = [0, 1, 0]
    joint_translation[K-1,] = [0.000999912451921, 0.130499970438, 0.20609999003]
    joint_rotation[K-1,] = [0, 0, 0]
    joint_q_min[K-1] = -3.1415
    joint_q_max[K-1] = 3.1415

    #Joint 15
    K = K + 1
    joint_name.append('Joint LSR')
    joint_axis[K-1,] = [1, 0, 0]
    joint_translation[K-1,] = [0.0330000045621, 0.099000025, 0.0]
    joint_rotation[K-1,] = [0, 0, 0]
    joint_q_min[K-1] = -0.26
    joint_q_max[K-1] = 3.1415

    #Joint 16
    K = K + 1
    joint_name.append('Joint LSY')
    joint_axis[K-1,] = [0, 0, 1]
    joint_translation[K-1,] = [-0.0330051372049, 0.0, -0.027]
    joint_rotation[K-1,] = [0, 0, 0]
    joint_q_min[K-1] = -3.1415
    joint_q_max[K-1] = 3.1415

    #Joint 17
    K = K + 1
    joint_name.append('Joint LEP')
    joint_axis[K-1,] = [0, 1, 0]
    joint_translation[K-1,] = [0.0299938548311, 0.0, -0.273]
    joint_rotation[K-1,] = [0, 0, 0]
    joint_q_min[K-1] = -2.96
    joint_q_max[K-1] = 0

    #Joint 18
    K = K + 1
    joint_name.append('Joint LWY')
    joint_axis[K-1,] = [0, 0, 1]
    joint_translation[K-1,] = [-0.03, 0.0, -0.052]
    joint_rotation[K-1,] = [0, 0, 0]
    joint_q_min[K-1] = -3.1415
    joint_q_max[K-1] = 3.1415

    #Joint 19
    K = K + 1
    joint_name.append('Joint LWP')
    joint_axis[K-1,] = [0, 1, 0]
    joint_translation[K-1,] = [0.0, 0.0, -0.261789998979]
    joint_rotation[K-1,] = [0, 0, 0]
    joint_q_min[K-1] = -3.1415
    joint_q_max[K-1] = 3.1415

    #Joint 20
    K = K + 1
    joint_name.append('Joint LWR')
    joint_axis[K-1,] = [0, 0, 1]
    joint_translation[K-1,] = [0.0, 0.0, -0.0275]
    joint_rotation[K-1,] = [0, 0, 0]
    joint_q_min[K-1] = -3.1415
    joint_q_max[K-1] = 3.1415

    #Joint 21
    K = K + 1
    joint_name.append('Joint RSP')
    joint_axis[K-1,] = [0, 1, 0]
    joint_translation[K-1,] = [0.000999912451921, -0.130499970438, 0.20609999003]
    joint_rotation[K-1,] = [0, 0, 0]
    joint_q_min[K-1] = -3.1415
    joint_q_max[K-1] = 3.1415

    #Joint 22
    K = K + 1
    joint_name.append('Joint RSR')
    joint_axis[K-1,] = [1, 0, 0]
    joint_translation[K-1,] = [0.0330000045621, -0.099000025, 0.0]
    joint_rotation[K-1,] = [0, 0, 0]
    joint_q_min[K-1] = -3.1415
    joint_q_max[K-1] = 0.26

    #Joint 23
    K = K + 1
    joint_name.append('Joint RSY')
    joint_axis[K-1,] = [0, 0, 1]
    joint_translation[K-1,] = [-0.0330051372049, -0.0, -0.027]
    joint_rotation[K-1,] = [0, 0, 0]
    joint_q_min[K-1] = -3.1415
    joint_q_max[K-1] = 3.1415

    #Joint 24
    K = K + 1
    joint_name.append('Joint REP')
    joint_axis[K-1,] = [0, 1, 0]
    joint_translation[K-1,] = [0.0299938548311, -0.0, -0.273]
    joint_rotation[K-1,] = [0, 0, 0]
    joint_q_min[K-1] = -2.96
    joint_q_max[K-1] = 0

    #Joint 25
    K = K + 1
    joint_name.append('Joint RWY')
    joint_axis[K-1,] = [0, 0, 1]
    joint_translation[K-1,] = [-0.03, -0.0, -0.052]
    joint_rotation[K-1,] = [0, 0, 0]
    joint_q_min[K-1] = -3.1415
    joint_q_max[K-1] = 3.1415

    #Joint 26
    K = K + 1
    joint_name.append('Joint RWP')
    joint_axis[K-1,] = [0, 1, 0]
    joint_translation[K-1,] = [0.0, -0.0, -0.261789998979]
    joint_rotation[K-1,] = [0, 0, 0]
    joint_q_min[K-1] = -3.1415
    joint_q_max[K-1] = 3.1415

    #Joint 27
    K = K + 1
    joint_name.append('Joint RWR')
    joint_axis[K-1,] = [0, 0, 1]
    joint_translation[K-1,] = [0.0, -0.0, -0.0275]
    joint_rotation[K-1,] = [0, 0, 0]
    joint_q_min[K-1] = -3.1415
    joint_q_max[K-1] = 3.1415

    ### Frames
    frame_name = []
    frame_axis = zeros((NF + 100,1))
    frame_translation = zeros((NF + 100,3))
    frame_rotation = zeros((NF + 100,3))

    #Frame 1
    K = 101
    frame_name.append('Left Foot')
    frame_axis[K-1] = -1
    frame_translation[K-1,] = [0.0, 0.0, -0.140]
    frame_rotation[K-1,] = [0, 0, 0]

    #Frame 2
    K = K + 1
    frame_name.append('Right Foot')
    frame_axis[K-1] = -1
    frame_translation[K-1,] = [0.0, 0.0, -0.140]
    frame_rotation[K-1,] = [0, 0, 0]

    #Frame 3
    K = K + 1
    frame_name.append('Left Hand')
    frame_axis[K-1] = -1
    frame_translation[K-1,] = [0, 0, -0.001*119.52]
    frame_rotation[K-1,] = [0, 0, 0]

    #Frame 4
    K = K + 1
    frame_name.append('Right Hand')
    frame_axis[K-1] = -1
    frame_translation[K-1,] = [0, 0, -0.001*119.52]
    frame_rotation[K-1,] = [0, 0, 0]

    # for index matching (Joint I -> Joint I-1)
    body_child_frame = body_child_frame-1

    ## get frame list
    # each frame - corresponding joint and frame
    frame_list = []
    def get_frame_list(self):
        for i in range(self.NB):
            self.frame_list = concatenate((self.frame_list,[i-1]))
            for j in range(self.body_no_child_frames[i]):
                index_child_frame = self.body_child_frame[i,j]
                if index_child_frame >= 100:
                    self.frame_list = concatenate((self.frame_list,[index_child_frame]))
        return self.frame_list

    ## get bf and cf
    # bf - body reference frame index
    # cf - child frame index
    body_bf = zeros((NB,1))
    body_cf = zeros((NB,3))

    def get_frame_list_bf_cf(self):
        for i in range(self.NB):
            self.body_bf[i] = argwhere(self.frame_list == i-1)
            for j in range(self.body_no_child_frames[i]):
                index_child_frame = argwhere(self.frame_list == self.body_child_frame[i,j])
                self.body_cf[i,j] = index_child_frame

    ## get body list
    # the body to which each frame belongs
    body_list = []

    def get_body_list(self):
        for k in range(self.frame_list.shape[0]):
            index_body = 0
            for i in range(self.NB):
                for j in range(self.body_no_child_frames[i]):
                    if self.body_cf[i,j] == k:
                        index_body = i
            self.body_list = concatenate((self.body_list,[index_body]))

    ## generate body path w.r.t base
    body_path = -1*ones((NB,8))

    def generate_body_path(self):
        for I in range(self.NB):
            i = 0
            self.body_path[I,i] = I
            i = 1
            K = I
            while K > -1:
                if self.body_parent[K] > -1:
                    self.body_path[I,i] = self.body_parent[K]
                K = int(self.body_parent[K])
                i = i + 1

    ## generate body path w.r.t LF
    body_path_LF = -1*ones((NB,16))

    def generate_body_path_LF(self):
        body_index_LF = self.body_name.index('Body LAR')
        base_body = body_index_LF

        for I in range(self.NB):
            # find the index of the current body in the list for base body
            index_tmp1 = argwhere(self.body_path[base_body,] == I)
            if index_tmp1 > -1: # if non-empty
                # reverse the path from the base to the Ith body
                tmp = self.body_path[base_body,index_tmp1::-1]
                self.body_path_LF[I,:tmp.size] = tmp
            else:
                # if not, combine both paths
                # index of body 0 in the parent body list of body I
                index_tmp2 = argwhere(self.body_path[I,] == 0)
                # index of the common body if found earlier than the last body
                value_tmp = self.body_path[I,index_tmp2-1]
                index_tmp3 = argwhere(self.body_path[base_body,] == value_tmp[0])

                if not index_tmp3: # if it wasn't found
                    # parent body path of body I without body 0
                    body_path_tmp1 = self.body_path[I,:index_tmp2]

                    # append the body path of base body until the common body
                    tmp = hstack((body_path_tmp1,self.body_path[base_body,::-1]))
                else: # if found
                    # parent body path of body I without body 0
                    body_path_tmp1 = self.body_path[I,:index_tmp2-1]

                    # append the body path of base body until the common body
                    tmp = hstack((body_path_tmp1, self.body_path[base_body, index_tmp3::-1]))

                self.body_path_LF[I,:tmp.size] = tmp

    ## generate body path w.r.t RF
    body_path_RF = -1*ones((NB,16))

    def generate_body_path_RF(self):
        body_index_RF = self.body_name.index('Body RAR')
        base_body = body_index_RF

        for I in range(self.NB):
            # find the index of the current body in the list for base body
            index_tmp1 = argwhere(self.body_path[base_body,] == I)
            if index_tmp1 > -1: # if non-empty
                # reverse the path from the base to the Ith body
                tmp = self.body_path[base_body,index_tmp1::-1]
                self.body_path_RF[I,:tmp.size] = tmp
            else:
                # if not, combine both paths
                # index of body 0 in the parent body list of body I
                index_tmp2 = argwhere(self.body_path[I,] == 0)
                # index of the common body if found earlier than the last body
                value_tmp = self.body_path[I,index_tmp2-1]
                index_tmp3 = argwhere(self.body_path[base_body,] == value_tmp[0])

                if not index_tmp3: # if it wasn't found
                    # parent body path of body I without body 0
                    body_path_tmp1 = self.body_path[I,:index_tmp2]

                    # append the body path of base body until the common body
                    tmp = hstack((body_path_tmp1,self.body_path[base_body,::-1]))
                else: # if found
                    # parent body path of body I without body 0
                    body_path_tmp1 = self.body_path[I,:index_tmp2-1]

                    # append the body path of base body until the common body
                    tmp = hstack((body_path_tmp1, self.body_path[base_body, index_tmp3::-1]))

                self.body_path_RF[I,:tmp.size] = tmp

    ### Remove the body 0 and body 1 intelligently
    """ we only remove the body from the list which is parent for both next
    body in the list and the previous in the list meaning that both bodies
    are connected through child frames that belong to the corresponding
    body so that the joint frame of that body doesn't need to be considered
    in describing the motion constituted by the joint rotations """

    ### filter for body_path_LF;

    def filter_body_path_LF(self):
        body_path_LF_tmp = -1*ones((self.NB,16))
        for I in range(self.NB):
            body_path_LF_I = self.body_path_LF[I,]
            n_bodies = argwhere(body_path_LF_I == -1)
            body_path_tmp = array([])
            for k in range(n_bodies[0]):
                index_body = body_path_LF_I[k]
                if k > 0 and k < n_bodies[0]:
                # if body I is parent body to the previous body and the next body in the path
                    if index_body == self.body_parent[body_path_LF_I[k-1]] and index_body == self.body_parent[body_path_LF_I[k+1]]:
                        a = 1
                    else:
                        body_path_tmp = hstack((body_path_tmp, body_path_LF_I[k]))
                else:
                    body_path_tmp = hstack((body_path_tmp, body_path_LF_I[k]))

            body_path_LF_tmp[I,:body_path_tmp.size] = body_path_tmp
        return body_path_LF_tmp

    ### filter for body_path_RF;

    def filter_body_path_RF(self):
        body_path_RF_tmp = -1*ones((self.NB,16))
        for I in range(self.NB):
            body_path_RF_I = self.body_path_RF[I,]
            n_bodies = argwhere(body_path_RF_I == -1)
            body_path_tmp = array([])
            for k in range(n_bodies[0]):
                index_body = body_path_RF_I[k]
                if k > 0 and k < n_bodies[0]:
                # if body I is parent body to the previous body and the next body in the path
                    if index_body == self.body_parent[body_path_RF_I[k-1]] and index_body == self.body_parent[body_path_RF_I[k+1]]:
                        a = 1
                    else:
                        body_path_tmp = hstack((body_path_tmp, body_path_RF_I[k]))
                else:
                    body_path_tmp = hstack((body_path_tmp, body_path_RF_I[k]))

            body_path_RF_tmp[I,:body_path_tmp.size] = body_path_tmp
        return body_path_RF_tmp

    ## Homogeneous Transformation Matrix
    # written on 07-21-13
    def HT_tree(self, q_data, index_frame):
        index = self.frame_list[index_frame]
        #q_data = array(q_data)

        if index >= 100: # a fixed frames
            i_joint = self.body_list[index_frame] - 1
            q = q_data[i_joint]
            R_axis = rotvec2tr(q[0,0], self.joint_axis[i_joint,])
            #R_r = trotx(self.frame_rotation[index,0])
            #R_p = troty(self.frame_rotation[index,1])
            #R_y = trotz(self.frame_rotation[index,2])

            x = self.frame_translation[index,0]
            y = self.frame_translation[index,1]
            z = self.frame_translation[index,2]
            T = transl(x,y,z)

            T_i = R_axis*T#*R_r*R_p*R_y
        else: # joints
            i_joint = self.body_list[index_frame] - 1

            # if first joint in a branch
            if i_joint < 0:
                R_axis = trotz(0)

            else: # not the first joint in a branch
                q = q_data[i_joint]
                R_axis = rotvec2tr(q[0,0], self.joint_axis[i_joint,])

            #R_r = trotx(self.joint_rotation[index,0])
            #R_p = troty(self.joint_rotation[index,1])
            #R_y = trotz(self.joint_rotation[index,2])

            x = self.joint_translation[index,0]
            y = self.joint_translation[index,1]
            z = self.joint_translation[index,2]
            T = transl(x,y,z)

            T_i = R_axis*T#*R_r*R_p*R_y

        return T_i

    ## Forward Kinematics
    # written on 07-22-13
    def FK_tree(self, q_data, index_frame, reference = 0):
        T_FK = trotz(0)

        # backward recursion
        # until it reaches the first frame in the branch
        while index_frame > -1:
            T_FK = self.HT_tree(q_data, index_frame)*T_FK
            # find the joint index that moves the body which the frame belongs to
            index_joint = self.body_list[index_frame] - 1
            # find the index of the frame for the joint
            if(index_joint >= 0):
                index_frame = argwhere(self.frame_list == index_joint)
                index_frame = index_frame[0]
                index_frame = index_frame[0]
            else: # stop at the base
                index_frame = -1;

        # reference 0: body, 2: LF, 3: RF
        if reference >= 2:
            if reference == 2:
                F_REF = self.F_LF
            elif reference == 3:
                F_REF = self.F_RF

            T_0_REF = self.FK_tree(q_data, F_REF)
            T_FK = inv(T_0_REF)*T_FK

        return T_FK

    ## Jacobian Computation
    # written on 07-22-13
    def J_tree(self, q_data, index_frame, reference = 0):
        Jacob = mat(zeros((6,self.NJ)))
        I = self.body_list[index_frame]
        if reference < 2: # Body
            T_0_E = self.FK_tree(q_data, index_frame)
            R_0_E = t2r(T_0_E)
            R_E_0 = R_0_E.transpose()
            p_k = transl(T_0_E)

            while I > 0 :
                index_bi = self.body_bf[I]
                T_bi = self.FK_tree(q_data, index_bi[0])
                p_bi = transl(T_bi)
                u_bi_i_1 = mat(self.joint_axis[I-1,]).T
                R_0_bi = t2r(T_bi)
                # u_i_1 is w.r.t b(i)
                u_i_1 = R_0_bi*u_bi_i_1
                w_k = u_i_1
                v_k = crossp(u_i_1,(p_k-p_bi))
                Jacob[:,I-1] = vstack((v_k, w_k))               
                I = self.body_parent[I][0]

            # if the reference is the end-effector    
            if reference == 1:
                Jacob = vstack((hstack((R_E_0, zeros((3,3)))), hstack((zeros((3,3)),R_E_0))))*Jacob
        elif reference >= 2: # reference is one of the feet
            ###      use of mask:
            #    the joint rotations in the support leg chain (from support foot to
            #    the waist) causes the movement in the negative direction w.r.t the
            #    support foot frame, so we need to incorporate the signs in the computation.
            mask_tmp = ones(( self.NJ))
            mask_tmp[self.J_HPY-1] = -mask_tmp[self.J_HPY-1]
            
            if reference == 2: # reference is left foot
                F_REF = self.F_LF
                body_path_B = self.body_path_LF
                mask_tmp[self.J_LHY-1:self.J_LF] = -mask_tmp[self.J_LHY-1:self.J_LF]

            elif reference == 3: # refrence is right foot
                F_REF = self.F_RF
                body_path_B = self.body_path_RF
                mask_tmp[self.J_RHY-1:self.J_RF] = -mask_tmp[self.J_RHY-1:self.J_RF]
                
            mask = diag(mask_tmp)
            
            # transformation from base to the support foot frame
            T_0_B = self.FK_tree(q_data,F_REF)
            R_0_B = t2r(T_0_B)
            T_0_k = self.FK_tree(q_data,index_frame)
            T_B_k = inv(T_0_B)*T_0_k
            p_B_k = transl(T_B_k)
            
            ## compute the jacobian
            cnt = 0
            body_path = body_path_B[I,]
            I = body_path[cnt]
            while I >= 0:
                if I > 0:
                    index_bi = self.body_bf[I]
                    T_0_bi = self.FK_tree(q_data,index_bi[0])
                    R_0_bi = t2r(T_0_bi)
                    T_B_bi = inv(T_0_B)*T_0_bi
                    p_B_bi = transl(T_B_bi)
                    
                    u_bi_i_1 = mat(self.joint_axis[I-1,]).T
                    u_0_i_1 = R_0_bi*u_bi_i_1
                    u_B_i_1 = R_0_B.T*u_0_i_1
                    
                    w_k = u_B_i_1
                    v_k = crossp(u_B_i_1,(p_B_k-p_B_bi))
                    Jacob[:,I-1] = vstack((v_k, w_k))      
                cnt += 1
                I = body_path[cnt]
            
            #apply the mask
            Jacob = Jacob*mask

        return Jacob
    
    ## CoM computation
    # written on 07-21-13
    def CoM(self, q_data, reference = 0):
        M_robot = 0
        CoM_robot = zeros((1,3))
        for i in range(1, self.NB):
            M_robot = M_robot + self.body_mass[i]
            bi = self.body_bf[i]

            # location of body reference frame of body i
            T_bi = self.FK_tree(q_data, bi[0])
            
            # rotation by joint i-1
            R_q_i_1 = rotvec2r(q_data[i-1,0], self.joint_axis[i-1,])
            
            # position of center of mass of body i rotated by joint i-1 
            # w.r.t bi frame
            s_i_star = R_q_i_1*mat(self.body_com[i,]).T
            
            p_CoM_i = T_bi*concatenate((s_i_star, mat([1])))
            CoM_robot = CoM_robot + self.body_mass[i]*p_CoM_i[:3,0].T
            
        # body 1
        K = 0
        M_robot = M_robot + self.body_mass[K]
        p_CoM_i = self.body_com[K,]
        CoM_robot = CoM_robot + self.body_mass[K]*p_CoM_i
        
        # compute CoM 
        CoM_robot = CoM_robot/M_robot
        
        if reference > 1:
            if reference == 2:
                F_REF = self.F_LF
            elif reference == 3:
                F_REF = self.F_RF
            
            T_0_REF = self.FK_tree(q_data, F_REF)    
            CoM_robot = inv(T_0_REF)*concatenate((CoM_robot.T, mat([1])))
            CoM_robot = CoM_robot[:3].T
        
        return CoM_robot
        
    ## CoM Jacobian Computation
    # written on 07-21-13
    def J_com_tree(self, q_data, reference = 0):
        # initialize J_mi
        Jacob_mi_L = zeros((3,self.NJ,self.NB))
        Jacob_com_L = mat(zeros((3,self.NJ)))
         
        if reference < 1:
            for i in range(1, self.NB):
                Jacob = mat(zeros((6,self.NJ)))
                Jacob_L = mat(zeros((3,self.NJ)))
                
                # reference frame of body i
                bi = self.body_bf[i][0]
                
                # location of body reference frame of body i
                T_bi = self.FK_tree(q_data, bi)
                
                # rotation by joint i-1
                R_q_i_1 = rotvec2r(q_data[i-1,0], self.joint_axis[i-1,])
                
                # position of center of mass of body i rotated by joint i-1
                # w.r.t bi frame
                s_i_star = R_q_i_1*mat(self.body_com[i,]).T
                
                # p_com_i = position of com of body i w.r.t base frame
                p_com_i = T_bi*concatenate((s_i_star, mat([1])))
                p_com_i = p_com_i[:3]
                
                # rotation matrix of b(i) w.r.t base frame
                R_bi = t2r(T_bi)
                R_bi_base = R_bi.T # rotation matrix of base frame w.r.t b(i) frame
                
                # computation of J_mi
                I = i
                while I > 0:
                    index_bi = self.body_bf[I][0]
                    T_0_bi = self.FK_tree(q_data, index_bi)
                    p_bi = transl(T_0_bi)
                    u_bi_i_1 = mat(self.joint_axis[I-1,]).T
                    R_0_bi = t2r(T_0_bi)
                    u_i_1 = R_0_bi*u_bi_i_1
            
                    w_mi = u_i_1
                    v_mi = crossp(u_i_1,(p_com_i-p_bi))
                    Jacob[:,I-1] = vstack((v_mi, w_mi))      
            
                    # the parent body becomes next body associated with kth frame
                    I = self.body_parent[I][0]
        
                # J_i
                Jacob_L = Jacob[:3,]
                Jacob_mi_L[ :, :, i] = Jacob_L
        
        # if the reference is feet
        elif reference > 1:
            mask_tmp = ones((self.NJ))
            mask_tmp[self.J_HPY-1] = -mask_tmp[self.J_HPY-1]
            
            if reference == 2: # reference is left foot
                F_REF = self.F_LF
                body_path_B = self.body_path_LF
                mask_tmp[self.J_LHY-1:self.J_LF] = -mask_tmp[self.J_LHY-1:self.J_LF]

            elif reference == 3: # refrence is right foot
                F_REF = self.F_RF
                body_path_B = self.body_path_RF
                mask_tmp[self.J_RHY-1:self.J_RF] = -mask_tmp[self.J_RHY-1:self.J_RF]
                
            mask = diag(mask_tmp)
            
            # transformation from base to the support foot frame
            T_0_B = self.FK_tree(q_data,F_REF)
            R_0_B = t2r(T_0_B)
            
            for i in range(self.NB):
                Jacob = mat(zeros((6,self.NJ)))
                Jacob_L = mat(zeros((3,self.NJ)))
                
                if i == 0: # for body 1
                    
                    # location of body reference frame of body 1 w.r.t base
                    T_B_0 = inv(T_0_B)
                    
                    # position of center of mass of body i rotated by joint i-1
                    # w.r.t bi frame
                    s_i_star = mat(self.body_com[i,]).T
                    
                    # p_com_i = position of com of body i w.r.t base frame
                    p_B_com_i = T_B_0*concatenate((s_i_star, mat([1])))
                    p_B_com_i = p_B_com_i[:3]
                    
                    # rotation matrix of base w.r.t support foot frame
                    R_B_0 = t2r(T_B_0)
                    R_0_B = R_B_0.T # rotation matrix of base frame w.r.t b(i) frame

                else:
                    # reference frame of body i
                    bi = self.body_bf[i][0]
                    
                    # location of body reference frame of body i
                    T_B_bi = inv(T_0_B)*self.FK_tree(q_data, bi)
                    
                    # rotation by joint i-1
                    R_q_i_1 = rotvec2r(q_data[i-1,0], self.joint_axis[i-1,])
                    
                    # position of center of mass of body i rotated by joint i-1
                    # w.r.t bi frame
                    s_i_star = R_q_i_1*mat(self.body_com[i,]).T
                    
                    # p_com_i = position of com of body i  w.r.t support foot frame
                    p_B_com_i = T_B_bi*concatenate((s_i_star, mat([1])))
                    p_B_com_i = p_B_com_i[:3]
                    
                    # rotation matrix of b(i)  w.r.t support foot frame
                    R_B_bi = t2r(T_B_bi)
                    R_bi_B = R_B_bi.T # rotation matrix of base frame w.r.t b(i) frame
                
                ## computation of J_mi w.r.t support foot frame     
                # get the body path from body i to the body of support foot
                body_path = body_path_B[i,]
                
                """ check if body path to the body i is a subset of body path of
                body 0. If yes, the index starts from the second one in the
                path since the body movement is expressed w.r.t its child
                joint frame instead of the body reference frame."""
                
                # check if the body path is a subset of body path of body 1
                x = Set(body_path_B[0,:])
                y = Set(body_path)
                if y.issubset(x):
                    cnt = 1
                else:
                    cnt = 0
                
                I = body_path[cnt]
                while I >= 0:
                    if I > 0:
                        index_bi = self.body_bf[I]
                        T_0_bi = self.FK_tree(q_data,index_bi[0])
                        R_0_bi = t2r(T_0_bi)
                        T_B_bi = inv(T_0_B)*T_0_bi
                        p_B_bi = transl(T_B_bi)
                        
                        u_bi_i_1 = mat(self.joint_axis[I-1,]).T
                        u_0_i_1 = R_0_bi*u_bi_i_1
                        u_B_i_1 = R_0_B.T*u_0_i_1
                        
                        w_k = u_B_i_1
                        v_k = crossp(u_B_i_1,(p_B_com_i-p_B_bi))
                        Jacob[:,I-1] = vstack((v_k, w_k))      
                    cnt += 1
                    I = body_path[cnt]
            
                #apply the mask
                Jacob = Jacob*mask
                Jacob_L = Jacob[:3,]   
                Jacob_mi_L[ :, :, i] = Jacob_L
        
        # compute Jcom_L
        M_robot = 0
        for i in range(self.NB):
            M_robot = M_robot + self.body_mass[i]
            # accumulate mi*Jmi_L
            Jacob_com_L = Jacob_com_L + self.body_mass[i]*Jacob_mi_L[:,:,i]

            #Jcom_L = sum_i(mi*Jmi_L)/M_robot
        Jacob_com_L = Jacob_com_L/M_robot
            
        return Jacob_com_L
    
    ## joint limit check (clamping)
    # written on 07-22-13
    def joint_limit_check(self,q_data):
        flag = 0
        for I in range(self.NJ):
            q_min = self.joint_q_min[I]
            q_max = self.joint_q_max[I]
            
            if q_data[I] > q_max:
                q_data[I] = q_max
                flag = 1
            elif q_data[I] < q_min:
                q_data[I] = q_min
                flag = 1
                
            if flag == 1:
                 
                print 'joint ' +str(I+1)+ ' limit has reached'
                
            flag = 0
        return q_data
            
    
    def __init__(self):
        # get frame list
        self.get_frame_list()
        # get bf and cf
        self.get_frame_list_bf_cf()
        # get body list
        self.get_body_list()
        # generate body path
        self.generate_body_path()
        # generate body path w.r.t LF
        self.generate_body_path_LF()
        # generate body path w.r.t RF
        self.generate_body_path_RF()
        # filter body path LF
        self.body_path_LF = self.filter_body_path_LF()
        # filter body path RF
        self.body_path_RF = self.filter_body_path_RF()


if __name__ == "__main__":

    robot = construct_DRC_Hubo()
    q_zero = mat(zeros((robot.NJ,1)))
    print robot.HT_tree(mat(pi/4*ones((robot.NJ,1))), 2)
    T = robot.FK_tree(mat(pi/4*ones((robot.NJ,1))), robot.F_RF, 1)
    print tr2diff(T)
    J1 = robot.J_tree(mat(pi/4*ones((robot.NJ,1))), robot.F_RF, 1)
    J2 = robot.J_tree(mat(pi/4*ones((robot.NJ,1))), robot.F_RF, 2)
    J3 = robot.J_tree(mat(pi/4*ones((robot.NJ,1))), robot.F_LF, 3)
    com = robot.CoM(mat(zeros((robot.NJ,1))))
    com1 = robot.CoM(mat(zeros((robot.NJ,1))),2)
    print com
    print com1
    Jcom = robot.J_com_tree(mat(zeros((robot.NJ,1))))
    print Jcom
    Jcom1 = robot.J_com_tree(mat(zeros((robot.NJ,1))),2)    
    print Jcom1
    print CosTraj(10,0,1)
    #print spline_interp(CosTraj(10,0,1),2)
    a = vstack((mat(CosTraj(10,0,1)),mat(CosTraj(10,0,1)))).T
    spline_interp(a,2)
    #plot(CycloidTraj(50, 0, 10, 5))
    #plot(com_motion_generation(robot, mat(pi/4*ones((robot.NJ,1))), robot.LF, 30, 0, 0))
    Tdata_swF = stepping_motion_generation(robot, mat(zeros((robot.NJ,1))), robot.LF, 10, 10, 50)
    plot(com_motion_des(robot,q_zero,robot.LF,30,robot.CoM(mat(pi/4*ones((robot.NJ,1))))))
    
