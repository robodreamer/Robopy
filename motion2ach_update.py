import hubo_ach as hubo
import ach
import sys
import time
from ctypes import *
from numpy import *
from numpy import linalg
from time import *

def moveRobot(trajectory_matrix, delay = 0.1, sendDataToAch = 1):
    #print trajectory_matrix
    # The joint array
    #default_trajectory=matrix(zeros((100,27)))
#    for row_iterator in range(0,100):
#        default_trajectory[row_iterator,23]=-(float)(row_iterator)/100
#    trajectory_matrix=default_trajectory
    joint_array=["WST","LHY", "LHR", "LHP", "LKN", "LAP", "LAR", "RHY", "RHR", "RHP", "RKN", "RAP", "RAR", "LSP", "LSR", "LSY", "LEB", "LWY", "LWP", "LWR", "RSP", "RSR", "RSY", "REB", "RWY", "RWP", "RWR"];
    
    no_of_rows=trajectory_matrix.shape[0]
    no_of_columns=trajectory_matrix.shape[1]
    converted_trajectory_matrix=convertToHuboAch(trajectory_matrix, no_of_rows, no_of_columns, joint_array, writeToFile=1)

    if (no_of_columns != 27):
        print "error: number of coluumns are not 27:  " + str(no_of_columns)+" columns found"
    
    # Open Hubo-Ach feed-forward and feed-back (reference and state) channels
    s = ach.Channel(hubo.HUBO_CHAN_STATE_NAME)
    r = ach.Channel(hubo.HUBO_CHAN_REF_NAME)
    s.flush()
    r.flush()

    # feed-forward will now be refered to as "state"
    state = hubo.HUBO_STATE()
    # feed-back will now be refered to as "ref"
    ref = hubo.HUBO_REF()

    for trajectory_iterator in range(0,no_of_rows):
        # Get the current feed-forward (state)
        [statuss, framesizes] = s.get(state, wait=False, last=False)
        #Send the commands
        for joint in range (0,32):
            #print converted_trajectory_matrix[trajectory_iterator,joint]
            ref.ref[joint]=converted_trajectory_matrix[trajectory_iterator,joint]
    
        # Write to the feed-forward channel
	if (sendDataToAch==1):
	        r.put(ref)
        sleep(delay)
    # Close the connection to the channels
    r.close()
    s.close()

    return 0
    
    
def jointMap (jointname, joint_array):
    for i in range(0,size(joint_array)):
        if (joint_array[i]==jointname):
            return i
    return -1

def getJointName(joint):
    if (joint==0):
        return "WST"
    elif (joint==1):
        return "NKY"
    elif (joint==2):
        return "NK1"
    elif (joint==3):
        return "NK2"
    elif (joint==4):
        return "LSP"
    elif (joint==5):
        return "LSR"
    elif (joint==6):
        return "LSY"
    elif (joint==7):
        return "LEB"
    elif (joint==8):
        return "LWY"
    elif (joint==9):
        return "LWR"
    elif (joint==10):
        return "LWP"
    elif (joint==11):
        return "RSP"
    elif (joint==12):
        return "RSR"
    elif (joint==13):
        return "RSY"
    elif (joint==14):
        return "REB"
    elif (joint==15):
        return "RWY"
    elif (joint==16):
        return "RWR"
    elif (joint==17):
        return "RWP"
    elif (joint==18):
        return "NULL"
    elif (joint==19):
        return "LHY"
    elif (joint==20):
        return "LHR"
    elif (joint==21):
        return "LHP"
    elif (joint==22):
        return "LKN"
    elif (joint==23):
        return "LAP"
    elif (joint==24):
        return "LAR"
    elif (joint==25):
        return "NULL"
    elif (joint==26):
        return "RHY"
    elif (joint==27):
        return "RHR"
    elif (joint==28):
        return "RHP"
    elif (joint==29):
        return "RKN"
    elif (joint==30):
        return "RAP"
    elif (joint==31):
        return "RAR"

def motion2file(in_matrix):
     file_matrix = open('trajectoryGenerated.traj', 'w')
     no_of_rows=in_matrix.shape[0]
     no_of_columns=in_matrix.shape[1]
     for row_iterator in range(0, no_of_rows):
          row_to_be_written=""
          for column_iterator in range(0, no_of_columns):
               row_to_be_written=row_to_be_written + str(in_matrix[row_iterator, column_iterator])+", "
          row_to_be_written=row_to_be_written+"\n"
          file_matrix.write(row_to_be_written)

def convertToHuboAch(trajectory_matrix, no_of_rows, no_of_columns, joint_array, writeToFile):
    no_of_columns=32
    converted_matrix=matrix([[0.0 for col in range(no_of_columns)] for row in range(no_of_rows)])
    for joint in range(0,no_of_columns):
        jointname=getJointName(joint)
        joint_in_our_array=jointMap(jointname, joint_array)
        for trajectory_setpoint in range(0,no_of_rows):
            if (joint_in_our_array>=0):
                converted_matrix[trajectory_setpoint,joint]=trajectory_matrix[trajectory_setpoint,joint_in_our_array]
            else:
                converted_matrix[trajectory_setpoint, joint]=0.0
    motion2file(converted_matrix)
    return converted_matrix
   




     
