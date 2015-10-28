# -*- coding: utf-8 -*-
"""
Created on Thu Jul 25 10:37:41 2013

@author: andypark
"""
## animate robot
# written on 07-22-13 
from numpy import *
from matplotlib.mlab import find
from construct_DRC_Hubo import *

## set up for plot animation
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph as pg
import pyqtgraph.opengl as gl  
    
# for the timer loop
index_ani_frame = 0
plot_coordinate_frames = 0    
 
## set up the figure
#app.processEvents()
app = QtGui.QApplication([])
view = gl.GLViewWidget()
view.show()

## create three grids, add each to the view
xgrid = gl.GLGridItem()
ygrid = gl.GLGridItem()
zgrid = gl.GLGridItem()
view.addItem(xgrid)
view.addItem(ygrid)
view.addItem(zgrid)


x = CosTraj(10,0,0)
x_pts = vstack([x,x,x]).transpose()
y_pts = vstack([x,x,x]).transpose()
z_pts = vstack([x,x,x]).transpose()

#### K = 1 
#x_line_1 = gl.GLLinePlotItem(pos=x_pts, color=(1,0,0,1), width = 1.)
#view.addItem(x_line_1) 
#y_line_1 = gl.GLLinePlotItem(pos=y_pts, color=(0,1,0,1), width = 1.)
#view.addItem(y_line_1)         
#z_line_1 = gl.GLLinePlotItem(pos=z_pts, color=(0,0,1,1), width = 1.)
#view.addItem(z_line_1)         
#
### K = 2 
#x_line_2 = gl.GLLinePlotItem(pos=x_pts, color=(1,0,0,1), width = 1.)
#view.addItem(x_line_2) 
#y_line_2 = gl.GLLinePlotItem(pos=y_pts, color=(0,1,0,1), width = 1.)
#view.addItem(y_line_2) 
#z_line_2 = gl.GLLinePlotItem(pos=z_pts, color=(0,0,1,1), width = 1.)
#view.addItem(z_line_2) 
#
### K = 3 
#x_line_3 = gl.GLLinePlotItem(pos=x_pts, color=(1,0,0,1), width = 1.)
#view.addItem(x_line_3)       
#y_line_3 = gl.GLLinePlotItem(pos=y_pts, color=(0,1,0,1), width = 1.)
#view.addItem(y_line_3) 
#z_line_3 = gl.GLLinePlotItem(pos=z_pts, color=(0,0,1,1), width = 1.)
#view.addItem(z_line_3) 
#
### K = 4 
#x_line_4 = gl.GLLinePlotItem(pos=x_pts, color=(1,0,0,1), width = 1.)
#view.addItem(x_line_4)       
#y_line_4 = gl.GLLinePlotItem(pos=y_pts, color=(0,1,0,1), width = 1.)
#view.addItem(y_line_4) 
#z_line_4 = gl.GLLinePlotItem(pos=z_pts, color=(0,0,1,1), width = 1.)
#view.addItem(z_line_4) 
#
### K = 5 
#x_line_5 = gl.GLLinePlotItem(pos=x_pts, color=(1,0,0,1), width = 1.)
#view.addItem(x_line_5)       
#y_line_5 = gl.GLLinePlotItem(pos=y_pts, color=(0,1,0,1), width = 1.)
#view.addItem(y_line_5) 
#z_line_5 = gl.GLLinePlotItem(pos=z_pts, color=(0,0,1,1), width = 1.)
#view.addItem(z_line_5) 
#
### K = 6 
#x_line_6 = gl.GLLinePlotItem(pos=x_pts, color=(1,0,0,1), width = 1.)
#view.addItem(x_line_6)       
#y_line_6 = gl.GLLinePlotItem(pos=y_pts, color=(0,1,0,1), width = 1.)
#view.addItem(y_line_6) 
#z_line_6 = gl.GLLinePlotItem(pos=z_pts, color=(0,0,1,1), width = 1.)
#view.addItem(z_line_6) 
#
### K = 7 
#x_line_7 = gl.GLLinePlotItem(pos=x_pts, color=(1,0,0,1), width = 1.)
#view.addItem(x_line_7)       
#y_line_7 = gl.GLLinePlotItem(pos=y_pts, color=(0,1,0,1), width = 1.)
#view.addItem(y_line_7) 
#z_line_7 = gl.GLLinePlotItem(pos=z_pts, color=(0,0,1,1), width = 1.)
#view.addItem(z_line_7) 

## K = 8 
x_line_8 = gl.GLLinePlotItem(pos=x_pts, color=(1,0,0,1), width = 1.)
view.addItem(x_line_8)       
y_line_8 = gl.GLLinePlotItem(pos=y_pts, color=(0,1,0,1), width = 1.)
view.addItem(y_line_8) 
z_line_8 = gl.GLLinePlotItem(pos=z_pts, color=(0,0,1,1), width = 1.)
view.addItem(z_line_8) 

### K = 9 
#x_line_9 = gl.GLLinePlotItem(pos=x_pts, color=(1,0,0,1), width = 1.)
#view.addItem(x_line_9)       
#y_line_9 = gl.GLLinePlotItem(pos=y_pts, color=(0,1,0,1), width = 1.)
#view.addItem(y_line_9) 
#z_line_9 = gl.GLLinePlotItem(pos=z_pts, color=(0,0,1,1), width = 1.)
#view.addItem(z_line_9) 
#
### K = 10
#x_line_10 = gl.GLLinePlotItem(pos=x_pts, color=(1,0,0,1), width = 1.)
#view.addItem(x_line_10)       
#y_line_10 = gl.GLLinePlotItem(pos=y_pts, color=(0,1,0,1), width = 1.)
#view.addItem(y_line_10) 
#z_line_10 = gl.GLLinePlotItem(pos=z_pts, color=(0,0,1,1), width = 1.)
#view.addItem(z_line_10)         
#
### K = 11 
#x_line_11 = gl.GLLinePlotItem(pos=x_pts, color=(1,0,0,1), width = 1.)
#view.addItem(x_line_11)       
#y_line_11 = gl.GLLinePlotItem(pos=y_pts, color=(0,1,0,1), width = 1.)
#view.addItem(y_line_11) 
#z_line_11 = gl.GLLinePlotItem(pos=z_pts, color=(0,0,1,1), width = 1.)
#view.addItem(z_line_11) 
#
### K = 12 
#x_line_12 = gl.GLLinePlotItem(pos=x_pts, color=(1,0,0,1), width = 1.)
#view.addItem(x_line_12)       
#y_line_12 = gl.GLLinePlotItem(pos=y_pts, color=(0,1,0,1), width = 1.)
#view.addItem(y_line_12) 
#z_line_12 = gl.GLLinePlotItem(pos=z_pts, color=(0,0,1,1), width = 1.)
#view.addItem(z_line_12) 
#
### K = 3 
#x_line_13 = gl.GLLinePlotItem(pos=x_pts, color=(1,0,0,1), width = 1.)
#view.addItem(x_line_13)       
#y_line_13 = gl.GLLinePlotItem(pos=y_pts, color=(0,1,0,1), width = 1.)
#view.addItem(y_line_13) 
#z_line_13 = gl.GLLinePlotItem(pos=z_pts, color=(0,0,1,1), width = 1.)
#view.addItem(z_line_13) 
#
### K = 4 
#x_line_14 = gl.GLLinePlotItem(pos=x_pts, color=(1,0,0,1), width = 1.)
#view.addItem(x_line_14)       
#y_line_14 = gl.GLLinePlotItem(pos=y_pts, color=(0,1,0,1), width = 1.)
#view.addItem(y_line_14) 
#z_line_14 = gl.GLLinePlotItem(pos=z_pts, color=(0,0,1,1), width = 1.)
#view.addItem(z_line_14) 

## K = 15 
x_line_15 = gl.GLLinePlotItem(pos=x_pts, color=(1,0,0,1), width = 1.)
view.addItem(x_line_15)       
y_line_15 = gl.GLLinePlotItem(pos=y_pts, color=(0,1,0,1), width = 1.)
view.addItem(y_line_15) 
z_line_15 = gl.GLLinePlotItem(pos=z_pts, color=(0,0,1,1), width = 1.)
view.addItem(z_line_15) 

### K = 16 
#x_line_16 = gl.GLLinePlotItem(pos=x_pts, color=(1,0,0,1), width = 1.)
#view.addItem(x_line_16)       
#y_line_16 = gl.GLLinePlotItem(pos=y_pts, color=(0,1,0,1), width = 1.)
#view.addItem(y_line_16) 
#z_line_16 = gl.GLLinePlotItem(pos=z_pts, color=(0,0,1,1), width = 1.)
#view.addItem(z_line_16) 
#
### K = 17 
#x_line_17 = gl.GLLinePlotItem(pos=x_pts, color=(1,0,0,1), width = 1.)
#view.addItem(x_line_17)       
#y_line_17 = gl.GLLinePlotItem(pos=y_pts, color=(0,1,0,1), width = 1.)
#view.addItem(y_line_17) 
#z_line_17 = gl.GLLinePlotItem(pos=z_pts, color=(0,0,1,1), width = 1.)
#view.addItem(z_line_17) 
#
### K = 18 
#x_line_18 = gl.GLLinePlotItem(pos=x_pts, color=(1,0,0,1), width = 1.)
#view.addItem(x_line_18)       
#y_line_18 = gl.GLLinePlotItem(pos=y_pts, color=(0,1,0,1), width = 1.)
#view.addItem(y_line_18) 
#z_line_18 = gl.GLLinePlotItem(pos=z_pts, color=(0,0,1,1), width = 1.)
#view.addItem(z_line_18) 
#
### K = 19 
#x_line_19 = gl.GLLinePlotItem(pos=x_pts, color=(1,0,0,1), width = 1.)
#view.addItem(x_line_19)       
#y_line_19 = gl.GLLinePlotItem(pos=y_pts, color=(0,1,0,1), width = 1.)
#view.addItem(y_line_19) 
#z_line_19 = gl.GLLinePlotItem(pos=z_pts, color=(0,0,1,1), width = 1.)
#view.addItem(z_line_19) 
#
### K = 20
#x_line_20 = gl.GLLinePlotItem(pos=x_pts, color=(1,0,0,1), width = 1.)
#view.addItem(x_line_20)       
#y_line_20 = gl.GLLinePlotItem(pos=y_pts, color=(0,1,0,1), width = 1.)
#view.addItem(y_line_20) 
#z_line_20 = gl.GLLinePlotItem(pos=z_pts, color=(0,0,1,1), width = 1.)
#view.addItem(z_line_20)   

## K = 21 
x_line_21 = gl.GLLinePlotItem(pos=x_pts, color=(1,0,0,1), width = 1.)
view.addItem(x_line_21)       
y_line_21 = gl.GLLinePlotItem(pos=y_pts, color=(0,1,0,1), width = 1.)
view.addItem(y_line_21) 
z_line_21 = gl.GLLinePlotItem(pos=z_pts, color=(0,0,1,1), width = 1.)
view.addItem(z_line_21) 

### K = 22 
#x_line_22 = gl.GLLinePlotItem(pos=x_pts, color=(1,0,0,1), width = 1.)
#view.addItem(x_line_22)       
#y_line_22 = gl.GLLinePlotItem(pos=y_pts, color=(0,1,0,1), width = 1.)
#view.addItem(y_line_22) 
#z_line_22 = gl.GLLinePlotItem(pos=z_pts, color=(0,0,1,1), width = 1.)
#view.addItem(z_line_22) 
#
### K = 23 
#x_line_23 = gl.GLLinePlotItem(pos=x_pts, color=(1,0,0,1), width = 1.)
#view.addItem(x_line_23)       
#y_line_23 = gl.GLLinePlotItem(pos=y_pts, color=(0,1,0,1), width = 1.)
#view.addItem(y_line_23) 
#z_line_23 = gl.GLLinePlotItem(pos=z_pts, color=(0,0,1,1), width = 1.)
#view.addItem(z_line_23) 
#
### K = 24 
#x_line_24 = gl.GLLinePlotItem(pos=x_pts, color=(1,0,0,1), width = 1.)
#view.addItem(x_line_24)       
#y_line_24 = gl.GLLinePlotItem(pos=y_pts, color=(0,1,0,1), width = 1.)
#view.addItem(y_line_24) 
#z_line_24 = gl.GLLinePlotItem(pos=z_pts, color=(0,0,1,1), width = 1.)
#view.addItem(z_line_24) 
#
### K = 25 
#x_line_25 = gl.GLLinePlotItem(pos=x_pts, color=(1,0,0,1), width = 1.)
#view.addItem(x_line_25)       
#y_line_25 = gl.GLLinePlotItem(pos=y_pts, color=(0,1,0,1), width = 1.)
#view.addItem(y_line_25) 
#z_line_25 = gl.GLLinePlotItem(pos=z_pts, color=(0,0,1,1), width = 1.)
#view.addItem(z_line_25) 
#
### K = 26 
#x_line_26 = gl.GLLinePlotItem(pos=x_pts, color=(1,0,0,1), width = 1.)
#view.addItem(x_line_26)       
#y_line_26 = gl.GLLinePlotItem(pos=y_pts, color=(0,1,0,1), width = 1.)
#view.addItem(y_line_26) 
#z_line_26 = gl.GLLinePlotItem(pos=z_pts, color=(0,0,1,1), width = 1.)
#view.addItem(z_line_26) 

## K = 27 
x_line_27 = gl.GLLinePlotItem(pos=x_pts, color=(1,0,0,1), width = 1.)
view.addItem(x_line_27)       
y_line_27 = gl.GLLinePlotItem(pos=y_pts, color=(0,1,0,1), width = 1.)
view.addItem(y_line_27) 
z_line_27 = gl.GLLinePlotItem(pos=z_pts, color=(0,0,1,1), width = 1.)
view.addItem(z_line_27) 

##################################################################

width_limb = 7.
##color_limb = (0.4,0.4,0.4,1)
color_limb = (1,1,1,1)
##color_hinge = (0.78,0.35,0.35,1)
color_hinge = (0.78,0.78,0.78,1)
## Left Arm 1-7
limb_1_2 = gl.GLLinePlotItem(pos= x_pts, color=color_limb, width = width_limb)
view.addItem(limb_1_2) 
limb_2_3 = gl.GLLinePlotItem(pos= x_pts, color=color_limb, width = width_limb)
view.addItem(limb_2_3) 
limb_3_4 = gl.GLLinePlotItem(pos= x_pts, color=color_limb, width = width_limb)
view.addItem(limb_3_4) 
limb_4_5 = gl.GLLinePlotItem(pos= x_pts, color=color_limb, width = width_limb)
view.addItem(limb_4_5) 
limb_5_6 = gl.GLLinePlotItem(pos= x_pts, color=color_limb, width = width_limb)
view.addItem(limb_5_6) 
limb_6_7 = gl.GLLinePlotItem(pos= x_pts, color=color_limb, width = width_limb)
view.addItem(limb_6_7) 

## Right Arm 8-14
limb_8_9 = gl.GLLinePlotItem(pos= x_pts, color=color_limb, width = width_limb)
view.addItem(limb_8_9) 
limb_9_10 = gl.GLLinePlotItem(pos= x_pts, color=color_limb, width = width_limb)
view.addItem(limb_9_10) 
limb_10_11 = gl.GLLinePlotItem(pos= x_pts, color=color_limb, width = width_limb)
view.addItem(limb_10_11) 
limb_11_12 = gl.GLLinePlotItem(pos= x_pts, color=color_limb, width = width_limb)
view.addItem(limb_11_12) 
limb_12_13 = gl.GLLinePlotItem(pos= x_pts, color=color_limb, width = width_limb)
view.addItem(limb_12_13) 
limb_13_14 = gl.GLLinePlotItem(pos= x_pts, color=color_limb, width = width_limb)
view.addItem(limb_13_14) 


## Left Leg 15-20
limb_15_16 = gl.GLLinePlotItem(pos= x_pts, color=color_limb, width = width_limb)
view.addItem(limb_15_16) 
limb_16_17 = gl.GLLinePlotItem(pos= x_pts, color=color_limb, width = width_limb)
view.addItem(limb_16_17) 
limb_17_18 = gl.GLLinePlotItem(pos= x_pts, color=color_limb, width = width_limb)
view.addItem(limb_17_18) 
limb_18_19 = gl.GLLinePlotItem(pos= x_pts, color=color_limb, width = width_limb)
view.addItem(limb_18_19) 
limb_19_20 = gl.GLLinePlotItem(pos= x_pts, color=color_limb, width = width_limb)
view.addItem(limb_19_20) 

## Right Leg 21-26
limb_21_22 = gl.GLLinePlotItem(pos= x_pts, color=color_limb, width = width_limb)
view.addItem(limb_21_22) 
limb_22_23 = gl.GLLinePlotItem(pos= x_pts, color=color_limb, width = width_limb)
view.addItem(limb_22_23) 
limb_23_24 = gl.GLLinePlotItem(pos= x_pts, color=color_limb, width = width_limb)
view.addItem(limb_23_24) 
limb_24_25 = gl.GLLinePlotItem(pos= x_pts, color=color_limb, width = width_limb)
view.addItem(limb_24_25) 
limb_25_26 = gl.GLLinePlotItem(pos= x_pts, color=color_limb, width = width_limb)
view.addItem(limb_25_26)

## Left Shoulder Hinge 1 27
limb_1_27 = gl.GLLinePlotItem(pos= x_pts, color=color_hinge, width = width_limb)
view.addItem(limb_1_27)

## Right Shoulder Hinge 8 27
limb_8_27 = gl.GLLinePlotItem(pos= x_pts, color=color_hinge, width = width_limb)
view.addItem(limb_8_27)

## Left Thigh Hinge 0 15
limb_0_15 = gl.GLLinePlotItem(pos= x_pts, color=color_hinge, width = width_limb)
view.addItem(limb_0_15)

## Right Thigh Hinge 0 22
limb_0_22 = gl.GLLinePlotItem(pos= x_pts, color=color_hinge, width = width_limb)
view.addItem(limb_0_22)

## Back Bone Hinge 0 27
limb_0_27 = gl.GLLinePlotItem(pos= x_pts, color=color_hinge, width = width_limb)
view.addItem(limb_0_27)

#################################################################
## Draw CoM

width_com = 2.
##color_limb = (0.4,0.4,0.4,1)
color_line_com = (1,0,0,1)
color_com = (0,1,0,1)
size_com = 10.

line_com = gl.GLLinePlotItem(pos= x_pts, color=color_line_com, width = width_com)
view.addItem(line_com)

point_com = gl.GLScatterPlotItem(pos = x_pts, color=color_com, size =size_com)
view.addItem(point_com)

#################################################################
## Draw Feet

length = 0.001*(140+79.2)
width = 0.001*130
height = 0.001
l2 = length*0.5
lf = 0.001*140
lr = 0.001*79.25
w2 = width*0.5
h2 = height*0.5

verts = array([
    [lf, w2, h2],[-lr, w2, h2],[lf, w2, -h2],[-lr, w2, -h2],[lf, -w2, h2],[-lr, -w2, h2],[lf, -w2, -h2],[-lr, -w2, -h2],
])
faces = array([
    [1, 0, 2],[1, 2, 3],[0, 4, 6],[0, 6, 2],[7, 4, 5],[7, 6, 4],[3, 5, 1],[3, 7, 5],[5, 0, 1],[5, 4, 0],[6, 7, 3],[6, 3, 2],
])
c_box = [0.5, 0.5, 0.78, 0.2]
colors = array([
    c_box,c_box,c_box,c_box,c_box,c_box,c_box,c_box,c_box,c_box,c_box,c_box,
])

## Mesh item will automatically compute face normals.
box_1 = gl.GLMeshItem(vertexes=verts, faces=faces, faceColors=colors, smooth=False)
#box_1.translate(5, 5, h2)
#box_1.rotate(90,1,0,0)
#m1.setGLOptions('additive')
view.addItem(box_1)

# RIGHT FOOT
#    length = 0.001
#    width = 0.001
#    height = 0.001
#    l2 = length*0.5
#    w2 = width*0.5
#    h2 = height*0.5
#    
#    verts = array([
#        [l2, w2, h2],[-l2, w2, h2],[l2, w2, -h2],[-l2, w2, -h2],[l2, -w2, h2],[-l2, -w2, h2],[l2, -w2, -h2],      [-l2, -w2, -h2],
#    ])
#    faces = array([
#        [1, 0, 2],[1, 2, 3],[0, 4, 6],[0, 6, 2],[7, 4, 5],[7, 6, 4],[3, 5, 1],[3, 7, 5],[5, 0, 1],[5, 4, 0],[6, 7, 3],[6, 3, 2],
#    ])
#    c_box = [0.72, 0.78, 0.98, 0.5]
#    colors = array([
#        c_box,c_box,c_box,c_box,c_box,c_box,c_box,c_box,c_box,c_box,c_box,c_box,
#    ])

## Mesh item will automatically compute face normals.
box_2 = gl.GLMeshItem(vertexes=verts, faces=faces, faceColors=colors, smooth=False)
#box_1.translate(5, 5, h2)
#box_1.rotate(90,1,0,0)
#m1.setGLOptions('additive')
view.addItem(box_2)

#################################################################

def Ani_robot(self, qdata, REF_frame = 0):
    #################################################################
    
    
    
   
    
    #init_Ani_robot()
    Nframes = size(qdata,0)
    if Nframes > 1:
        if size(REF_frame) == 1:
            REF_frame = REF_frame*ones((Nframes,1))
    REF_frame_data = REF_frame
    
    ## find the index of frame where support foot changes
    if size(REF_frame_data) > 1:
        indicies_change_ref = find(hstack(([0],diff(REF_frame_data[:,0]))))
        k = 0
    else:
        indicies_change_ref = zeros((Nframes))
        k = 0
    
    Tbase_data = zeros((Nframes,3))
    DHdata = zeros((112,4,Nframes))
    xdata_LH = zeros((Nframes,3))
    xdata_RH = zeros((Nframes,3))
    xdata_LF = zeros((Nframes,3))
    xdata_RF = zeros((Nframes,3))
    CoMdata = zeros((Nframes,3))
    
    for I in range(Nframes):
        q_tmp = qdata[I,].T
        
        if size(REF_frame_data) > 1:
            REF_FRAME = REF_frame_data[I,0]
        else:
            REF_FRAME = REF_frame_data
            
        if I == 0:  
            Tbase = transl(0,0,0)
        else:
            if size(indicies_change_ref,0) > 0:
                if(I == indicies_change_ref[k]):
                    if(REF_FRAME == 2): # Left Foot
                        #DHB2 = transl(transl(DHLL06)-transl(DHRL06));
                        Tbase = Tbase*transl(transl(DHRL06)-transl(DHLL06))
                    elif(REF_FRAME == 3): # Right Foot
                        Tbase = Tbase*transl(transl(DHLL06)-transl(DHRL06))
                        #DHB2 = transl(0,0,0);
                    if(size(indicies_change_ref,0) > k):
                        k = k + 1
            
        
        # store the position of the reference
        Tbase_data[I,] = transl(Tbase).T
        
        ## change of reference 
        DHLF = self.FK_tree(q_tmp,self.F_LAR)
        DHRF = self.FK_tree(q_tmp,self.F_LAR)
            
        if REF_FRAME == 2: ## LEFT FOOT
            DH_Base = Tbase*inv(DHLF)
            
        elif REF_FRAME == 3: ## RIGHT FOOT
            DH_Base = Tbase*inv(DHRF)
        
        else: ## Body
            DH_Base = Tbase
        
        
            
        # waist position
        DHB2 = DH_Base*self.FK_tree(q_tmp,self.J_HPY)
        
        # matrices from DH table - left_arm

        DHLA01 = DH_Base*self.FK_tree(q_tmp,self.F_LSP)
        DHLA02 = DH_Base*self.FK_tree(q_tmp,self.F_LSR)
        DHLA03 = DH_Base*self.FK_tree(q_tmp,self.F_LSY)
        DHLA04 = DH_Base*self.FK_tree(q_tmp,self.F_LEP)
        DHLA05 = DH_Base*self.FK_tree(q_tmp,self.F_LWY)
        DHLA06 = DH_Base*self.FK_tree(q_tmp,self.F_LWP)
        DHLA07 = DH_Base*self.FK_tree(q_tmp,self.F_LWR)

        DHLA = vstack((DHLA01,DHLA02,DHLA03,DHLA04,DHLA05,DHLA06,DHLA07))
        
        # matrices from DH table - right_arm

        DHRA01 = DH_Base*self.FK_tree(q_tmp,self.F_RSP)
        DHRA02 = DH_Base*self.FK_tree(q_tmp,self.F_RSR)
        DHRA03 = DH_Base*self.FK_tree(q_tmp,self.F_RSY)
        DHRA04 = DH_Base*self.FK_tree(q_tmp,self.F_REP)
        DHRA05 = DH_Base*self.FK_tree(q_tmp,self.F_RWY)
        DHRA06 = DH_Base*self.FK_tree(q_tmp,self.F_RWP)
        DHRA07 = DH_Base*self.FK_tree(q_tmp,self.F_RWR)

        DHRA = vstack((DHRA01,DHRA02,DHRA03,DHRA04,DHRA05,DHRA06,DHRA07))
        
        # matrices from DH table - left_leg

        DHLL01 = DH_Base*self.FK_tree(q_tmp,self.F_LHY)
        DHLL02 = DH_Base*self.FK_tree(q_tmp,self.F_LHR)
        DHLL03 = DH_Base*self.FK_tree(q_tmp,self.F_LHP)
        DHLL04 = DH_Base*self.FK_tree(q_tmp,self.F_LKP)
        DHLL05 = DH_Base*self.FK_tree(q_tmp,self.F_LAP)
        DHLL06 = DH_Base*self.FK_tree(q_tmp,self.F_LAR)
        
        DHLL = vstack((DHLL01,DHLL02,DHLL03,DHLL04,DHLL05,DHLL06))
        
        # matrices from DH table - right_leg
        
        DHRL01 = DH_Base*self.FK_tree(q_tmp,self.F_RHY)
        DHRL02 = DH_Base*self.FK_tree(q_tmp,self.F_RHR)
        DHRL03 = DH_Base*self.FK_tree(q_tmp,self.F_RHP)
        DHRL04 = DH_Base*self.FK_tree(q_tmp,self.F_RKP)
        DHRL05 = DH_Base*self.FK_tree(q_tmp,self.F_RAP)
        DHRL06 = DH_Base*self.FK_tree(q_tmp,self.F_RAR)
        
        DHRL = vstack((DHRL01,DHRL02,DHRL03,DHRL04,DHRL05,DHRL06))
        
        # Base1 - shoulder center
        DHB1 = DH_Base*transl(0,0,0)
        DH = vstack((DHB2, DHLA, DHRA, DHLL, DHRL, DHB1))
        
        DHdata[:,:,I] = DH
        xdata_LH[I,] = transl(DHLA07).T
        xdata_RH[I,] = transl(DHRA07).T
        xdata_LF[I,] = transl(DHLL06).T
        xdata_RF[I,] = transl(DHRL06).T
        
        CoMdata[I,] = self.CoM(q_tmp,REF_FRAME)
    
    frame_flag = 1
    length = 0.1
    N_DH = size(DH,0)/4
    origin_data = zeros((3,N_DH))
    
    #for I in range(1):
    #global I
    
    I = 0
    DH = DHdata[:,:,I]

          
    def plot_update():
        global index_ani_frame
#        global  x_line_1, y_line_1, z_line_1
#        global  x_line_2, y_line_2, z_line_2
#        global  x_line_3, y_line_3, z_line_3
#        global  x_line_4, y_line_4, z_line_4
#        global  x_line_5, y_line_5, z_line_5
#        global  x_line_6, y_line_6, z_line_6
#        global  x_line_7, y_line_7, z_line_7
        global  x_line_8, y_line_8, z_line_8
#        global  x_line_9, y_line_9, z_line_9
#        global  x_line_10, y_line_10, z_line_10
#        global  x_line_11, y_line_11, z_line_11
#        global  x_line_12, y_line_12, z_line_12
#        global  x_line_13, y_line_13, z_line_13
#        global  x_line_14, y_line_14, z_line_14
        global  x_line_15, y_line_15, z_line_15
#        global  x_line_16, y_line_16, z_line_16
#        global  x_line_17, y_line_17, z_line_17
#        global  x_line_18, y_line_18, z_line_18
#        global  x_line_19, y_line_19, z_line_19
#        global  x_line_20, y_line_20, z_line_20
        global  x_line_21, y_line_21, z_line_21
#        global  x_line_22, y_line_22, z_line_22
#        global  x_line_23, y_line_23, z_line_23
#        global  x_line_24, y_line_24, z_line_24
#        global  x_line_25, y_line_25, z_line_25
#        global  x_line_26, y_line_26, z_line_26
        global  x_line_27, y_line_27, z_line_27
        global  box_1, box_2
        global  color_limb, width_limb, color_hinge
        ## Left Arm 1-7
        global  limb_1_2, limb_2_3, limb_3_4, limb_4_5, limb_5_6, limb_6_7
        ## Right Arm 8-14
        global  limb_8_9, limb_9_10, limb_10_11, limb_11_12, limb_12_13, limb_13_14
        ## Left Leg 15-20
        global  limb_15_16, limb_16_17, limb_17_18, limb_18_19, limb_19_20
        ## Right Leg 21-26
        global  limb_21_22, limb_22_23, limb_23_24, limb_24_25, limb_25_26
        ## Left Shoulder Hinge 1 27
        ## Right Shoulder Hinge 8 27
        ## Left Thigh Hinge 0 15
        ## Right Thigh Hinge 0 22
        ## Back Bone Hinge 0 27
        global limb_1_27, limb_8_27, limb_0_15, limb_0_22, limb_0_27
        global plot_coordinate_frames
        
        
        DH = DHdata[:,:,index_ani_frame]
        
        ## K = 8 LH
        K = 8
        (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
        x_line_8.setData(pos= x_pts, color=(1,0,0,1), width =1.)
        y_line_8.setData(pos= y_pts, color=(0,1,0,1), width =1.)
        z_line_8.setData(pos= z_pts, color=(0,0,1,1), width =1.)
        
        ## K = 15 RH
        K = 15
        (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
        x_line_15.setData(pos= x_pts, color=(1,0,0,1), width =1.)
        y_line_15.setData(pos= y_pts, color=(0,1,0,1), width =1.)
        z_line_15.setData(pos= z_pts, color=(0,0,1,1), width =1.)
        
        ## K = 21 LF
        K = 21
        (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
        x_line_21.setData(pos= x_pts, color=(1,0,0,1), width =1.)
        y_line_21.setData(pos= y_pts, color=(0,1,0,1), width =1.)
        z_line_21.setData(pos= z_pts, color=(0,0,1,1), width =1.)
        
        ## K = 27 RF
        K = 27
        (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
        x_line_27.setData(pos= x_pts, color=(1,0,0,1), width =1.)
        y_line_27.setData(pos= y_pts, color=(0,1,0,1), width =1.)
        z_line_27.setData(pos= z_pts, color=(0,0,1,1), width =1.)
            
        
        if 0:
            ## display coordinate frames
            ## K = 1 
            K = 1
            (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
            x_line_1.setData(pos= x_pts, color=(1,0,0,1), width =1.)
            y_line_1.setData(pos= y_pts, color=(0,1,0,1), width =1.)
            z_line_1.setData(pos= z_pts, color=(0,0,1,1), width =1.)
            
            ## K = 2 
            K = 2
            (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
            x_line_2.setData(pos= x_pts, color=(1,0,0,1), width =1.)
            y_line_2.setData(pos= y_pts, color=(0,1,0,1), width =1.)
            z_line_2.setData(pos= z_pts, color=(0,0,1,1), width =1.)
            
            
            ## K = 3 
            K = 3
            (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
            x_line_3.setData(pos= x_pts, color=(1,0,0,1), width =1.)
            y_line_3.setData(pos= y_pts, color=(0,1,0,1), width =1.)
            z_line_3.setData(pos= z_pts, color=(0,0,1,1), width =1.)
            
            ## K = 4 
            K = 4
            (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
            x_line_4.setData(pos= x_pts, color=(1,0,0,1), width =1.)
            y_line_4.setData(pos= y_pts, color=(0,1,0,1), width =1.)
            z_line_4.setData(pos= z_pts, color=(0,0,1,1), width =1.)
            
            ## K = 5 
            K = 5
            (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
            x_line_5.setData(pos= x_pts, color=(1,0,0,1), width =1.)
            y_line_5.setData(pos= y_pts, color=(0,1,0,1), width =1.)
            z_line_5.setData(pos= z_pts, color=(0,0,1,1), width =1.)
            
            ## K = 6 
            K = 6
            (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
            x_line_6.setData(pos= x_pts, color=(1,0,0,1), width =1.)
            y_line_6.setData(pos= y_pts, color=(0,1,0,1), width =1.)
            z_line_6.setData(pos= z_pts, color=(0,0,1,1), width =1.)
            
            ## K = 7 
            K = 7
            (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
            x_line_7.setData(pos= x_pts, color=(1,0,0,1), width =1.)
            y_line_7.setData(pos= y_pts, color=(0,1,0,1), width =1.)
            z_line_7.setData(pos= z_pts, color=(0,0,1,1), width =1.)
            
            ## K = 8 LH
            K = 8
            (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
            x_line_8.setData(pos= x_pts, color=(1,0,0,1), width =1.)
            y_line_8.setData(pos= y_pts, color=(0,1,0,1), width =1.)
            z_line_8.setData(pos= z_pts, color=(0,0,1,1), width =1.)
            
            ## K = 9 
            K = 9
            (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
            x_line_9.setData(pos= x_pts, color=(1,0,0,1), width =1.)
            y_line_9.setData(pos= y_pts, color=(0,1,0,1), width =1.)
            z_line_9.setData(pos= z_pts, color=(0,0,1,1), width =1.)
            
            ## K = 10
            K = 10
            (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
            x_line_10.setData(pos= x_pts, color=(1,0,0,1), width =1.)
            y_line_10.setData(pos= y_pts, color=(0,1,0,1), width =1.)
            z_line_10.setData(pos= z_pts, color=(0,0,1,1), width =1.)        
            
            ## K = 11 
            K = 11
            (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
            x_line_11.setData(pos= x_pts, color=(1,0,0,1), width =1.)
            y_line_11.setData(pos= y_pts, color=(0,1,0,1), width =1.)
            z_line_11.setData(pos= z_pts, color=(0,0,1,1), width =1.)
            
            ## K = 12 
            K = 12
            (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
            x_line_12.setData(pos= x_pts, color=(1,0,0,1), width =1.)
            y_line_12.setData(pos= y_pts, color=(0,1,0,1), width =1.)
            z_line_12.setData(pos= z_pts, color=(0,0,1,1), width =1.)
            
            ## K = 13 
            K = 13
            (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
            x_line_13.setData(pos= x_pts, color=(1,0,0,1), width =1.)
            y_line_13.setData(pos= y_pts, color=(0,1,0,1), width =1.)
            z_line_13.setData(pos= z_pts, color=(0,0,1,1), width =1.)
            
            ## K = 14 
            K = 14
            (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
            x_line_14.setData(pos= x_pts, color=(1,0,0,1), width =1.)
            y_line_14.setData(pos= y_pts, color=(0,1,0,1), width =1.)
            z_line_14.setData(pos= z_pts, color=(0,0,1,1), width =1.)
            
            ## K = 15 RH
            K = 15
            (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
            x_line_15.setData(pos= x_pts, color=(1,0,0,1), width =1.)
            y_line_15.setData(pos= y_pts, color=(0,1,0,1), width =1.)
            z_line_15.setData(pos= z_pts, color=(0,0,1,1), width =1.)
            
            ## K = 16 
            K = 16
            (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
            x_line_16.setData(pos= x_pts, color=(1,0,0,1), width =1.)
            y_line_16.setData(pos= y_pts, color=(0,1,0,1), width =1.)
            z_line_16.setData(pos= z_pts, color=(0,0,1,1), width =1.)
            
            ## K = 17 
            K = 17
            (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
            x_line_17.setData(pos= x_pts, color=(1,0,0,1), width =1.)
            y_line_17.setData(pos= y_pts, color=(0,1,0,1), width =1.)
            z_line_17.setData(pos= z_pts, color=(0,0,1,1), width =1.)
            
            ## K = 18 
            K = 18
            (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
            x_line_18.setData(pos= x_pts, color=(1,0,0,1), width =1.)
            y_line_18.setData(pos= y_pts, color=(0,1,0,1), width =1.)
            z_line_18.setData(pos= z_pts, color=(0,0,1,1), width =1.)
            
            ## K = 19 
            K = 19
            (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
            x_line_19.setData(pos= x_pts, color=(1,0,0,1), width =1.)
            y_line_19.setData(pos= y_pts, color=(0,1,0,1), width =1.)
            z_line_19.setData(pos= z_pts, color=(0,0,1,1), width =1.)
            
            ## K = 20
            K = 20
            (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
            x_line_20.setData(pos= x_pts, color=(1,0,0,1), width =1.)
            y_line_20.setData(pos= y_pts, color=(0,1,0,1), width =1.)
            z_line_20.setData(pos= z_pts, color=(0,0,1,1), width =1.)  
            
            ## K = 21 LF
            K = 21
            (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
            x_line_21.setData(pos= x_pts, color=(1,0,0,1), width =1.)
            y_line_21.setData(pos= y_pts, color=(0,1,0,1), width =1.)
            z_line_21.setData(pos= z_pts, color=(0,0,1,1), width =1.)
            
            ## K = 22 
            K = 22
            (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
            x_line_22.setData(pos= x_pts, color=(1,0,0,1), width =1.)
            y_line_22.setData(pos= y_pts, color=(0,1,0,1), width =1.)
            z_line_22.setData(pos= z_pts, color=(0,0,1,1), width =1.) 
            
            ## K = 23 
            K = 23
            (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
            x_line_23.setData(pos= x_pts, color=(1,0,0,1), width =1.)
            y_line_23.setData(pos= y_pts, color=(0,1,0,1), width =1.)
            z_line_23.setData(pos= z_pts, color=(0,0,1,1), width =1.)
            
            ## K = 24 
            K = 24
            (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
            x_line_24.setData(pos= x_pts, color=(1,0,0,1), width =1.)
            y_line_24.setData(pos= y_pts, color=(0,1,0,1), width =1.)
            z_line_24.setData(pos= z_pts, color=(0,0,1,1), width =1.)
            
            ## K = 25 
            K = 25
            (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
            x_line_25.setData(pos= x_pts, color=(1,0,0,1), width =1.)
            y_line_25.setData(pos= y_pts, color=(0,1,0,1), width =1.)
            z_line_25.setData(pos= z_pts, color=(0,0,1,1), width =1.)
            
            ## K = 26 
            K = 26
            (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
            x_line_26.setData(pos= x_pts, color=(1,0,0,1), width =1.)
            y_line_26.setData(pos= y_pts, color=(0,1,0,1), width =1.)
            z_line_26.setData(pos= z_pts, color=(0,0,1,1), width =1.)
            
            ## K = 27 RF
            K = 27
            (x_pts, y_pts, z_pts) = CoordinateFrame(DH, K-1)                          
            x_line_27.setData(pos= x_pts, color=(1,0,0,1), width =1.)
            y_line_27.setData(pos= y_pts, color=(0,1,0,1), width =1.)
            z_line_27.setData(pos= z_pts, color=(0,0,1,1), width =1.)
            
            #################################################################

   
        ## Left Arm 1-7
        pts = Origin_Frame(DH, 1, 2)
        limb_1_2.setData(pos= pts, color=color_limb, width = width_limb)
        pts = Origin_Frame(DH, 2, 3)
        limb_2_3.setData(pos= pts, color=color_limb, width = width_limb)
        pts = Origin_Frame(DH, 3, 4)
        limb_3_4.setData(pos= pts, color=color_limb, width = width_limb)
        pts = Origin_Frame(DH, 4, 5)
        limb_4_5.setData(pos= pts, color=color_limb, width = width_limb)
        pts = Origin_Frame(DH, 5, 6)
        limb_5_6.setData(pos= pts, color=color_limb, width = width_limb)
        pts = Origin_Frame(DH, 6, 7)
        limb_6_7.setData(pos= pts, color=color_limb, width = width_limb)            
        
        ## Right Arm 8-14
        pts = Origin_Frame(DH, 8, 9)
        limb_8_9.setData(pos= pts, color=color_limb, width = width_limb)
        pts = Origin_Frame(DH, 9, 10)
        limb_9_10.setData(pos= pts, color=color_limb, width = width_limb)
        pts = Origin_Frame(DH, 10, 11)
        limb_10_11.setData(pos= pts, color=color_limb, width = width_limb)
        pts = Origin_Frame(DH, 11, 12)
        limb_11_12.setData(pos= pts, color=color_limb, width = width_limb)
        pts = Origin_Frame(DH, 12, 13)
        limb_12_13.setData(pos= pts, color=color_limb, width = width_limb)
        pts = Origin_Frame(DH, 13, 14)
        limb_13_14.setData(pos= pts, color=color_limb, width = width_limb)   
        
        
        ## Left Leg 15-20
        pts = Origin_Frame(DH, 15, 16)
        limb_15_16.setData(pos= pts, color=color_limb, width = width_limb)
        pts = Origin_Frame(DH, 16, 17)
        limb_16_17.setData(pos= pts, color=color_limb, width = width_limb)
        pts = Origin_Frame(DH, 17, 18)
        limb_17_18.setData(pos= pts, color=color_limb, width = width_limb)
        pts = Origin_Frame(DH, 18, 19)
        limb_18_19.setData(pos= pts, color=color_limb, width = width_limb)
        pts = Origin_Frame(DH, 19, 20)
        limb_19_20.setData(pos= pts, color=color_limb, width = width_limb)
        
        ## Right Leg 21-26
        pts = Origin_Frame(DH, 21, 22)
        limb_21_22.setData(pos= pts, color=color_limb, width = width_limb)
        pts = Origin_Frame(DH, 22, 23)
        limb_22_23.setData(pos= pts, color=color_limb, width = width_limb)
        pts = Origin_Frame(DH, 23, 24)
        limb_23_24.setData(pos= pts, color=color_limb, width = width_limb)
        pts = Origin_Frame(DH, 24, 25)
        limb_24_25.setData(pos= pts, color=color_limb, width = width_limb)
        pts = Origin_Frame(DH, 25, 26)
        limb_25_26.setData(pos= pts, color=color_limb, width = width_limb)
        
        ## Left Shoulder Hinge 1 27
        pts = Origin_Frame(DH, 1, 27)
        limb_1_27.setData(pos= pts, color=color_hinge, width = width_limb)
        
        ## Right Shoulder Hinge 8 27
        pts = Origin_Frame(DH, 8, 27)
        limb_8_27.setData(pos= pts, color=color_hinge, width = width_limb)
    
        ## Left Thigh Hinge 0 15
        pts = Origin_Frame(DH, 0, 15)
        limb_0_15.setData(pos= pts, color=color_hinge, width = width_limb)
    
        ## Right Thigh Hinge 0 22
        pts = Origin_Frame(DH, 0, 22)
        limb_0_22.setData(pos= pts, color=color_hinge, width = width_limb)
    
        ## Back Bone Hinge 0 27
        pts = Origin_Frame(DH, 0, 27)
        limb_0_27.setData(pos= pts, color=color_hinge, width = width_limb)
        
        #################################################################
        ## Draw CoM
        
        CoM = CoMdata[index_ani_frame,]
        
        com_line_pts = vstack(([CoM[0],CoM[0]],[CoM[1],CoM[1]],[0,CoM[2]])).T
        line_com.setData(pos= com_line_pts, color=color_line_com, width = width_com)
        
        com_pts = vstack(([CoM[0]],[CoM[1]],[CoM[2]])).T
        point_com.setData(pos = com_pts, color=color_com, size=size_com)
        
        #################################################################
        ## Draw Feet
        
        K1 = 21 - 1
        T_LF = DH[K1*4:K1*4+4,0:4]
        K2 = 27 - 1
        T_RF = DH[K2*4:K2*4+4,0:4]
        #T_LF = transl(1,2,3)
        #T_RF = transl(2,3,4)
        
        
        # LEFT FOOT
        
        
        ## Mesh item will automatically compute face normals.
        #box_1 = gl.GLMeshItem(vertexes=verts, faces=faces, faceColors=colors, smooth=False)
        box_1.setMeshData(vertexes=verts, faces=faces, faceColors=colors, smooth=False)            
        box_1.setTransform(QtGui.QMatrix4x4(reshape(array(T_LF),16)))
        #box_1.rotate(90,1,0,0)
        #m1.setGLOptions('additive')
        #view.addItem(box_1)
        
        # RIGHT FOOT
        
        ## Mesh item will automatically compute face normals.
        box_2.setMeshData(vertexes=verts, faces=faces, faceColors=colors, smooth=False)
        box_2.setTransform(QtGui.QMatrix4x4(reshape(array(T_RF),16)))
        #box_1.rotate(90,1,0,0)
        #m1.setGLOptions('additive')
        #view.addItem(box_2)
    
        #################################################################
        
        index_ani_frame += 1
        
        if index_ani_frame == Nframes:
            index_ani_frame = 0
                
                #print K
                #print T
        
                
    
    

    timer = QtCore.QTimer()
    timer.timeout.connect(plot_update)
    timer.start(50)
    QtGui.QApplication.instance().exec_()        
#        while 1:
#            #time.sleep(0.02)
