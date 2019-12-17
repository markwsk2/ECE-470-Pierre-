# -*- coding: utf-8 -*-
"""
Created on Wed Nov 27 15:59:15 2019

@author: aweso
"""


import numpy as np
from scipy.linalg import expm
import vrep

vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)



if clientID!=-1:
    print ('Connected to remote API server')

else:
    print ('Connection unsuccessful')
    exit

jointHandles = [0,0,0,0,0,0]

errorCode,jointHandles[0]=vrep.simxGetObjectHandle(clientID, 'UR3_joint1',vrep.simx_opmode_oneshot_wait)
errorCode,jointHandles[1]=vrep.simxGetObjectHandle(clientID, 'UR3_joint2',vrep.simx_opmode_oneshot_wait)
errorCode,jointHandles[2]=vrep.simxGetObjectHandle(clientID, 'UR3_joint3',vrep.simx_opmode_oneshot_wait)
errorCode,jointHandles[3]=vrep.simxGetObjectHandle(clientID, 'UR3_joint4',vrep.simx_opmode_oneshot_wait)
errorCode,jointHandles[4]=vrep.simxGetObjectHandle(clientID, 'UR3_joint5',vrep.simx_opmode_oneshot_wait)
errorCode,jointHandles[5]=vrep.simxGetObjectHandle(clientID, 'UR3_joint6',vrep.simx_opmode_oneshot_wait)

errorCode,pen_sensor=vrep.simxGetObjectHandle(clientID, 'feltPen_sensor',vrep.simx_opmode_oneshot_wait)

errorCode,floor=vrep.simxGetObjectHandle(clientID, 'ResizableFloor_5_25',vrep.simx_opmode_oneshot_wait)

print(pen_sensor)
    
errorCode,robot_joint_handle=vrep.simxGetObjectHandle(clientID, 'UR3_joint', vrep.simx_opmode_oneshot_wait)

    


def Get_MS():

	M = np.eye(4)
	S = np.zeros((6,6))

	S = np.array([[0, 0, 0, 0, 1, 0] ,\
	[0, 1, 1, 1, 0, 1] ,\
	[1, 0, 0, 0, 0, 0] ,\
	[0, -0.152, -0.152, -0.152, 0, -0.152] ,\
	[0, 0, 0, 0, 0.152, 0] ,\
	[0, 0, .244, .457, -0.110, 0.540]])
	M = np.array([[0, -1, 0, 0.540],\
	[0, 0, -1, 0.252],\
	[1, 0, 0, 0.205] ,\
	[0, 0, 0, 1]])


	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):
    PI = np.pi

	# Initialize the return_value 
    return_value = [None, None, None, None, None, None]

    theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
    T = np.eye(4)

    M, S = Get_MS()


    S1 = np.array([[0, -1, 0, 0],\
		[1, 0, 0, 0],\
		[0, 0, 0, 0],\
		[0, 0, 0, 0]])
    S2 = np.array([[0, 0, 1, -0.152],\
		[0, 0, 0, 0],\
		[-1, 0, 0, 0],\
		[0, 0, 0, 0]])
    S3 = np.array([[0, 0, 1, -0.152],\
		[0, 0, 0, 0],\
		[-1, 0, 0, 0.244],\
		[0, 0, 0, 0]])
    S4 = np.array([[0, 0, 1, -0.152],\
		[0, 0, 0, 0],\
		[-1, 0, 0, 0.457],\
		[0, 0, 0, 0]])
    S5 = np.array([[0, 0, 0, 0],\
		[0, 0, -1, 0.152],\
		[0, 1, 0, -0.110],\
		[0, 0, 0, 0]])
    S6 = np.array([[0, 0, 1, -0.152],\
		[0, 0, 0, 0],\
		[-1, 0, 0, 0.540],\
		[0, 0, 0, 0]])
    T = np.dot(np.dot(np.dot(np.dot(np.dot(np.dot(expm(S1*theta[0]),expm(S2*theta[1])), expm(S3*theta[2])), expm(S4*theta[3])), expm(S5*theta[4])), expm(S6*theta[5])), M)



	

    return_value[0] = theta1
    return_value[1] = theta2 + PI/2
    return_value[2] = theta3
    return_value[3] = theta4 - PI
    return_value[4] = theta5
    return_value[5] = theta6

    return return_value


    
    
def lab_invk(xWpen, yWpen, zWpen, yaw_WpenDegree):
    PI = np.pi
    
    # theta1 to theta6
    thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    l01 = 0.152
    l02 = 0.120
    l03 = 0.244
    l04 = 0.093
    l05 = 0.213
    l06 = 0.083
    l07 = 0.083
    l08 = 0.082    
    l09 = 0.001
    l10 = 0.112


    xgrip = xWpen 
    ygrip = yWpen  
    zgrip = zWpen 
    
    yaw_ang = (yaw_WpenDegree*PI)/180
    xcen = xgrip - (np.cos(yaw_ang)*l09)
    ycen = ygrip - (np.sin(yaw_ang)*l09)
    zcen = zgrip

	# Distance of rectangle to find theta 1
    d1 = np.sqrt(((xcen)**2)+((ycen)**2))

    alpha = np.arcsin((l02-l04+l06)/d1)
    beta = np.arctan2(ycen,xcen)
	# theta1
    thetas[0] = beta-alpha        # Default value Need to Change

	# theta6
    thetas[5] = thetas[0] + ((PI/2)-yaw_ang)     # Default value Need to Change

	# Find Square Dist 
    square_dist = np.sqrt((l06+.027)**2+(l07)**2)
	# Find the two required angles
    gamma = np.arctan2(l07,(l06+0.027))
    theta_x = (PI/2)-(((PI/2)-thetas[0])+gamma)


 
    x3end = xcen+(np.sin(theta_x)*square_dist)
    y3end = ycen-(np.cos(theta_x)*square_dist)
    z3end = zcen+l10+l08

    d = z3end-l01

    R = np.sqrt((x3end)**2+(y3end)**2+(d)**2)

    psi_1 = np.arcsin(d/R)

    psi_2 = np.arccos(((R)**2+(l03)**2-(l05)**2)/(2*l03*R))

    omega = np.arccos(((l03)**2+(l05)**2-(R)**2)/(2*l03*l05))


    thetas[1]= -(psi_1+psi_2)     # Default value Need to Change
    thetas[2]= PI-omega 
    sigma = PI-omega+thetas[1]    # Default value Need to Change
    thetas[3]= -((PI/2)+sigma) - PI/2 # Default value Need to Change
    thetas[4]=-PI/2      # Default value Need to Change

    print("theta1 to theta6: " + str(thetas) + "\n")

    return lab_fk(float(thetas[0]), float(thetas[1]), float(thetas[2]), float(thetas[3]), float(thetas[4]), float(thetas[5]) )
    
    
def draw_line(start_ptx,start_pty, end_ptx, end_pty, jointHandles):
    
    
    for i in range (1,101):

           a = end_ptx - start_ptx
           b = end_pty - start_pty
               
           if(a == 0):
               x = start_ptx
               y = start_pty + i*(0.01)*(b)             
     
           if(b == 0):
               x = start_ptx + i*(0.01)*(a)
               y = start_pty
               
           elif(a != 0 and b != 0):
               m = b/a
               x = start_ptx + i*(0.01)*(a)
               y = m*(x - start_ptx) + start_pty
           #Calculate Inverse Kinematics
           thetas = lab_invk(x, y, 0, 0)
           
           #Move robot arm
           vrep.simxPauseCommunication(clientID,True)
           vrep.simxSetJointTargetPosition(clientID,jointHandles[0],thetas[0],vrep.simx_opmode_oneshot)
           vrep.simxSetJointTargetPosition(clientID,jointHandles[1],thetas[1],vrep.simx_opmode_oneshot)
           vrep.simxSetJointTargetPosition(clientID,jointHandles[2],thetas[2],vrep.simx_opmode_oneshot)
           vrep.simxSetJointTargetPosition(clientID,jointHandles[3],thetas[3],vrep.simx_opmode_oneshot)
           vrep.simxSetJointTargetPosition(clientID,jointHandles[4],thetas[4],vrep.simx_opmode_oneshot)
           vrep.simxSetJointTargetPosition(clientID,jointHandles[5],thetas[5],vrep.simx_opmode_oneshot)
           vrep.simxPauseCommunication(clientID,False)

    return 
    
    
def draw_circ(start_ptx,start_pty, r, jointHandles):
    
    
    for i in range (0,721):
        PI=np.pi

        x = start_ptx + r*np.cos(i*PI/180)
        y = start_pty + r*np.sin(i*PI/180)
           
        #Calculate Inverse Kinematics   
        thetas = lab_invk(x, y, 0, 0)
        #Move robot arm   
        vrep.simxPauseCommunication(clientID,True)
        vrep.simxSetJointTargetPosition(clientID,jointHandles[0],thetas[0],vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID,jointHandles[1],thetas[1],vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID,jointHandles[2],thetas[2],vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID,jointHandles[3],thetas[3],vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID,jointHandles[4],thetas[4],vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID,jointHandles[5],thetas[5],vrep.simx_opmode_oneshot)
        vrep.simxPauseCommunication(clientID,False)
        
        #Compensate for starting location
        if(i >= 200):
            vrep.simxSetModelProperty(clientID, floor,0,vrep.simx_opmode_oneshot)
    
    return
    
def draw_arc(start_ptx,start_pty, r, jointHandles):
    
    
    for i in range (0,721):
        PI=np.pi
        
        
        x = start_ptx + r*np.cos(i*PI/180)
        y = start_pty + r*np.sin(i*PI/180)

               
           
        print(x)
        print(y)
           
        thetas = lab_invk(x, y, 0, 0)
           
        vrep.simxPauseCommunication(clientID,True)
        vrep.simxSetJointTargetPosition(clientID,jointHandles[0],thetas[0],vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID,jointHandles[1],thetas[1],vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID,jointHandles[2],thetas[2],vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID,jointHandles[3],thetas[3],vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID,jointHandles[4],thetas[4],vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID,jointHandles[5],thetas[5],vrep.simx_opmode_oneshot)
        vrep.simxPauseCommunication(clientID,False)
        
        if(i >= 330 and i <= 420):
            vrep.simxSetModelProperty(clientID, floor,0,vrep.simx_opmode_oneshot)
        else:
            vrep.simxSetModelProperty(clientID, floor, vrep.sim_modelproperty_not_detectable,vrep.simx_opmode_oneshot)
            

    
    
    return    
    
    
    
def main():
    
    input_done = 0
    
    
    while(input_done == 0):
        print('1:line')
        print('2:square')
        print('3:right triangle')
        print('4:circle')
        print('5:smiley face')
        print('-----------')
        input_string = raw_input("Enter shape:  ")
        print("You entered " + input_string + "\n")

        if(int(input_string) == 1):
            input_done = 1
            shape = 1
        elif (int(input_string) == 2):
            input_done = 1
            shape = 2
        elif (int(input_string) == 3):
            input_done = 1
            shape = 3
        elif (int(input_string) == 4):
            input_done = 1
            shape = 4
        elif (int(input_string) == 5):
            input_done = 1
            shape = 5
        elif (int(input_string) == 0):
            input_done = 1
            shape = 0
        else:
            print("Please just enter the character 1 2 3 4 or 5 \n\n")
    if shape == 0:
        #move to starting location    
        vrep.simxPauseCommunication(clientID,True)
        vrep.simxSetJointTargetPosition(clientID,jointHandles[0],0,vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID,jointHandles[1],0,vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID,jointHandles[2],0,vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID,jointHandles[3],0,vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID,jointHandles[4],0,vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID,jointHandles[5],0,vrep.simx_opmode_oneshot)
        vrep.simxPauseCommunication(clientID,False)
        return
    
    print("Enter starting point")
    start_ptx = float(input("Enter x position: "))
    start_pty = float(input("Enter y position: "))
        
    thetas = []
    thetas = lab_invk(float(start_ptx), float(start_pty), 0, 0)
        

    #setup joint speeds
    vrep.simxSetJointTargetVelocity(clientID,jointHandles[0],0.01,vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetVelocity(clientID,jointHandles[1],0.01,vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetVelocity(clientID,jointHandles[2],0.01,vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetVelocity(clientID,jointHandles[3],0.01,vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetVelocity(clientID,jointHandles[4],0.01,vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetVelocity(clientID,jointHandles[5],0.01,vrep.simx_opmode_oneshot)
        
    #disable drawing    
    vrep.simxSetModelProperty(clientID, floor, vrep.sim_modelproperty_not_detectable,vrep.simx_opmode_oneshot)
        
    #move to starting location    
    vrep.simxPauseCommunication(clientID,True)
    vrep.simxSetJointTargetPosition(clientID,jointHandles[0],thetas[0],vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID,jointHandles[1],thetas[1],vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID,jointHandles[2],thetas[2],vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID,jointHandles[3],thetas[3],vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID,jointHandles[4],thetas[4],vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID,jointHandles[5],thetas[5],vrep.simx_opmode_oneshot)
    vrep.simxPauseCommunication(clientID,False)
        
    
    
       
       
    
    #draw line    
    if shape == 1:
       
       print("Enter end point")
       end_ptx = float(input("Enter x coordinate: "))
       end_pty = float(input("Enter y coordinate: "))
       

       
       #enable drawing
       vrep.simxSetModelProperty(clientID, floor,0,vrep.simx_opmode_oneshot)
       
       
       draw_line(start_ptx,start_pty,end_ptx,end_pty,jointHandles)
    
    #draw square
    if shape == 2:

        size = float(input("Enter Side Length:"))
        
        #enable drawing
        vrep.simxSetModelProperty(clientID, floor,0,vrep.simx_opmode_oneshot)
        

        
        end_ptx = start_ptx + size
        end_pty = start_pty
        
        draw_line(start_ptx,start_pty,end_ptx,end_pty,jointHandles)
        
        start_ptx = end_ptx
        end_pty = start_pty + size
        
        draw_line(start_ptx,start_pty,end_ptx,end_pty,jointHandles)
        
        end_ptx = start_ptx - size
        start_pty = end_pty
        
        draw_line(start_ptx,start_pty,end_ptx,end_pty,jointHandles)
        
        start_ptx = end_ptx
        end_pty = start_pty - size
        
        draw_line(start_ptx,start_pty,end_ptx,end_pty,jointHandles)
    
    #draw right triangle
    if shape == 3:
        legA = float(input("Enter Leg A Length:"))
        legB = float(input("Enter Leg B Length:"))
        
        #enable drawing
        vrep.simxSetModelProperty(clientID, floor,0,vrep.simx_opmode_oneshot)
        
        end_a = start_ptx + legA
        end_pty = start_pty
        
        draw_line(start_ptx,start_pty,end_a,end_pty,jointHandles)
        
        end_ptx = start_ptx
        end_b = start_pty + legB
        
        
        draw_line(end_a,start_pty,start_ptx,end_b,jointHandles)
        
        
        draw_line(start_ptx,end_b,start_ptx,start_pty,jointHandles)
        
    #draw circle    
    if shape == 4:
        print('Enter radius')
        r = float(input("Enter radius:"))

        
        #enable drawing
        vrep.simxSetModelProperty(clientID, floor,0,vrep.simx_opmode_oneshot)
        
        draw_circ(start_ptx,start_pty,r,jointHandles) 
        
    #draw smiley face
    if shape == 5:
        print('Enter radius')
        r = float(input("Enter radius:"))
     
        #enable drawing
        vrep.simxSetModelProperty(clientID, floor,0,vrep.simx_opmode_oneshot)
        
        draw_circ(start_ptx,start_pty,r,jointHandles)
        
        eye1x = start_ptx - r/2
        eye1y = start_pty - r/3     
        
        #disable drawing    
        vrep.simxSetModelProperty(clientID, floor, vrep.sim_modelproperty_not_detectable,vrep.simx_opmode_oneshot)
        eye = r/4
        draw_circ(eye1x,eye1y,eye,jointHandles)
        
        eye2x = start_ptx - r/2
        eye2y = start_pty + r/3
        
        #disable drawing    
        vrep.simxSetModelProperty(clientID, floor, vrep.sim_modelproperty_not_detectable,vrep.simx_opmode_oneshot)
        draw_circ(eye2x,eye2y,eye,jointHandles)
        
        r = r/1.5
        draw_arc(start_ptx,start_pty,r,jointHandles)
    return

main()


exit

