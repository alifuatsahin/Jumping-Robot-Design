import numpy as np
import pybullet as p
import pybullet_data
import time
import os

import matplotlib.pyplot as plt

class Parameters:
    #def __init__(self, link3, link1, stiffness, rest, compression, urdf_angle = 45*np.pi/180,  friction = 1.3):
    #    self.link3 = link3 #link
    #    self.link1 = link1 #link 
    #    self.stiffness = stiffness #spring stiffness
    #    self.rest = rest #rest angle
    #    self.compression = compression #compression angle
        
    #    self.urdf_angle = urdf_angle
    #    self.friction = friction #friction
    
    def __init__(self, x,  friction = 1.3,density = 1.2, thickness = 4):
        self.l1 = x[0]
        self.l2 = x[1]
        self.l3 = x[2]
        self.l4 = x[3]
        self.l5 = x[4]
        self.compression = x[5]
        self.rest = x[6]
        self.stiffness = x[7]
        
        self.density = density
        self.friction = friction #friction
        self.t = thickness
        
    def to_array():
        return[self.l1,self.l2,self.l3,self.l4,self.l5,self.compression,self.rest,self.stiffness]
   

class model():
    jump = []
    lenght = []
    max_high = 0
    max_lenght = 0
    energy = 0
    
    def __init__(self, parameters):
        self.parameters = parameters

    def simulate(self,visualisation = False):
        print("start simulation")
        self.jump = []
        self.lenght = []
        cwd = os.getcwd()
       
        # p.connect(p.GUI)  # or p.DIRECT for non-graphical version
        if visualisation:
            p.connect(p.GUI)
        else: 
            p.connect(p.DIRECT)
        
        #Create Plane
        plane = p.createCollisionShape(p.GEOM_PLANE)
        p.createMultiBody(0, 0)
        p.changeDynamics(plane, -1, lateralFriction=0.8)

        #Link Lengths [m]
        l1 = self.parameters.l1/1000
        l2 = self.parameters.l2/1000
        l3 = self.parameters.l3/1000
        l4 = self.parameters.l4/1000
        l5 = self.parameters.l5/1000

        t = self.parameters.t/1000
        rest_angle = self.parameters.rest*np.pi/180
        compression = self.parameters.compression*np.pi/180

        #Material Parameters
        density = self.parameters.density*1000 #kg/m3
        stiffness = self.parameters.stiffness/1000 #N.m/deg

        #Arrays
        link_Masses = []
        linkCollisionShapeIndices = []
        linkVisualShapeIndices = []
        linkPositions = []
        linkOrientations = []
        linkInertialFramePositions = []
        linkInertialFrameOrientations = []
        linkParentIndices = []
        jointTypes = []
        axis = []

        #CREATE COLLISION SHAPE
        #Base Link (link1)
        base_v = p.createVisualShape(shapeType=p.GEOM_CAPSULE,
                              radius=t,
                              length=l1)

        base_c = p.createCollisionShape(shapeType=p.GEOM_CAPSULE,
                              radius=t,
                              height=l1)

        deg = np.rad2deg(rest_angle-compression)
        basePos = [0, 0, l2] #[x,y,z]
        base_orientation = [np.sqrt(2)/2, 0, 0, np.sqrt(2)/2] #quaternion [x,y,z,w]
        base_i_pos = [0, 0, 0]
        base_i_orientation = [0, 0, 0, 1]
        base_color = [0, 1, 0, 0] #[R, G, B, alpha]
        baseMass = density*np.pi*pow(t,2)*l1

        #Link2
        l2_v = p.createVisualShape(shapeType=p.GEOM_CAPSULE,
                              radius=t,
                              length=l2,
                              rgbaColor = [1, 0, 0, 1],
                              visualFramePosition=[0,0,-l2/2])

        l2_c = p.createCollisionShape(shapeType=p.GEOM_CAPSULE,
                              radius=t,
                              height=l2,
                              collisionFramePosition=[0,0,-l2/2])

        l2_pos = [0, 0, -l1/2] #[x,y,z]
        l2_orientation = [0, 0, 0, 1] #quaternion [x,y,z,w]
        l2_i_pos = [0, 0, 0]
        l2_i_orientation = [0, 0, 0, 1]
        l2_parent = 0
        l2_mass = density*np.pi*pow(t,2)*l2
        l2_joint = p.JOINT_REVOLUTE
        l2_joint_axis = [1, 0, 0]

        linkCollisionShapeIndices.append(l2_c)
        linkVisualShapeIndices.append(l2_v)
        linkPositions.append(l2_pos)
        linkOrientations.append(l2_orientation)
        linkInertialFramePositions.append(l2_i_pos)
        linkInertialFrameOrientations.append(l2_i_orientation)
        linkParentIndices.append(l2_parent)
        link_Masses.append(l2_mass)
        jointTypes.append(l2_joint)
        axis.append(l2_joint_axis)

      #Link3
        l3_v = p.createVisualShape(shapeType=p.GEOM_CAPSULE,
                              radius=t,
                              length=l3,
                              rgbaColor = [0, 1, 0, 1],
                              visualFramePosition=[0,0,l3/2])

        l3_c = p.createCollisionShape(shapeType=p.GEOM_CAPSULE,
                              radius=t,
                              height=l3,
                              collisionFramePosition=[0,0,l3/2])

        l3_pos = [0, 0, l1/2] #[x,y,z]
        l3_orientation = [0, 0, 0, 1] #quaternion [x,y,z,w]
        l3_i_pos = [0, 0, 0]
        l3_i_orientation = [0, 0, 0, 1]
        l3_parent = 0
        l3_mass = density*np.pi*pow(t,2)*l3
        l3_joint = p.JOINT_REVOLUTE
        l3_joint_axis = [1, 0, 0]

        linkCollisionShapeIndices.append(l3_c)
        linkVisualShapeIndices.append(l3_v)
        linkPositions.append(l3_pos)
        linkOrientations.append(l3_orientation)
        linkInertialFramePositions.append(l3_i_pos)
        linkInertialFrameOrientations.append(l3_i_orientation)
        linkParentIndices.append(l3_parent)
        link_Masses.append(l3_mass)
        jointTypes.append(l3_joint)
        axis.append(l3_joint_axis)

        #Link4
        l4_v = p.createVisualShape(shapeType=p.GEOM_CAPSULE,
                              radius=t,
                              length=l4,
                              rgbaColor = [0, 0, 1, 1],
                              visualFramePosition=[0,0,l4/2])

        l4_c = p.createCollisionShape(shapeType=p.GEOM_CAPSULE,
                              radius=t,
                              height=l4,
                              collisionFramePosition=[0,0,l4/2])

        l4_pos = [0, 0, -l2] #[x,y,z]
        l4_orientation = [0, 0, 0, 1] #quaternion [x,y,z,w]
        l4_i_pos = [0, 0, 0]
        l4_i_orientation = [0, 0, 0, 1]
        l4_parent = 1
        l4_mass = density*np.pi*pow(t,2)*l4
        l4_joint = p.JOINT_REVOLUTE
        l4_joint_axis = [1, 0, 0]

        linkCollisionShapeIndices.append(l4_c)
        linkVisualShapeIndices.append(l4_v)
        linkPositions.append(l4_pos)
        linkOrientations.append(l4_orientation)
        linkInertialFramePositions.append(l4_i_pos)
        linkInertialFrameOrientations.append(l4_i_orientation)
        linkParentIndices.append(l4_parent)
        link_Masses.append(l4_mass)
        jointTypes.append(l4_joint)
        axis.append(l4_joint_axis)

      #Link5_1
        l51_v = p.createVisualShape(shapeType=p.GEOM_CAPSULE,
                              radius=t,
                              length=l5,
                              rgbaColor = [1, 0, 0, 1],
                              visualFramePosition=[0,0,l5/2])

        l51_c = p.createCollisionShape(shapeType=p.GEOM_CAPSULE,
                              radius=t,
                              height=l5,
                              collisionFramePosition=[0,0,l5/2])

        l51_pos = [0, 0, l4] #[x,y,z]
        l51_orientation = [0, np.sin(np.pi/6), 0, np.cos(np.pi/6)] #quaternion [x,y,z,w]
        l51_i_pos = [0, 0, 0]
        l51_i_orientation = [0, 0, 0, 1]
        l51_parent = 3
        l51_mass = density*np.pi*pow(t,2)*l5
        l51_joint = p.JOINT_FIXED
        l51_joint_axis = [1, 0, 0]

        linkCollisionShapeIndices.append(l51_c)
        linkVisualShapeIndices.append(l51_v)
        linkPositions.append(l51_pos)
        linkOrientations.append(l51_orientation)
        linkInertialFramePositions.append(l51_i_pos)
        linkInertialFrameOrientations.append(l51_i_orientation)
        linkParentIndices.append(l51_parent)
        link_Masses.append(l51_mass)
        jointTypes.append(l51_joint)
        axis.append(l51_joint_axis)

      #Link5_2
        l52_v = p.createVisualShape(shapeType=p.GEOM_CAPSULE,
                              radius=t,
                              length=l5,
                              rgbaColor = [1, 0, 0, 1],
                              visualFramePosition=[0,0,l5/2])

        l52_c = p.createCollisionShape(shapeType=p.GEOM_CAPSULE,
                              radius=t,
                              height=l5,
                              collisionFramePosition=[0,0,l5/2])

        l52_pos = [0, 0, l4] #[x,y,z]
        l52_orientation = [0, -np.sin(np.pi/12), 0, np.cos(np.pi/12)] #quaternion [x,y,z,w]
        l52_i_pos = [0, 0, 0]
        l52_i_orientation = [0, 0, 0, 1]
        l52_parent = 3
        l52_mass = density*np.pi*pow(t,2)*l5
        l52_joint = p.JOINT_FIXED
        l52_joint_axis = [1, 0, 0]

        linkCollisionShapeIndices.append(l52_c)
        linkVisualShapeIndices.append(l52_v)
        linkPositions.append(l52_pos)
        linkOrientations.append(l52_orientation)
        linkInertialFramePositions.append(l52_i_pos)
        linkInertialFrameOrientations.append(l52_i_orientation)
        linkParentIndices.append(l52_parent)
        link_Masses.append(l52_mass)
        jointTypes.append(l52_joint)
        axis.append(l52_joint_axis)

        jumper = p.createMultiBody(baseMass=baseMass,
                        baseCollisionShapeIndex=base_c,
                        baseVisualShapeIndex=base_v,
                        basePosition = basePos,
                        baseOrientation = base_orientation,
                        baseInertialFramePosition = base_i_pos,
                        baseInertialFrameOrientation = base_i_orientation,
                        linkMasses = link_Masses,
                        linkCollisionShapeIndices = linkCollisionShapeIndices,
                        linkVisualShapeIndices = linkVisualShapeIndices,
                        linkPositions = linkPositions,
                        linkOrientations = linkOrientations,
                        linkInertialFramePositions = linkInertialFramePositions,
                        linkInertialFrameOrientations = linkInertialFrameOrientations,
                        linkParentIndices = linkParentIndices,
                        linkJointTypes = jointTypes,
                        linkJointAxis = axis)

        nJoints = p.getNumJoints(jumper)
        jointNameToId = {}
        jointIndices = np.zeros(nJoints)
        for i in range(nJoints):
            jointInfo = p.getJointInfo(jumper, i)
            jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
            jointIndices[i] = jointInfo[0]
            #print(jointInfo)

        p.resetJointState(jumper, 0, -(rest_angle-compression))
        p.resetJointState(jumper, nJoints-1, np.pi-(rest_angle-compression))
        p.resetJointState(jumper, 1, (rest_angle-compression))

        constraint = p.createConstraint(jumper, 
                                      1, 
                                      jumper, 
                                      nJoints-1,
                                      p.JOINT_POINT2POINT,
                                      [0, 0, 0], 
                                      [0, 0, l4], 
                                      [0, 0, l3])

        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)

        for id in range(nJoints):
            p.setJointMotorControl2(jumper, 
                                jointIndex=id,
                                controlMode=p.VELOCITY_CONTROL,
                                force=0.001)
    

        step = 0.01
        counter = 0

        for step in range(500):
            focus, _ = p.getBasePositionAndOrientation(jumper)
            
            self.jump.append(focus[2])
            self.lenght.append(focus[1])
            p.resetDebugVisualizerCamera(cameraDistance=0.5, 
                                      cameraYaw=75, 
                                      cameraPitch=-20, 
                                      cameraTargetPosition=focus)
            
            counter += step 
            motor_angle = (p.getJointState(jumper, 0)[0] + rest_angle)*180/np.pi
            
            p.setJointMotorControl2(jumper,
              jointIndex=0,
              controlMode=p.TORQUE_CONTROL,
              force=-motor_angle*stiffness,
              )

            p.stepSimulation()
            time.sleep(0.01)
            
            
        self.max_high = max(self.jump)
        self.max_lenght = max(self.lenght)
        
        print("max high",np.round(self.max_high,2),"[unit] max distance",np.round(self.max_lenght,2),"[unit]")
        return self.max_lenght
    
    def plot(self):
        if (self.jump == []):
            self.simulate()
        # Plot the curve
        plt.plot(np.arange(len(self.jump)), self.jump)
        plt.xlabel('Index')
        plt.ylabel('Value')
        plt.title('Random Curve')
        plt.grid(True)
        plt.show()
        
        
        
#plt.plot(np.arange(len(length)), length)
#plt.xlabel('Index')
#plt.ylabel('Value')
#plt.title('Random Curve')
#plt.grid(True)
#plt.show()