import numpy as np
import pybullet as p
import pybullet_data
import time
import os

import matplotlib.pyplot as plt

class Parameters:
    
    def __init__(self, x,  friction = 1.3,density = 1.2, thickness = 4):
        self.l2 = x[0]
        self.l3_c = x[1]
        self.l4_c = x[2]
        self.l5_c = x[3]
        self.compression_ratio = x[4]
        self.rest_angle = x[5]
        self.stiffness = x[6]
        self.link_angle = x[7]
        
        self.density = density
        self.friction = friction #friction
        self.t = thickness
        
    def to_array(self):
        return[self.l2,self.l3_c,self.l4_c,self.l5_c,self.compression_ratio,self.rest_angle,self.stiffness,self.link_angle]
   

class model():
    jump = []
    length = []
    max_high = 0
    max_length = 0
    energy = 0
    
    def __init__(self, parameters):
        #print("init")
        self.parameters = parameters
        self.jump = []
        self.length = []
        self.max_high = 0
        self.max_length = 0
        self.energy = 0 

    def simulate(self, visualisation = False):
        count = 0
        compress_count = 0
        contact_count = 0
        time_limit = 10
        start = True
        compressed = False
        EPS = 2*np.pi/180
        
        if visualisation:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT)

        plane = p.createCollisionShape(p.GEOM_PLANE)
        p.createMultiBody(0, plane)
        p.changeDynamics(plane, -1, lateralFriction=1.5)

        # Parameters
        link_angle = self.parameters.link_angle*np.pi/180
        rest_angle = self.parameters.rest_angle*np.pi/180

        l2 = self.parameters.l2
        l3 = self.parameters.l3_c*self.parameters.l2
        l4 = self.parameters.l4_c*self.parameters.l2
        l5 = self.parameters.l5_c*self.parameters.l2

        t = max(l2,l3,l5)/25 #self.parameters.t

        if l3 != l2:
            beta = np.arcsin((l3-l2)*np.sin(rest_angle)/l4)
            l1 = np.sin(rest_angle - beta)*l4/np.sin(rest_angle)
        else:
            beta = 0
            l1 = l4

        compression = self.parameters.compression_ratio*(rest_angle + beta)

        l1 = l1/1000
        l2 = l2/1000
        l3 = l3/1000
        l4 = l4/1000
        l5 = l5/1000
        t = t/1000

        # Material Properties
        density = 1.2*1000 #kg/m3
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

        # Base (link 1)
        base_v = p.createVisualShape(shapeType=p.GEOM_CAPSULE,
                                radius=t,
                                length=l1)

        base_c = p.createCollisionShape(shapeType=p.GEOM_CAPSULE,
                                radius=t,
                                height=l1)

        base_pos = [0,0,max(l2, l3)]
        base_ori = [np.sin((np.pi/2 + beta)/2),0,0,np.cos((np.pi/2 + beta)/2)]
        base_mass = 0.07 #density*l1*np.pi*pow(t,2)
        base_inertial_pos = [0,0,0]
        base_inertial_ori = [0,0,0,1]

        # Link 2
        l2_v = p.createVisualShape(shapeType=p.GEOM_CAPSULE,
                                radius=t,
                                length=l2,
                                rgbaColor=[1,0,0,1],
                                visualFramePosition=[0,0,-l2/2])

        l2_c = p.createCollisionShape(shapeType=p.GEOM_CAPSULE,
                                radius=t,
                                height=l2,
                                collisionFramePosition=[0,0,-l2/2])


        l2_pos = [0,0,l1/2]
        l2_ori = [-np.sin((rest_angle)/2),0,0,np.cos((rest_angle)/2)]
        l2_mass = density*l2*np.pi*pow(t,2)
        l2_inertial_pos = [0,0,-l2/2]
        l2_inertial_ori = [0,0,0,1]
        l2_joint = p.JOINT_REVOLUTE
        l2_joint_axis = [1, 0, 0]
        l2_parent = 0

        linkCollisionShapeIndices.append(l2_c)
        linkVisualShapeIndices.append(l2_v)
        linkPositions.append(l2_pos)
        linkOrientations.append(l2_ori)
        linkInertialFramePositions.append(l2_inertial_pos)
        linkInertialFrameOrientations.append(l2_inertial_ori)
        linkParentIndices.append(l2_parent)
        link_Masses.append(l2_mass)
        jointTypes.append(l2_joint)
        axis.append(l2_joint_axis)

        # Link 3

        l3_v = p.createVisualShape(shapeType=p.GEOM_CAPSULE,
                                radius=t,
                                length=l3,
                                rgbaColor=[0,1,0,1],
                                visualFramePosition=[0,0,-l3/2])

        l3_c = p.createCollisionShape(shapeType=p.GEOM_CAPSULE,
                                radius=t,
                                height=l3,
                                collisionFramePosition=[0,0,-l3/2])

        l3_pos = [0,0,-l1/2]
        l3_ori = [-np.sin((rest_angle)/2),0,0,np.cos((rest_angle)/2)]
        l3_mass = density*l3*np.pi*pow(t,2)
        l3_inertial_pos = [0,0,-l3/2]
        l3_inertial_ori = [0,0,0,1]
        l3_joint = p.JOINT_REVOLUTE
        l3_joint_axis = [1, 0, 0]
        l3_parent = 0

        linkCollisionShapeIndices.append(l3_c)
        linkVisualShapeIndices.append(l3_v)
        linkPositions.append(l3_pos)
        linkOrientations.append(l3_ori)
        linkInertialFramePositions.append(l3_inertial_pos)
        linkInertialFrameOrientations.append(l3_inertial_ori)
        linkParentIndices.append(l3_parent)
        link_Masses.append(l3_mass)
        jointTypes.append(l3_joint)
        axis.append(l3_joint_axis)

        # Link 4

        l4_v = p.createVisualShape(shapeType=p.GEOM_CAPSULE,
                                radius=t,
                                length=l4,
                                rgbaColor=[0,0,1,1],
                                visualFramePosition=[0,0,-l4/2])

        l4_c = p.createCollisionShape(shapeType=p.GEOM_CAPSULE,
                                radius=t,
                                height=l4,
                                collisionFramePosition=[0,0,-l4/2])

        l4_pos = [0,0,-l2]
        l4_ori = [np.sin((rest_angle - beta)/2),0,0,np.cos((rest_angle - beta)/2)]
        l4_mass = density*l4*np.pi*pow(t,2)
        l4_inertial_pos = [0,0,-l4/2]
        l4_inertial_ori = [0,0,0,1]
        l4_joint = p.JOINT_REVOLUTE
        l4_joint_axis = [1, 0, 0]
        l4_parent = 1

        linkCollisionShapeIndices.append(l4_c)
        linkVisualShapeIndices.append(l4_v)
        linkPositions.append(l4_pos)
        linkOrientations.append(l4_ori)
        linkInertialFramePositions.append(l4_inertial_pos)
        linkInertialFrameOrientations.append(l4_inertial_ori)
        linkParentIndices.append(l4_parent)
        link_Masses.append(l4_mass)
        jointTypes.append(l4_joint)
        axis.append(l4_joint_axis)

        # Link 5 - 1

        l5_v = p.createVisualShape(shapeType=p.GEOM_CAPSULE,
                                radius=t,
                                length=l5,
                                rgbaColor=[1,0,1,1],
                                visualFramePosition=[0,0,-l5/2])

        l5_c = p.createCollisionShape(shapeType=p.GEOM_CAPSULE,
                                radius=t,
                                height=l5,
                                collisionFramePosition=[0,0,-l5/2])

        l5_pos = [0,0,0]
        l5_ori = [0,np.sin((np.pi - link_angle)/2),0,np.cos((np.pi - link_angle)/2)]
        l5_mass = density*l5*np.pi*pow(t,2)
        l5_inertial_pos = [0,0,-l5/2]
        l5_inertial_ori = [0,0,0,1]
        l5_joint = p.JOINT_FIXED
        l5_joint_axis = [1, 0, 0]
        l5_parent = 3

        linkCollisionShapeIndices.append(l5_c)
        linkVisualShapeIndices.append(l5_v)
        linkPositions.append(l5_pos)
        linkOrientations.append(l5_ori)
        linkInertialFramePositions.append(l5_inertial_pos)
        linkInertialFrameOrientations.append(l5_inertial_ori)
        linkParentIndices.append(l5_parent)
        link_Masses.append(l5_mass)
        jointTypes.append(l5_joint)
        axis.append(l5_joint_axis)

        # Link 5 - 2

        l5_2_v = p.createVisualShape(shapeType=p.GEOM_CAPSULE,
                                radius=t,
                                length=l5,
                                rgbaColor=[1,0,1,1],
                                visualFramePosition=[0,0,-l5/2])

        l5_2_c = p.createCollisionShape(shapeType=p.GEOM_CAPSULE,
                                radius=t,
                                height=l5,
                                collisionFramePosition=[0,0,-l5/2])

        l5_2_pos = [0,0,0]
        l5_2_ori = [0,np.sin((np.pi + link_angle)/2),0,np.cos((np.pi + link_angle)/2)]
        l5_2_mass = density*l5*np.pi*pow(t,2)
        l5_2_inertial_pos = [0,0,-l5/2]
        l5_2_inertial_ori = [0,0,0,1]
        l5_2_joint = p.JOINT_FIXED
        l5_2_joint_axis = [1, 0, 0]
        l5_2_parent = 3

        linkCollisionShapeIndices.append(l5_2_c)
        linkVisualShapeIndices.append(l5_2_v)
        linkPositions.append(l5_2_pos)
        linkOrientations.append(l5_2_ori)
        linkInertialFramePositions.append(l5_2_inertial_pos)
        linkInertialFrameOrientations.append(l5_2_inertial_ori)
        linkParentIndices.append(l5_2_parent)
        link_Masses.append(l5_2_mass)
        jointTypes.append(l5_2_joint)
        axis.append(l5_2_joint_axis)

        jumper = p.createMultiBody(baseMass=base_mass,
                        baseCollisionShapeIndex=base_c,
                        baseVisualShapeIndex=base_v,
                        basePosition = base_pos,
                        baseOrientation = base_ori,
                        baseInertialFramePosition = base_inertial_pos,
                        baseInertialFrameOrientation = base_inertial_ori,
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

        p.resetJointState(jumper, 0, compression)
        p.resetJointState(jumper, 1, -compression)
        p.resetJointState(jumper, 4, compression)

        constraint = p.createConstraint(jumper, 
                                        4, 
                                        jumper, 
                                        1,
                                        p.JOINT_POINT2POINT,
                                        [1, 0, 0], 
                                        [0, 0, -l3/2], 
                                        [0, 0, -l4/2])


        for id in range(nJoints):
            p.changeDynamics(jumper, id,
                            lateralFriction=1.0,
                            spinningFriction=0.15,
                            rollingFriction=0.01,
                            restitution=0.9,
                            jointDamping=0.001)

        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)

        while count < time_limit:
            focus, _ = p.getBasePositionAndOrientation(jumper)
            p.resetDebugVisualizerCamera(cameraDistance=0.8, 
                                        cameraYaw=90, 
                                        cameraPitch=-20, 
                                        cameraTargetPosition=focus)
            
            if count < 0.5:
                pass
            elif start:
                p.setJointMotorControl2(jumper, 0, p.POSITION_CONTROL, targetPosition=compression)
                motor_angle = (p.getJointState(jumper, 0)[0])

                if motor_angle > compression - EPS and motor_angle < compression + EPS:
                    compressed = True
                if compressed:
                    compress_count += 1/240

                if compress_count > 0.5:
                    for id in range(nJoints):
                        p.setJointMotorControl2(jumper, 
                                                jointIndex=id,
                                                controlMode=p.VELOCITY_CONTROL,
                                                force=0.001)
                    initial_pos_arr, _ = p.getBasePositionAndOrientation(jumper)
                    start = False

            else:
                motor_angle = (p.getJointState(jumper, 0)[0])*180/np.pi

                self.jump.append(focus[2])
                self.length.append(focus[1])
            
                p.setJointMotorControl2(jumper,
                jointIndex=0,
                controlMode=p.TORQUE_CONTROL,
                force=-motor_angle*stiffness,
                )

                if contact_count > 0.5:
                    contacts = p.getContactPoints(jumper, plane)

                    if contacts:
                        final_pos_arr, _ = p.getBasePositionAndOrientation(jumper)
                        self.max_length = final_pos_arr[1] - initial_pos_arr[1]
                        self.max_high = max(self.jump)
                        break
                else:
                    contact_count += 1/240

            p.stepSimulation()
            if visualisation:
                time.sleep(1./240.)
            count += 1/240


        self.energy = 0.5*stiffness*pow(compression, 2)*180/np.pi

        p.resetSimulation()
        p.disconnect()

        print("max high",np.round(self.max_high,3),"[m] max distance",np.round(self.max_length,3),"[m] energy ",np.round(self.energy,4),"J")

        return self.max_length, self.energy    

        
    
    def plot(self):
        plt.figure()
        if (self.jump == []):
            self.simulate()
        # Plot the curve
        plt.plot(self.length, self.jump)
        plt.xlabel('length')
        plt.ylabel('high')
        plt.title('jump')
        plt.grid(True)
        plt.show()
        
        plt.figure()
        if (self.jump == []):
            self.simulate()
        # Plot the curve
        plt.plot(np.arange(len(self.jump)), self.jump)
        plt.xlabel('time')
        plt.ylabel('high')
        plt.title('jump ')
        plt.grid(True)
        plt.show()
        
        plt.figure()
        # Plot the curve
        plt.plot(np.arange(len(self.length)), self.length)
        plt.xlabel('distance')
        plt.ylabel('Value')
        plt.title('Jump')
        plt.grid(True)
        plt.show()
        
        