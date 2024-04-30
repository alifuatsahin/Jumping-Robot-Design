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
    
    def __init__(self, x, urdf_angle = 45*np.pi/180,  friction = 1.3):
        self.link3 = x[0]
        self.link1 = x[1]
        self.stiffness = x[2]
        self.rest = x[3]
        self.compression = x[4]
        
        self.urdf_angle = urdf_angle
        self.friction = friction #friction
        
    def to_array():
        return [self.link3,self.link1,self.stiffness,self.rest,self.compression] # 5 variables

class model():
    
    jump = []
    max_high = 0
    max_lenght = 0
    energy = 0
    
    
    
    def __init__(self, parameters):
        self.parameters = parameters

    def simulate(self):
        print("start simulation")
        lenght = []
        self.jump = []
        cwd = os.getcwd()
        urdf_path = os.path.join(cwd, "../urdf/test.urdf")

        # p.connect(p.GUI)  # or p.DIRECT for non-graphical version
        p.connect(p.DIRECT)
        p.resetSimulation()
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)

        planeID = p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
        jumper = p.loadURDF(urdf_path, [0, 0, 0], [0, 0, 0, 1], useFixedBase=0)

        p.changeDynamics(planeID, -1, lateralFriction=self.parameters.friction)

        nJoints = p.getNumJoints(jumper)
        jointNameToId = {}
        for i in range(nJoints):
            jointInfo = p.getJointInfo(jumper, i)
            jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]

        for name in range(len(jointNameToId)):
            p.setJointMotorControl2(jumper,
                                    jointIndex=name,
                                    controlMode=p.VELOCITY_CONTROL,
                                    force=0,
                                    )

        j12 = jointNameToId["j12"]
        j25s = jointNameToId["j25s"]
        j5s5 = jointNameToId["j5s5"]
        j54 = jointNameToId["j54"]
        j43 = jointNameToId["j43"]

        p.changeDynamics(jumper, j12, angularDamping=0.3)

        compressed = self.parameters.urdf_angle - self.parameters.rest + self.parameters.compression
        p.resetJointState(jumper, j12, compressed)

        constraint = p.createConstraint(jumper,
                                         j43,
                                         jumper,
                                         -1,
                                         p.JOINT_POINT2POINT,
                                         [0, 0, 0],
                                         [0, 0, self.parameters.link3 / 2],
                                         [0, 0, -self.parameters.link1 / 2])
        log = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "record.mp4")
        for step in range(300):
            focus_position, _ = p.getBasePositionAndOrientation(jumper)
            p.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=75, cameraPitch=-20, cameraTargetPosition=focus_position)
            self.jump.append(focus_position[2])
            lenght.append(focus_position[1])
            motor_angle = (p.getJointState(jumper, j12)[0] + self.parameters.urdf_angle - self.parameters.rest) * 180 / np.pi
            #print(motor_angle)

            p.setJointMotorControl2(jumper,
                                    jointIndex=j12,
                                    controlMode=p.TORQUE_CONTROL,
                                    force=-motor_angle * self.parameters.stiffness,
                                    )

            p.stepSimulation()
            time.sleep(0.01)
        print("end simulation")
        self.max_high = max(self.jump)
        focus_position, _ = p.getBasePositionAndOrientation(jumper)
        self.max_lenght = np.abs(focus_position[1])
        
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