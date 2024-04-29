import numpy as np
import pybullet as p
import pybullet_data
import time
import os

class Parameters:
    def __init__(self, link3, link1, urdf_angle, stiffness, rest, friction, compression):
        self.link3 = link3
        self.link1 = link1
        self.urdf_angle = urdf_angle
        self.stiffness = stiffness
        self.rest = rest
        self.friction = friction
        self.compression = compression

class model():
    
    def __init__(self, parameters):
        self.parameters = parameters

    def simulate(self):
        cwd = os.getcwd()
        urdf_path = os.path.join(cwd, "../urdf/test.urdf")

        p.connect(p.GUI)  # or p.DIRECT for non-graphical version
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

            motor_angle = (p.getJointState(jumper, j12)[0] + self.parameters.urdf_angle - self.parameters.rest) * 180 / np.pi
            print(motor_angle)

            p.setJointMotorControl2(jumper,
                                    jointIndex=j12,
                                    controlMode=p.TORQUE_CONTROL,
                                    force=-motor_angle * self.parameters.stiffness,
                                    )

            p.stepSimulation()
            time.sleep(0.01)
        
        p.stopStateLogging(log)