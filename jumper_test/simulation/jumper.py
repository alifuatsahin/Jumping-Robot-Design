import os
import numpy as np
import time
import pybullet as p

def simulate(l1, l2, l3, l4, l5, compression, rest_angle, stiffness):
  p.connect(p.GUI) #or p.DIRECT to simulate faster

  #Create Plane
  plane = p.createCollisionShape(p.GEOM_PLANE)
  p.createMultiBody(0, 0)
  p.changeDynamics(plane, -1, lateralFriction=0.8)

  #Link Lengths [m]
  l1 = l1/1000
  l2 = l2/1000
  l3 = l3/1000
  l4 = l4/1000
  l5 = l5/1000
  t = 4/1000
  rest_angle = rest_angle*np.pi/180
  compression = compression*np.pi/180

  #Material Parameters
  density = 1.2*1000 #kg/m3
  stiffness = stiffness #N.m/deg

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
    print(jointInfo)

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
  p.setRealTimeSimulation(1)

  time_step = 1/240
  counter = 0
  start = True
  switch = 0

  while counter < 3:
      focus, _ = p.getBasePositionAndOrientation(jumper)
      p.resetDebugVisualizerCamera(cameraDistance=0.5, 
                                  cameraYaw=75, 
                                  cameraPitch=-20, 
                                  cameraTargetPosition=focus)
      counter += time_step

      if counter < 0.5 and start:
        pass
      elif start:
        p.setRealTimeSimulation(0)
        for id in range(nJoints):
          p.setJointMotorControl2(jumper, 
                                  jointIndex=id,
                                  controlMode=p.VELOCITY_CONTROL,
                                  force=0.001)
        start = False
      else:
        motor_angle = (p.getJointState(jumper, 0)[0] + rest_angle)*180/np.pi
        
        p.setJointMotorControl2(jumper,
          jointIndex=0,
          controlMode=p.TORQUE_CONTROL,
          force=-motor_angle*stiffness,
          )
        
        contacts = p.getContactPoints(jumper, plane)
        print(contacts)

        for contact in contacts:
          if contact[1] == jumper and contact[2] == plane:
            if switch == 1:
              switch = 2
            break
        if not contacts and switch == 0:
          switch = 1
        if switch == 2:
          break

        p.stepSimulation()

      time.sleep(time_step)

simulate(l1=60, l2=100, l3=100, l4=60, l5=80, compression=40, rest_angle=60, stiffness=8/1000)