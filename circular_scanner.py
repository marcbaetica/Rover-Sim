import pybullet as p
import time
import numpy as np


p.connect(p.GUI)
p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0, 0)
# top
boxHalfLength = 0.05
boxHalfWidth = 0.05
boxHalfHeight = 0.5
body = p.createCollisionShape(p.GEOM_BOX, halfExtents=[boxHalfLength, boxHalfWidth, boxHalfHeight])
pin = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05])
wgt = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05])

mass = 10000
visualShapeId = -1
nlnk = 2
link_Masses = [0, 1]
linkCollisionShapeIndices = [pin, wgt]
linkVisualShapeIndices = [-1] * nlnk
linkPositions = [[0.0, 0.0, 0.0], [1.0, 0, 0]]
linkOrientations = [[0, 0, 0, 1]] * nlnk
linkInertialFramePositions = [[0, 0, 0]] * nlnk
linkInertialFrameOrientations = [[0, 0, 0, 1]] * nlnk
indices = [0, 1]
jointTypes = [p.JOINT_REVOLUTE, p.JOINT_REVOLUTE]
axis = [[0, 0, 1], [0, 1, 0]]
basePosition = [0, 0, 0.5]
baseOrientation = [0, 0, 0, 1]

block = p.createMultiBody(mass, body, visualShapeId, basePosition, baseOrientation,
                          linkMasses=link_Masses,
                          linkCollisionShapeIndices=linkCollisionShapeIndices,
                          linkVisualShapeIndices=linkVisualShapeIndices,
                          linkPositions=linkPositions,
                          linkOrientations=linkOrientations,
                          linkInertialFramePositions=linkInertialFramePositions,
                          linkInertialFrameOrientations=linkInertialFrameOrientations,
                          linkParentIndices=indices,
                          linkJointTypes=jointTypes,
                          linkJointAxis=axis)
p.setGravity(0, 0, -0.0)

p.setTimeStep(0.01)
p.setRealTimeSimulation(0)
# p.resetBaseVelocity(top,angularVelocity=[0,0,30])
p.changeDynamics(block, 0, lateralFriction=0, angularDamping=0.000, linearDamping=0.000, spinningFriction=0,
                 rollingFriction=0)
p.changeDynamics(block, 1, lateralFriction=0, angularDamping=0.000, linearDamping=0.000, spinningFriction=0,
                 rollingFriction=0)
# p.changeDynamics(block,-1,lateralFriction=10000,angularDamping=1.000,linearDamping=0.000,spinningFriction=10000)
p.resetDebugVisualizerCamera(cameraDistance=2, cameraYaw=10, cameraPitch=-20, cameraTargetPosition=[0.0, 0.0, 0.25])

t0 = time.time()
t = time.time()
while ((t - t0) < 3):
    t = time.time()

p.resetBasePositionAndOrientation(block, [0, 0, 0.5], [0, 0, 0, 1])
p.enableJointForceTorqueSensor(block, 0, enableSensor=1)
# p.setJointMotorControl2(block,0,p.POSITION_CONTROL,targetPosition=-1,force=1000,maxVelocity=1)
p.setJointMotorControl2(block, 0, p.VELOCITY_CONTROL, 0, 0, 0)
# p.setJointMotorControl2(block,0,controlMode=p.TORQUE_CONTROL,force=-20)
# dum2=p.getDynamicsInfo(block,0)

dum2 = p.getJointState(block, 0)

# init robot position
t0 = time.time()
t = time.time()
its = 0
while ((t - t0) < 5):
    its += 1
    # p.enableJointForceTorqueSensor(block,0,enableSensor=1)
    p.setJointMotorControl2(block, 0, controlMode=p.TORQUE_CONTROL, force=-1)
    dum2 = p.getJointState(block, 0)

    t = time.time()

    dum = p.getLinkState(block, 1)
    xft = dum[0]
    dum = p.rayTest(xft - np.array([0, 0, 0.051]), xft - np.array([0, 0, 1.051]))
    # print(t-t0,1*dum[0][2])

    p.stepSimulation()
    time.sleep(1. / 100.)
    # print(t-t0,80*dum[0][2])
p.disconnect()
