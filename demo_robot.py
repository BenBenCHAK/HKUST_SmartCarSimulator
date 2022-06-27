import numpy as np

import pybullet as p
import pybullet_data

from PIL import Image
from time import sleep

class pyBulletView:
    def __init__(self):
        # Basic pyBullet config
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -10)
        
        # PyBullet view config
        self.switchCamera = p.addUserDebugParameter('God view / Car camera', 1, 0, 1)
        self.takePic = p.addUserDebugParameter('Take Picture', 1, 0, 1)
        self.angle = p.addUserDebugParameter('Steering', -0.5, 0.5, 0)
        self.throttle = p.addUserDebugParameter('Throttle', 0, 20, 0)
        self.takePicClicked = p.readUserDebugParameter(self.takePic)
        
        # PyBullet load stuff
        self.car = p.loadURDF('/src/simplecar.urdf', [0, 0, 0.1])
        self.plane = p.loadURDF('/src/simpleplane.urdf')
        self.baseId = p.loadURDF("/src/track.urdf", [0, 0, 0], useFixedBase=1, globalScaling=0.1)
        self.baseTextureId = p.loadTexture("/src/track_smaller.png")
        p.changeVisualShape(self.baseId, -1, textureUniqueId=self.baseTextureId)

        self.wheel_indices = [1, 3, 4, 5]
        self.hinge_indices = [0, 2]
        
        # number_of_joints = p.getNumJoints(self.car)
        # for joint_number in range(number_of_joints):
        #     info = p.getJointInfo(self.car, joint_number)
        #     print(info[0], ": ", info[1], "\n")

        # configure gui and camera
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0)

    def __del__(self):
        p.disconnect()

if __name__ == '__main__':
    pBV = pyBulletView()

    while True:
        carNumClicked = p.readUserDebugParameter(pBV.switchCamera)
        user_angle = p.readUserDebugParameter(pBV.angle)
        user_throttle = p.readUserDebugParameter(pBV.throttle)
        
        for joint_index in pBV.wheel_indices:
            p.setJointMotorControl2(pBV.car, joint_index, p.VELOCITY_CONTROL, targetVelocity=user_throttle)
        for joint_index in pBV.hinge_indices:
            p.setJointMotorControl2(pBV.car, joint_index, p.POSITION_CONTROL, targetPosition=user_angle)
        
        position, orientation = p.getBasePositionAndOrientation(pBV.car)

        if carNumClicked % 2 == 0:
            eulerOri = p.getEulerFromQuaternion(orientation)[2]
            p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=(np.degrees(eulerOri - np.pi / 2)), cameraPitch=-25, cameraTargetPosition=(position[0] + np.cos(eulerOri), position[1] + np.sin(eulerOri), position[2]))
            
            p.getCameraImage(128, 120)
        else:
            p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=50, cameraPitch=-25, cameraTargetPosition=(0, 0, 0))
        
        if pBV.takePicClicked != p.readUserDebugParameter(pBV.takePic):
            image = p.getCameraImage(128, 120)[2]
            grey = np.uint8(np.dot(image[...,:3], [0.2989, 0.5870, 0.1140]))
            im = Image.fromarray(grey, mode="L")
            print(grey)
            
            # im = Image.fromarray(image, mode="RGBA").convert("L")
            im.save("capture.png")
            
            pBV.takePicClicked = p.readUserDebugParameter(pBV.takePic)
        
        p.stepSimulation()
