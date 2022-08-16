import numpy as np

import pybullet as p

from PIL import Image
from time import sleep

import socket
import select

IMG_WIDTH = 128
IMG_HEIGHT = 120

DEFAULT_FRAME_RATE = 1./240.

A_G = -9.81

MAX_THROTTLE = 50 #20
MAX_STEERING = 1 #0.5

class pyBulletView:
    def switchMode(self):
        p.removeAllUserParameters()

        self.viewMode = 1 - self.viewMode

        if self.viewMode == 0:
            self.btnSwitchCamera = p.addUserDebugParameter('Switch to Client Mode', 1, 0, 0)
            
            self.btnRemoveMark = p.addUserDebugParameter('Remove current trajectory', 1, 0, 0)

            self.sldCarX = p.addUserDebugParameter('Car x-coordinate', -5, 5, 0)
            self.sldCarY = p.addUserDebugParameter('Car y-coordinate', -5, 5, 0)
            self.sldCarYaw = p.addUserDebugParameter('Car direction', -np.pi, np.pi, 0)

            self.sldSteering = p.addUserDebugParameter('Wheel direction', -MAX_STEERING, MAX_STEERING, 0)
            self.sldThrottle = p.addUserDebugParameter('Motor speed', 0, MAX_THROTTLE, 0)       
        elif self.viewMode == 1:
            self.btnSwitchCamera = p.addUserDebugParameter('Switch to God Mode', 1, 0, 1)

            self.sldCameraHeight = p.addUserDebugParameter('Car camera height', 0.1, 0.25, 0.25)
            self.sldCameraOffset = p.addUserDebugParameter('Car camera offset', 0.8, 1.2, 0.85)
            self.sldCameraAngle = p.addUserDebugParameter('Car camera angle', -1, 1, 0.5)

            self.btnEnableMarking = p.addUserDebugParameter('Show / Hide trajectory', 1, 0, 0)

            self.btnTakePic = p.addUserDebugParameter('Take and save camera picture', 1, 0, 0)

        self.btnStartSimulation = p.addUserDebugParameter('Enable / Disable simulation', 1, 0, 0)

    def __init__(self):
        # Basic pyBullet config
        p.connect(p.GUI)
        # p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, A_G)

        # View Mode: 0 = God mode; 1 = Client mode; Below will switch back to God mode
        self.viewMode = 1
        self.switchMode()

        self.__markFrom = [[0]*3]*4

        self.lastTakePicClicked = 0
        self.lastRemoveMarkClicked = 0

        # User input device data
        self.__isKeyLeftPressed = False
        self.__isKeyUpPressed = False
        self.__isKeyRightPressed = False
        self.__isKeyDownPressed = False

        # Camera data
        self.cameraDistance = 3
        self.cameraYaw = 50
        self.cameraPitch = -25
        self.cameraTargetPosition = (0, 0, 0)

        self.carImage = p.getCameraImage(IMG_WIDTH, IMG_HEIGHT)[2]
        
        # PyBullet load materials
        self.car = p.loadURDF('/src/simplecar.urdf', [0, 0, 0.1], globalScaling=0.5)
        self.plane = p.loadURDF('/src/simpleplane.urdf')
        self.trackId = p.createVisualShape(p.GEOM_MESH, fileName="/src/track.obj")
        p.createMultiBody(0, baseVisualShapeIndex=self.trackId, basePosition=[-4, -4, -0.08])

        self.wheel_indices = [1, 3, 4, 5]
        self.hinge_indices = [0, 2]
        self.camera_indices = [6, 7, 8]
        self.motor_wheels = [4, 5]
        # for joint_number in range(p.getNumJoints(self.car)):
        #     info = p.getJointInfo(self.car, joint_number)
        #     print(info[0], ": ", info[1], "\n")

        # configure gui and camera
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0)

    def __del__(self):
        p.disconnect()

    def controlInputAndParam(self, counter):
        self.__counter = counter

        if self.viewMode != p.readUserDebugParameter(self.btnSwitchCamera):
            self.switchMode()

        if self.viewMode == 0:
            self.carX = p.readUserDebugParameter(self.sldCarX)
            self.carY = p.readUserDebugParameter(self.sldCarY)
            self.carYaw = p.readUserDebugParameter(self.sldCarYaw)
            self.carThr = p.readUserDebugParameter(self.sldThrottle)
            self.carSte = p.readUserDebugParameter(self.sldSteering)
        elif self.viewMode == 1:
            self.trajectoryOn = p.readUserDebugParameter(self.btnEnableMarking) % 2
            self.cameraAngle = p.readUserDebugParameter(self.sldCameraAngle)
            self.cameraHeight = p.readUserDebugParameter(self.sldCameraHeight)
            self.cameraDistance = p.readUserDebugParameter(self.sldCameraOffset)
        
        # Keyboard events
        keys = p.getKeyboardEvents()
        
        self.__isKeyLeftPressed = keys.get(p.B3G_LEFT_ARROW)
        self.__isKeyUpPressed = keys.get(p.B3G_UP_ARROW)
        self.__isKeyRightPressed = keys.get(p.B3G_RIGHT_ARROW)
        self.__isKeyDownPressed = keys.get(p.B3G_DOWN_ARROW)

        self.__simulationState = p.readUserDebugParameter(self.btnStartSimulation) % 2

        if self.viewMode == 1 and self.__simulationState == 0:
            self.__camAngle = self.cameraAngle
            self.__camHeight = self.cameraHeight
            self.__camDistance = self.cameraDistance

        carBasePos, carBaseOriQuat = p.getBasePositionAndOrientation(self.car)
        [self.__carBaseX, self.__carBaseY, self.__carBaseZ] = carBasePos
        [self.__carBaseRoll, self.__carBasePitch, self.__carBaseYaw] = p.getEulerFromQuaternion(carBaseOriQuat)

    def carControl(self):
        if self.__simulationState == 1: return

        if self.viewMode == 0:
            self.__carBaseX = self.carX
            self.__carBaseY = self.carY
            self.__carBaseYaw = self.carYaw
        elif self.viewMode == 1:
            pass
        
        p.resetBasePositionAndOrientation(self.car, [self.__carBaseX, self.__carBaseY, self.__carBaseZ], p.getQuaternionFromEuler([self.__carBaseRoll, self.__carBasePitch, self.__carBaseYaw]))

    def motorControl(self, user_throttle, user_steering):
        if self.__simulationState == 0:
            targetV = 0
            targetP = 0
        else:
            if self.viewMode == 0:
                targetV = self.carThr
                targetP = - self.carSte
            elif self.viewMode == 1:
                targetV = user_throttle
                targetP = - user_steering

        p.setJointMotorControlArray(self.car, self.wheel_indices, p.VELOCITY_CONTROL, targetVelocities=[targetV]*4)
        p.setJointMotorControlArray(self.car, self.hinge_indices, p.POSITION_CONTROL, targetPositions=[targetP]*2)
        
        # p.setJointMotorControl2(self.car, 7, p.POSITION_CONTROL, targetPosition=(p.readUserDebugParameter(self.cameraHeight)))
        
    def cameraControl(self):
        if self.viewMode == 0:
            *_, cameraYaw, cameraPitch, cameraDistance, [camX, camY, camZ] = p.getDebugVisualizerCamera()

            if (self.__isKeyLeftPressed):
                camX -= .02 * np.cos(np.deg2rad(cameraYaw))
                camY -= .02 * np.sin(np.deg2rad(cameraYaw))
            if (self.__isKeyRightPressed):
                camX += .02 * np.cos(np.deg2rad(cameraYaw))
                camY += .02 * np.sin(np.deg2rad(cameraYaw))
            if (self.__isKeyUpPressed):
                camX -= .02 * np.sin(np.deg2rad(cameraYaw))
                camY += .02 * np.cos(np.deg2rad(cameraYaw))
            if (self.__isKeyDownPressed):
                camX += .02 * np.sin(np.deg2rad(cameraYaw))
                camY -= .02 * np.cos(np.deg2rad(cameraYaw))
            
            cameraTargetPosition = [camX, camY, camZ]
        elif self.viewMode == 1:
            cameraDistance = self.__camDistance
            cameraYaw = np.rad2deg(self.__carBaseYaw - np.pi / 2)
            cameraPitch = - np.rad2deg(self.__camAngle)
            planarOffset = (1 - self.__camDistance * (1 - np.cos(self.__camAngle)))
            cameraTargetPosition = (
                self.__carBaseX + np.cos(self.__carBaseYaw) * planarOffset,
                self.__carBaseY + np.sin(self.__carBaseYaw) * planarOffset,
                self.__carBaseZ + self.__camHeight - self.__camDistance * np.sin(self.__camAngle)
            )

        p.resetDebugVisualizerCamera(cameraDistance, cameraYaw, cameraPitch, cameraTargetPosition)

    def pictureControl(self):
        if self.viewMode != 1: return

        self.carImage = p.getCameraImage(IMG_WIDTH, IMG_HEIGHT)[2]

        if self.lastTakePicClicked != p.readUserDebugParameter(self.btnTakePic):
            im = Image.fromarray(self.getCarCameraImage(), mode="L")
            im.save("capture.png")
            
            self.lastTakePicClicked = p.readUserDebugParameter(self.btnTakePic)

    def pathControl(self):
        if self.viewMode == 0:
            if self.lastRemoveMarkClicked != p.readUserDebugParameter(self.btnRemoveMark):
                p.removeAllUserDebugItems()
                
                self.lastRemoveMarkClicked = p.readUserDebugParameter(self.btnRemoveMark)
        elif self.viewMode == 1:
            if self.__counter % 50 == 0:
                self.leaveMarking(self.trajectoryOn == 1)

    def leaveMarking(self, mark):
        LFmarkTo, RFmarkTo, LBmarkTo, RBmarkTo = p.getLinkStates(self.car, self.wheel_indices)
        if mark:
            p.addUserDebugLine(self.__markFrom[0], LFmarkTo[0], [1, 0, 1], 1, 0)
            p.addUserDebugLine(self.__markFrom[1], RFmarkTo[0], [1, 0, 1], 1, 0)
            p.addUserDebugLine(self.__markFrom[2], LBmarkTo[0], [1, 1, 0], 1, 0)
            p.addUserDebugLine(self.__markFrom[3], RBmarkTo[0], [1, 1, 0], 1, 0)
        self.__markFrom = LFmarkTo[0], RFmarkTo[0], LBmarkTo[0], RBmarkTo[0]

    def getCarCameraImage(self):
        return np.dot(self.carImage[...,:3], [0.299, 0.587, 0.114]).astype(np.uint8)

    def nextFrame(self, delay=DEFAULT_FRAME_RATE):
        p.stepSimulation()
        sleep(delay)

class pySCserver:
    def __init__(self):
        self.pBV = pyBulletView()

        self.__recv_str = ""

        self.__motor_speed = 0
        self.__motor_turn = 0

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(("localhost", 8888))
        self.server.setblocking(False)
        self.server.listen(1)

        self.inputs = [self.server]
        self.outputs = []

        print("-----Server started")
        self.__counter = 0

    def __del__(self):
        # try:
        #     self.connection.close()
        # except:
        #     pass
        print("-----Server ended with counter: " + str(self.__counter))

    def loop(self, delay=DEFAULT_FRAME_RATE):
        self.pBV.controlInputAndParam(self.__counter)

        self.pBV.carControl()
        self.pBV.motorControl(self.__motor_speed, self.__motor_turn)
        self.pBV.cameraControl()
        self.pBV.pictureControl()
        self.pBV.pathControl()

        self.pBV.nextFrame(delay)

    def addCounter(self):
        self.__counter += 1
    def getCounter(self):
        return self.__counter

    def receive(self, num_bytes):
        readable, _, exceptional = select.select(self.inputs, self.outputs, self.inputs, 0)

        for s in readable:
            if s is self.server:
                self.connection, self.address = s.accept()
                self.connection.setblocking(False)
                self.inputs.append(self.connection)
            else:
                try:
                    self.__recv_str = s.recv(num_bytes).decode("ascii")
                    if self.__recv_str:
                        if s not in self.outputs:
                            self.outputs.append(s)
                    else:
                        if s in self.outputs:
                            self.outputs.remove(s)
                        self.inputs.remove(s)
                        s.close()
                except:
                    pass

        for s in exceptional:
            self.inputs.remove(s)
            if s in self.outputs:
                self.outputs.remove(s)
            s.close()
    def getReceivedString(self):
        return self.__recv_str

    def send(self, data):
        _, writable, exceptional = select.select(self.inputs, self.outputs, self.inputs, 0)

        for sck in writable:
            sck.sendall(data)
            self.outputs.remove(sck)

        for s in exceptional:
            self.inputs.remove(s)
            if s in self.outputs:
                self.outputs.remove(s)
            s.close()
        
    def parseCommand(self, debugMode=1):
        if not self.__recv_str:
            return

        if len(self.__recv_str) == 1:
            self.__recv_str += '\0\0'
        elif len(self.__recv_str) == 2:
            self.__recv_str += '\0'

        parsedValue = (ord(self.__recv_str[2]) - 1) * 128 + ord(self.__recv_str[1])

        if self.__recv_str == 'PNG':
            if debugMode in [0, 2]: print("Got a Ping. Giving out an Acknowledgement.")
            if debugMode in [1, 2]: self.send(bytes("ACK", "ascii"))
        elif self.__recv_str[0] == 'F':
            if debugMode in [0, 2]: print("Move forward for speed of", parsedValue)
            if debugMode in [1, 2]: self.__motor_speed = parsedValue * MAX_THROTTLE / 16256
        elif self.__recv_str[0] == 'B':
            if debugMode in [0, 2]: print("Move backward for speed of", parsedValue)
            if debugMode in [1, 2]: self.__motor_speed = -parsedValue * MAX_THROTTLE / 16256
        elif self.__recv_str[0] == 'L':
            if debugMode in [0, 2]: print("Turn left for degree of", parsedValue)
            if debugMode in [1, 2]: self.__motor_turn = -parsedValue * MAX_STEERING / 16256
        elif self.__recv_str[0] == 'R':
            if debugMode in [0, 2]: print("Turn right for degree of", parsedValue)
            if debugMode in [1, 2]: self.__motor_turn = parsedValue * MAX_STEERING / 16256
        elif self.__recv_str == 'STP':
            if debugMode in [0, 2]: print("Stop moving")
            if debugMode in [1, 2]: self.__motor_speed = 0
        elif self.__recv_str == 'STR':
            if debugMode in [0, 2]: print("Turn straight")
            if debugMode in [1, 2]: self.__motor_turn = 0
        elif self.__recv_str == 'GET':
            img_matrix = self.pBV.getCarCameraImage()
            
            if debugMode in [0, 2]: print("Send image")
            if debugMode in [1, 2]: self.send(imgEncode(img_matrix, IMG_WIDTH, IMG_HEIGHT))
        else:
            print("Unknown command")

        if (self.__recv_str[0] in ['F', 'B', 'L', 'R', 'S']) or self.__recv_str == 'GET' and debugMode == 0:
            self.send(b"X")

def generateZeros(img_width, img_height):
    return np.zeros((img_width, img_height), dtype=np.uint8)
def generateOnes(img_width, img_height):
    return np.ones((img_width, img_height), dtype=np.uint8)
def generateValue(fill_value, img_width, img_height):
    return np.full((img_width, img_height), fill_value=fill_value, dtype=np.uint8)
def generateRandom(img_width, img_height):
    return np.random.randint(256, size=(IMG_HEIGHT, IMG_WIDTH))
def generateGradient(img_width, img_height):
    img = np.zeros((img_height, img_width), dtype=np.uint8)

    for i in range(img_height):
        for j in range(img_width):
            img[i, j] = (i + j) / (img_width + img_height) * 256

    return img

def imgEncode(img_matrix, img_width, img_height):
    img_serial = img_matrix.reshape(img_width * img_height)
    return bytes(img_serial)
