import numpy as np

import pybullet as p
import pybullet_data

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

# IMG_WIDTH = 10
# IMG_HEIGHT = 12

class pyBulletView:
    def __init__(self):
        # Basic pyBullet config
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, A_G)
        
        # PyBullet view config
        self.carX = p.addUserDebugParameter('Car X coordinate', -5, 5, 0)
        self.carY = p.addUserDebugParameter('Car Y coordinate', -5, 5, 0)
        self.carA = p.addUserDebugParameter('Car Direction', -np.pi, np.pi, 0)
        self.steering = p.addUserDebugParameter('Steering', -MAX_STEERING, MAX_STEERING, 0)
        self.throttle = p.addUserDebugParameter('Throttle', 0, MAX_THROTTLE, 0)

        self.cameraHeight = p.addUserDebugParameter('Car Camera Height', 0.1, 0.25, 0.25)
        self.cameraOffset = p.addUserDebugParameter('Car Camera Offset', 0.8, 1.2, 0.85)
        self.cameraAngle = p.addUserDebugParameter('Car Camera Angle', -1, 1, 0.5)
        self.switchCamera = p.addUserDebugParameter('God view / Car camera', 1, 0, 1)
        self.takePic = p.addUserDebugParameter('Take Picture', 1, 0, 1)

        self.isSimulating = p.addUserDebugParameter('Enable / Disable simulation', 1, 0, 1)

        self.takePicClicked = p.readUserDebugParameter(self.takePic)
        self.carImage = p.getCameraImage(IMG_WIDTH, IMG_HEIGHT)[2]
        
        # PyBullet load materials
        self.car = p.loadURDF('/src/simplecar.urdf', [0, 0, 0.1], globalScaling=0.5)
        self.plane = p.loadURDF('/src/simpleplane.urdf')
        self.trackId = p.createVisualShape(p.GEOM_MESH, fileName="/src/track.obj", meshScale=[1]*3, rgbaColor=[1]*3+[1])
        p.createMultiBody(0, baseVisualShapeIndex=self.trackId, basePosition=[-4, -4, -0.08])

        self.wheel_indices = [1, 3, 4, 5]
        self.hinge_indices = [0, 2]
        self.camera_indices = [6, 7, 8]
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

    def carControl(self):
        if p.readUserDebugParameter(self.isSimulating) % 2 == 0: return

        carBasePosition = list(p.getBasePositionAndOrientation(self.car)[0])
        carBaseOrientationEuler = list(p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.car)[1]))

        carBasePosition[0] = p.readUserDebugParameter(self.carX)
        carBasePosition[1] = p.readUserDebugParameter(self.carY)
        carBaseOrientationEuler[2] = p.readUserDebugParameter(self.carA)

        p.resetBasePositionAndOrientation(self.car, carBasePosition, p.getQuaternionFromEuler(carBaseOrientationEuler))

    def motorControl(self, user_throttle, user_steering):
        # receiving == 1 means getting command and drive but 0 means manual control in debugging
        receiving = 1
        for joint_index in self.wheel_indices:
            p.setJointMotorControl2(self.car, joint_index, p.VELOCITY_CONTROL, targetVelocity=(p.readUserDebugParameter(self.throttle) if (p.readUserDebugParameter(self.isSimulating) % 2 == receiving) else user_throttle))
        for joint_index in self.hinge_indices:
            p.setJointMotorControl2(self.car, joint_index, p.POSITION_CONTROL, targetPosition=(-p.readUserDebugParameter(self.steering) if (p.readUserDebugParameter(self.isSimulating) % 2 == receiving) else -user_steering))

    def cameraControl(self):
        carNumClicked = p.readUserDebugParameter(self.switchCamera)
        position, orientation = p.getBasePositionAndOrientation(self.car)

        if carNumClicked % 2 == 0:
            _, _, yaw = p.getEulerFromQuaternion(orientation)
            camDist = p.readUserDebugParameter(self.cameraOffset)
            camAngle = p.readUserDebugParameter(self.cameraAngle)
            p.resetDebugVisualizerCamera(cameraDistance=camDist,
                                        cameraYaw=np.degrees(yaw - np.pi / 2),
                                        cameraPitch=-camAngle*180/np.pi,
                                        cameraTargetPosition=(position[0] + np.cos(yaw) * (1 - camDist * (1 - np.cos(camAngle))),
                                                            position[1] + np.sin(yaw) * (1 - camDist * (1 - np.cos(camAngle))),
                                                            position[2] + p.readUserDebugParameter(self.cameraHeight) - camDist * np.sin(camAngle)))
            
            self.carImage = p.getCameraImage(IMG_WIDTH, IMG_HEIGHT)[2]
        else:
            # p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=50, cameraPitch=-25, cameraTargetPosition=(0, 0, 0))
            pass

    def pictureControl(self):
        if self.takePicClicked != p.readUserDebugParameter(self.takePic):
            grey = np.uint8(np.dot(self.carImage[...,:3], [0.2989, 0.5870, 0.1140]))
            im = Image.fromarray(grey, mode="L")
            # im = Image.fromarray(image, mode="RGBA").convert("L")
            
            im.save("capture.png")
            
            self.takePicClicked = p.readUserDebugParameter(self.takePic)

    def nextFrame(self, delay=DEFAULT_FRAME_RATE):
        p.stepSimulation()
        sleep(delay)

class pySCserver:
    def __init__(self):
        self.pBV = pyBulletView()

        self.__motor_speed = 0
        self.__motor_turn = 0

        self.__recv_str = ""

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
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
        self.pBV.carControl()
        self.pBV.motorControl(self.__motor_speed, self.__motor_turn)
        self.pBV.cameraControl()
        self.pBV.pictureControl()

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
                self.__recv_str = s.recv(num_bytes).decode("ascii")
                if self.__recv_str:
                    if s not in self.outputs:
                        self.outputs.append(s)
                else:
                    if s in self.outputs:
                        self.outputs.remove(s)
                    self.inputs.remove(s)
                    s.close()

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

        if self.__recv_str[0] == 'F':
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
            img_matrix = np.uint8(np.dot((p.getCameraImage(IMG_WIDTH, IMG_HEIGHT)[2])[...,:3], [0.2989, 0.5870, 0.1140]))
            
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
