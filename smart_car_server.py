import numpy as np
import socket
import time

IMG_WIDTH = 10#128
IMG_HEIGHT = 12#120

class pySCserver:
    def __init__(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind(("localhost", 8888))
        print("Server started")
        self.server.listen(0)
        self.connection, self.address = self.server.accept()

        self.__counter = 0

    def __del__(self):
        self.connection.close()
        input("Server ended with counter: " + str(self.__counter))

    def addCounter(self):
        self.__counter += 1
    def getCounter(self):
        return self.__counter

    def receive(self, num_bytes):
        self.__recv_str = self.connection.recv(1024)[0:num_bytes].decode("ascii")
    def getReceivedString(self):
        return self.__recv_str

    def send(self, data):
        self.connection.sendall(data)

    def parseCommand(self, num_bytes):
        if self.__recv_str[0] == 'F':
            print("Move forward for speed of", int(self.__recv_str[1:num_bytes]))
        elif self.__recv_str[0] == 'B':
            print("Move backward for speed of", int(self.__recv_str[1:num_bytes]))
        elif self.__recv_str[0] == 'L':
            print("Turn left for degree of", int(self.__recv_str[1:num_bytes]))
        elif self.__recv_str[0] == 'R':
            print("Turn right for degree of", int(self.__recv_str[1:num_bytes]))
        else:
            print("Unknown command")

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
    temp_string = ""
    img_serial = img_matrix.reshape(img_width * img_height)

    for pixel in img_serial:
        if pixel >= 0 and pixel < 128:
            temp_string += chr(0)
            temp_string += chr(pixel)
        elif pixel >= 128 and pixel < 256:
            temp_string += chr(127)
            temp_string += chr(pixel - 128)

    return bytes(temp_string, encoding="ascii")

    # return bytes("".join(chr(0) + chr(pixel) if pixel >= 0 and pixel < 128 else chr(127) + chr(pixel - 128) for pixel in img_serial), encoding="ascii")

if __name__ == '__main__':
    sc = pySCserver()

    while True:
        sc.addCounter()

        sc.receive(5)
        if not sc.getReceivedString():
            break

        sc.parseCommand(5)

        img = generateRandom(IMG_WIDTH, IMG_HEIGHT)
        sc.send(imgEncode(img, IMG_WIDTH, IMG_HEIGHT))

        time.sleep(0.1)
