import lib.smart_car_server as scs
# import lib.SClib as pyb

import numpy as np
import pybullet as p
from PIL import Image
import pybullet_data

import threading

# def SCreceive():
#     sc.receive(3)

# t = threading.Thread(target=SCreceive)

if __name__ == '__main__':
    sc = scs.pySCserver()

    # t.start()

    while True:
        sc.addCounter()

        sc.receive(3)
        # if not sc.getReceivedString():
        #     break

        sc.parseCommand(2)

        sc.loop()
