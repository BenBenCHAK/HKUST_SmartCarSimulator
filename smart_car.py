import lib.smart_car_server as scs

import numpy as np
import pybullet as p
from PIL import Image
from time import sleep
import pybullet_data

if __name__ == '__main__':
    sc = scs.pySCserver()

    while True:
        sc.addCounter()

        sc.receive(3)
        if not sc.getReceivedString():
            break

        sc.parseCommand(3)

        sleep(0.1)
