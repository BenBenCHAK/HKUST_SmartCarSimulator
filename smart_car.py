import lib.smart_car_server as scs
# import lib.SClib as pyb

import numpy as np
import pybullet as p
from PIL import Image
import pybullet_data

if __name__ == '__main__':
    sc = scs.pySCserver()

    while True:
        sc.receive(3)

        sc.parseCommand(2)

        sc.loop()

        sc.addCounter()
        # print(sc.getCounter())
