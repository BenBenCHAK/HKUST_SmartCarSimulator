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

        # 0 for debug message, 1 for real control and 2 for both
        sc.parseCommand(1)

        sc.loop()

        sc.addCounter()
        # print(sc.getCounter())
