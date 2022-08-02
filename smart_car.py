import lib.smart_car_server as scs

if __name__ == '__main__':
    sc = scs.pySCserver()

    while True:
        sc.receive(3)

        # 0 for debug message, 1 for real control and 2 for both
        sc.parseCommand(1)

        sc.loop()

        sc.addCounter()
        # print(sc.getCounter())
