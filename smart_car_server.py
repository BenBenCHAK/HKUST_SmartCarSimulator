import socket
import time

if __name__ == '__main__':
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("localhost", 8888))
    server.listen(0)
    connection, address = server.accept()
    print(connection, address)
    num=0
    while True:
        # connection, address = server.accept()
        # print(connection, address)

        recv_str=connection.recv(1024)[0:5]
        recv_str=recv_str.decode("ascii")
        if not recv_str:
            break
        num=num+1
        # print( recv_str,num)

        if recv_str[0] == 'F':
            print("Move forward for speed of", int(recv_str[1:5]))
        elif recv_str[0] == 'B':
            print("Move backward for speed of", int(recv_str[1:5]))
        elif recv_str[0] == 'L':
            print("Turn left for degree of", int(recv_str[1:5]))
        elif recv_str[0] == 'R':
            print("Turn right for degree of", int(recv_str[1:5]))
        else:
            print("Unknown command")

        connection.send( bytes("Connected: %s," % recv_str,encoding="ascii"))
        time.sleep(0.1)


    connection.close()
    input("Server ended")