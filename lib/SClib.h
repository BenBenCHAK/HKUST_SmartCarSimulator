#ifndef LIB_CLIENT_H
#define LIB_CLIENT_H

#include <stdio.h>  
#include <Winsock2.h>
#include <windows.h>
#pragma comment(lib,"ws2_32.lib")
#pragma warning(disable:4996)
#include <iostream>

class serverSmartCar {
    public:
        serverSmartCar();
        ~serverSmartCar();

        int connectServer(const char* ip, const int port);

        void motorSpeed(int speed);
        void motorTurn(int angle);

    private:
        WORD wVersionRequested;
        WSADATA wsaData;
        int err;

        bool hasErr;

        SOCKET sockClient;
        SOCKADDR_IN addrSrv;
};



#endif