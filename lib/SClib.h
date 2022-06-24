#ifndef LIB_CLIENT_H
#define LIB_CLIENT_H

#include <stdio.h>
#include <Winsock2.h>
#include <windows.h>

#define IMG_WIDTH 128
#define IMG_HEIGHT 120

// #define IMG_WIDTH 10
// #define IMG_HEIGHT 12

#pragma comment(lib,"ws2_32.lib")
#pragma warning(disable:4996)

class serverSmartCar {
    public:
        serverSmartCar();
        ~serverSmartCar();

        int connectServer(const char* ip, const int port);

        void motorSpeed(int speed);
        void motorTurn(int angle);

        void getImg();
        void printImg();

    private:
        WORD wVersionRequested;
        WSADATA wsaData;
        int err;

        bool hasErr;

        SOCKET sockClient;
        SOCKADDR_IN addrSrv;

        uint8_t img_matrix[IMG_HEIGHT][IMG_WIDTH]; 
};

#endif