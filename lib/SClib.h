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

        int initServer(const char* ip, const int port);
        int connectServer();

        void motorSpeed(int speed);
        void motorTurn(int angle);

        void getImg1D(char *pixels);
        void getImg2D();
        uint8_t getPixel(int x, int y);
        void printImg();

    private:
        WORD wVersionRequested;
        WSADATA wsaData;
        int err;

        bool hasErr;
        bool lostConnection;

        SOCKET sockClient;
        SOCKADDR_IN addrSrv;

        uint8_t img_matrix[IMG_HEIGHT][IMG_WIDTH];
};

#endif