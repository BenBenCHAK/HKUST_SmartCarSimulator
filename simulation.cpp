#include <iostream>

#include "lib/SClib.cpp"

void debugControl(serverSmartCar &sc) {
    sc.getImg2D();

    sc.motorSpeed(-19800);
    sc.motorSpeed(-16256);
    sc.motorSpeed(-16255);
    sc.motorSpeed(-150);
    sc.motorSpeed(-100);

    sc.motorSpeed(0);

    sc.motorSpeed(100);
    sc.motorSpeed(150);
    sc.motorSpeed(16255);
    sc.motorSpeed(16256);
    sc.motorSpeed(19800);

    sc.motorTurn(-19800);
    sc.motorTurn(-16256);
    sc.motorTurn(-16255);
    sc.motorTurn(-150);
    sc.motorTurn(-100);

    sc.motorTurn(0);
    
    sc.motorTurn(100);
    sc.motorTurn(150);
    sc.motorTurn(16255);
    sc.motorTurn(16256);
    sc.motorTurn(19800);
}

void simpleControl(serverSmartCar &sc) {
    Sleep(1000);

    sc.motorTurn(10000);

    Sleep(100);

    sc.motorSpeed(15000);

    Sleep(5000);

    sc.motorTurn(0);
}

void imageControl(serverSmartCar &sc) {
    sc.getImg2D();

    int leftW = 0, rightW = 0;
    
    for (int y = 60; y < 120; y++) {
        for (int x = 0; x < 64; x++) {
            if (sc.getPixel(x, y) > 200)
                leftW++;
        }
        for (int x = 64; x < 128; x++) {
            if (sc.getPixel(x, y) > 200)
                rightW++;
        }
    }

    sc.motorTurn((rightW - leftW) * 150);
    sc.motorSpeed(10000);

    // Check if certain pixel is correct
    // printf("%d\n", sc.getPixel(127, 119));
}

int main() {
    serverSmartCar server;
    server.connectServer();

    // debugControl(server);
    // simpleControl(server);
    while (true) {
        imageControl(server);
        Sleep(20);
    }

    return 0;
}
