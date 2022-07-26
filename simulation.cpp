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
    
    sc.motorTurn(15000);
    std::cout << "turn right now" << std::endl;

    Sleep(1000);

    sc.motorSpeed(15000);
    std::cout << "go fast now" << std::endl;

    Sleep(1000);

    sc.motorTurn(-15000);
    std::cout << "turn left now" << std::endl;
}

int main() {
    serverSmartCar server;
    server.connectServer();

    // debugControl(server);
    // simpleControl(server);
    server.motorSpeed(15000);

    return 0;
}
