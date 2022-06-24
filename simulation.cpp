#include <iostream>
#include "lib/SClib.cpp"
using namespace std;

int main() {
    serverSmartCar hi;
    hi.connectServer();

    hi.getImg();
    // hi.printImg();

    hi.motorSpeed(-19800);
    hi.motorSpeed(-16256);
    hi.motorSpeed(-16255);
    hi.motorSpeed(-150);
    hi.motorSpeed(-100);

    hi.motorSpeed(0);

    hi.motorSpeed(100);
    hi.motorSpeed(150);
    hi.motorSpeed(16255);
    hi.motorSpeed(16256);
    hi.motorSpeed(19800);

    hi.motorTurn(-19800);
    hi.motorTurn(-16256);
    hi.motorTurn(-16255);
    hi.motorTurn(-150);
    hi.motorTurn(-100);

    hi.motorTurn(0);
    
    hi.motorTurn(100);
    hi.motorTurn(150);
    hi.motorTurn(16255);
    hi.motorTurn(16256);
    hi.motorTurn(19800);

    return 0;
}