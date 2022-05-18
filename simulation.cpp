#include <iostream>
#include "lib/SClib.cpp"
using namespace std;

int main() {
    serverSmartCar hi;
    hi.connectServer();

    hi.motorSpeed(19800);

    Sleep(1000);

    hi.motorTurn(-80);

    return 0;
}