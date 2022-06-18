#include "SClib.h"
#include <string>

serverSmartCar::serverSmartCar() {
	hasErr = false;

	wVersionRequested = MAKEWORD(1, 1);

	err = WSAStartup(wVersionRequested, &wsaData);
	if (err != 0) {
		hasErr = true;
	}

	if (LOBYTE(wsaData.wVersion) != 1 || HIBYTE(wsaData.wVersion) != 1) {
		WSACleanup();
		hasErr = true;
	}

	// if (!hasErr) {
	// 	std::string temp = "     ";
	// 	char const *baseCh = temp.c_str();
	// 	send(sockClient, baseCh, strlen(baseCh), 0);
	// }
}
serverSmartCar::~serverSmartCar() {
	if (!hasErr) {
		closesocket(sockClient);
	}
	WSACleanup();

	getchar();
}

int serverSmartCar::connectServer(const char* ip = "127.0.0.1", const int port = 8888) {
	if (hasErr) return -1;

	sockClient = socket(AF_INET, SOCK_STREAM, 0);

	addrSrv.sin_addr.S_un.S_addr = inet_addr(ip);
	addrSrv.sin_family = AF_INET;
	addrSrv.sin_port = htons(port);
	connect(sockClient, (SOCKADDR*)&addrSrv, sizeof(SOCKADDR));

	return 0;
}

void serverSmartCar::motorSpeed(int speed = 0) {
	if (hasErr) return;

	// getImg();

	std::string temp = "     ";
	if (speed >= 0) {
		temp[0] = 'F';
	} else if (speed < 0) {
		speed *= -1;
		temp[0] = 'B';
	}
	speed %= 10000;
	for (int i = 4; i > 0; i--) {
	    temp[i] = (char)('0' + speed % 10);
	    speed /= 10;
	}

    char const *baseCh = temp.c_str();
	send(sockClient, baseCh, strlen(baseCh), 0);

	getImg();
}
void serverSmartCar::motorTurn(int angle = 0) {
	if (hasErr) return;

	// getImg();

	std::string temp = "     ";
	if (angle >= 0) {
		temp[0] = 'R';
	} else if (angle < 0) {
		angle *= -1;
		temp[0] = 'L';
	}
	angle %= 10000;
	for (int i = 4; i > 0; i--) {
	    temp[i] = (char)('0' + angle % 10);
	    angle /= 10;
	}

    char const *baseCh = temp.c_str();
	send(sockClient, baseCh, strlen(baseCh), 0);

	getImg();
}

void serverSmartCar::getImg() {
	const int NUM_PIXEL_ENCODED = IMG_WIDTH * IMG_HEIGHT * 2;

	if (hasErr) return;

	char recvBuf[NUM_PIXEL_ENCODED];
	recv(sockClient, recvBuf, NUM_PIXEL_ENCODED, 0);
	printf("------------------------------------------\n");
	for (int i = 0; i < NUM_PIXEL_ENCODED; i += 2) {
		if (recvBuf[i] == 0) {
			img_matrix[i % IMG_HEIGHT][i % IMG_WIDTH] = recvBuf[i + 1];
		} else {
			img_matrix[i % IMG_HEIGHT][i % IMG_WIDTH] = recvBuf[i + 1] + 128;
		}

		printf("%4d", img_matrix[i % IMG_HEIGHT][i % IMG_WIDTH]);
		if ((i/2) % IMG_WIDTH == IMG_WIDTH - 1) printf("\n");
	}
	printf("------------------------------------------\n");
}
