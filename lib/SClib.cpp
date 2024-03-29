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
}
serverSmartCar::~serverSmartCar() {
	if (!hasErr) {
		closesocket(sockClient);
	}
	WSACleanup();

	getchar();
}

int serverSmartCar::initServer(const char* ip = "127.0.0.1", const int port = 8888) {
	if (hasErr) return -1;

	addrSrv.sin_addr.S_un.S_addr = inet_addr(ip);
	addrSrv.sin_family = AF_INET;
	addrSrv.sin_port = htons(port);

	lostConnection = true;

	return 0;
}
int serverSmartCar::connectServer() {
	/* This can be put inside a loop
		adv: keep trying to connect to the server, 
			so that if the server is closed, it will run still
		disadv: making python select overwhelmed
		sol: only try to reconnect when not receiving properly
			+ remove outdated writables / output in select
	*/

	if (hasErr) return -1;

	// if (lostConnection) {
		sockClient = socket(AF_INET, SOCK_STREAM, 0);
		connect(sockClient, (SOCKADDR*)&addrSrv, sizeof(SOCKADDR));
	// }

	return 0;
}

void serverSmartCar::getImg1D(char *pixels) {
	if (hasErr) return;

	const int NUM_PIXEL_ENCODED = IMG_WIDTH * IMG_HEIGHT;

	std::string temp = "GET";
    char const *baseCh = temp.c_str();
	send(sockClient, baseCh, strlen(baseCh), 0);

	char recvBuf[NUM_PIXEL_ENCODED];
	recv(sockClient, recvBuf, NUM_PIXEL_ENCODED, 0);

	for (int i = 0; i < NUM_PIXEL_ENCODED; i++) {
		pixels[i*3] = recvBuf[i];
		pixels[i*3 + 1] = recvBuf[i];
		pixels[i*3 + 2] = recvBuf[i];
	}
}
void serverSmartCar::getImg2D() {
	if (hasErr) return;

	const int NUM_PIXEL_ENCODED = IMG_WIDTH * IMG_HEIGHT;

	std::string temp = "GET";
    char const *baseCh = temp.c_str();
	send(sockClient, baseCh, strlen(baseCh), 0);

	char recvBuf[NUM_PIXEL_ENCODED];
	recv(sockClient, recvBuf, NUM_PIXEL_ENCODED, 0);

	for (int i = 0; i < NUM_PIXEL_ENCODED; i++) {
		img_matrix[i / IMG_WIDTH][i % IMG_WIDTH] = recvBuf[i];
	}
}
uint8_t serverSmartCar::getPixel(int x, int y) {
	return img_matrix[y][x];
}
void serverSmartCar::printImg() {
	for (int y = 0; y < IMG_HEIGHT; y++) {
		for (int x = 0; x < IMG_WIDTH; x++)
			printf("%4d ", img_matrix[y][x]);
		printf("\n");
	}
}

void serverSmartCar::motorSpeed(int speed = 0) {
	if (hasErr) return;

	std::string temp = "   ";

	if (speed == 0) {
		temp = "STP";
	} else {
		temp[0] = speed > 0 ? 'F' : 'B';
		if (speed < 0) speed *= -1;
		if (speed > 16255) speed = 16255;
		temp[2] = speed / 128 + 1;
		temp[1] = speed % 128;
	}

    char const *baseCh = temp.c_str();
	send(sockClient, baseCh, strlen(baseCh), 0);

	char recvBuf[1];
	recv(sockClient, recvBuf, 1, 0);
}
void serverSmartCar::motorTurn(int angle = 0) {
	if (hasErr) return;

	std::string temp = "   ";

	if (angle == 0) {
		temp = "STR";
	} else {
		temp[0] = angle > 0 ? 'R' : 'L';
		if (angle < 0) angle *= -1;
		if (angle > 16255) angle = 16255;
		temp[2] = angle / 128 + 1;
		temp[1] = angle % 128;
	}

    char const *baseCh = temp.c_str();
	send(sockClient, baseCh, strlen(baseCh), 0);

	char recvBuf[1];
	recv(sockClient, recvBuf, 1, 0);
}
