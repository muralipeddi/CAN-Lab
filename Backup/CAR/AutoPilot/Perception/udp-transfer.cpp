#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>

#include <chrono>
#include <thread>

int main() {
    const char* ip = "127.0.0.1";
    //const char* ip = "192.168.0.112";
    int port = 5002;

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "Error creating socket" << std::endl;
        return 1;
    }

    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    serverAddr.sin_addr.s_addr = inet_addr(ip);

    while (true) {
        int x = 250, y = 900;
        char message[50];
        sprintf(message, "X: %d, Y: %d", x, y);

        if (sendto(sockfd, message, strlen(message), 0, reinterpret_cast <const sockaddr*>(&serverAddr), sizeof(serverAddr)) < 0) {
            std::cerr << "Error in sendto" << std::endl;
            return 1;
        }

        std::cout << "Message sent: " << message << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    close(sockfd);

    return 0;

}