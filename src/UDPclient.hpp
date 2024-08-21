#include <iostream>
#include <cstring>
#include <cstdlib>
#include <string>
#include <memory>
#include <arpa/inet.h>
#include <unistd.h>

#define PORT 8080
#define MAXLINE 1024  // this needs fixing

class UDPClient
{
    private:
        int sockfd;
        char buffer[MAXLINE];
        struct sockaddr_in servaddr;
        socklen_t len;
    
    public:
        UDPClient();
        ~UDPClient();

        void sendHandshake();
        void receiveInitialization();
        void sendEstimation(const std::string& estimation);
        void processLoop();

};