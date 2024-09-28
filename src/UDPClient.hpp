#pragma once
#include <iostream>
#include <cstring>
#include <cstdlib>
#include <string>
#include <memory>
#include <arpa/inet.h>
#include <unistd.h>


class UDPClient
{
private:
    int sock_fd;
    struct sockaddr_in servaddr;
    socklen_t len;

public:
    UDPClient(int port);
    ~UDPClient();
    void send_handshake(std::string handshake);
    int get_sock_fd();
    socklen_t get_sock_len();
    sockaddr_in get_servaddr();
    void send_estimation(const std::string &estimation);
};