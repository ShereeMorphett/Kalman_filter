#pragma once
#include <string>
#include <vector>
#include "UDPClient.hpp"
#include <iostream>


inline constexpr int MAXLINE = 100024;

class Server
{
private:
    UDPClient client;
    char buffer[MAXLINE];

    void predict();
    void update();
    void send_result();

public:
    void loop();
    void parse_data(std::string str_buffer);
    Server(int port = 8080, std::string handshake = "READY");
    ~Server();
};
