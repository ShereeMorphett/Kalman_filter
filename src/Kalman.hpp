#pragma once
#include <string>
#include <vector>

#include "UDPClient.hpp"

inline constexpr int MAXLINE = 1024;

class Kalman
{
private:
    std::vector<std::vector<double>> data;
    UDPClient client;
    char buffer[MAXLINE];

public:
    void extract_data(std::string server_data);
    void filter_loop();
    Kalman(int port = 8080, std::string handshake = "READY");
    ~Kalman();
};