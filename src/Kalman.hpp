#pragma once
#include <string>
#include <vector>
#include "vec3.hpp"
#include "UDPClient.hpp"

inline constexpr int MAXLINE = 1024;

class Kalman
{
private:
    std::vector<vec3<double>> data;
    UDPClient client;
    vec3<double> direction;
    vec3<double> acceleration;
    double speed;
    char buffer[MAXLINE];

public:
    void extract_data(std::string server_data);
    void filter_loop();
    void parse_data(std::string stream);
    vec3<double> calculate_estimation();
    Kalman(int port = 8080, std::string handshake = "READY");
    ~Kalman();
};