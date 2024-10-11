#pragma once
#include <string>
#include <vector>
#include "UDPClient.hpp"
#include <Eigen/Dense>

inline constexpr int MAXLINE = 1024;

class Kalman
{
private:
    UDPClient client;
    double speed;
    char buffer[MAXLINE];
    Eigen::Vector3d acceleration;
    Eigen::MatrixXd F; // State transition matrix
    Eigen::MatrixXd B; // Control input matrix (acceleration)
    Eigen::MatrixXd H; // Measurement matrix
    Eigen::MatrixXd Q; // Process noise covariance matrix
    Eigen::MatrixXd R; // Measurement noise covariance matrix
    Eigen::MatrixXd P; // Error covariance matrix
    Eigen::VectorXd X; // State vector (position, velocity) X(0): Position x. X(1): Position y.  X(2): Position  z. X(3): Velocity  x. X(4): Velocity  y.  X(5): Velocity  z.
    Eigen::VectorXd Z; // Measurement vector (GPS position)

public:
    void extract_data(std::string server_data);
    void filter_loop();
    void parse_data(std::string stream);
    Eigen::Vector3d calculate_estimation();
    Kalman(int port = 8080, std::string handshake = "READY");
    ~Kalman();
};