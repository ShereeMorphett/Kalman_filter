#pragma once
#include <string>
#include <vector>
#include "UDPClient.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <map>

inline constexpr int MAXLINE = 1024;

class Kalman
{
private:
    UDPClient client;
    unsigned int k; // current iteration of Kalman
    double speed;
    char buffer[MAXLINE];
    std::string true_position; //Todo: should this be here?
    const double variance_accelerometer = 1e-9; // 1e-3 * 1e-3
    const double variance_gyroscope = 1e-4;     // 1e-2 * 1e-2
    const double variance_gps = 1e-2;           // 1e-1 * 1e-1
    const double kmh_to_ms = 1000. / 3600.;

    Eigen::VectorXd StateVector;              // State vector (position, velocity) X(0): Position x. X(1): Position y.  X(2): Position  z. X(3): Velocity  x. X(4): Velocity  y.  X(5): Velocity  z.  -
    Eigen::MatrixXd StateTransitionMatrix;    // State transition matrix  -
    Eigen::MatrixXd ProcessErrorMatrix;       // Process noise covariance matrix  -
    Eigen::MatrixXd ControlInputMatrix;       // Control input matrix (acceleration)  - U NOT NECESSARY
    Eigen::MatrixXd MeasurementNoiseMatrix;   // Measurement noise covariance matrix  -
    Eigen::MatrixXd MeasurementToStateMatrix; // Measurement matrix  - H
    Eigen::MatrixXd ErrorCovarianceMatrix;    // Error covariance matrix  -
    Eigen::VectorXd MeasurementVector;        // Measurement vector (GPS position)   -
    std::chrono::time_point<std::chrono::steady_clock> last_update;

    enum class Type
    {
        Position,
        Acceleration,
        Velocity,
        Direction,
        TruePosition
    };

    struct MeasurementData
    {
        Eigen::Vector3d values;
        Type type;
    };

    MeasurementData last_orientation;
    MeasurementData parse_eigen_vec3(std::istringstream& data, Kalman::Type type);
    MeasurementData parse_data(std::string str_buffer);
    void process_data(MeasurementData data);

    double get_dt();
    void set_state_transition_matrix();
    void update_state_transition_matrix(double dt, MeasurementData data);
    void set_process_error_matrix();
    Eigen::MatrixXd get_body_to_inertial_rotation(Eigen::Vector3d angles);
    void set_measurement_vector(MeasurementData& data);
    void set_measurement_to_state_matrix();

    void predict();
    void update();
    void send_result();

public:
    void filter_loop();
    Kalman(int port = 8080, std::string handshake = "READY");
    ~Kalman();
};