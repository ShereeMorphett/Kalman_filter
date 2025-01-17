/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   Kalman.hpp                                         :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: dorian <dorian@student.42.fr>              +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2025/01/16 18:34:12 by dorian            #+#    #+#             */
/*   Updated: 2025/01/16 19:51:10 by dorian           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#pragma once
#include <string>
#include <vector>
#include "UDPClient.hpp"
#include "types.hpp"
#include "Parser.hpp"
#include <Eigen/Dense>
#include <chrono>

class Kalman
{
private:
    UDPClient client;
    Parser parser;
    double dt = 0.01;
    const double variance_accelerometer = 1e-9; // 1e-3 * 1e-3
    const double variance_gyroscope = 1e-4;     // 1e-2 * 1e-2
    const double variance_gps = 1e-2;           // 1e-1 * 1e-1

    Eigen::VectorXd state_vector;                // State vector (position, velocity) X(0): Position x. X(1): Position y.  X(2): Position  z. X(3): Velocity  x. X(4): Velocity  y.  X(5): Velocity  z.  -
    Eigen::MatrixXd state_transition_matrix;     // State transition matrix  -
    Eigen::MatrixXd error_covariance_matrix;     // Error covariance matrix  -
    Eigen::MatrixXd process_error_matrix;        // Process noise covariance matrix  -
    Eigen::MatrixXd control_input_matrix;        // Control input matrix (acceleration)  - B
    Eigen::VectorXd control_input_vector;        // Control input noise covariance matrix  -
    Eigen::MatrixXd measurement_to_state_matrix; // Measurement matrix  - H
    Eigen::VectorXd measurement_vector;          // Measurement vector (GPS position)   -
    Eigen::MatrixXd measurement_noise_matrix;    // Measurement noise covariance matrix  -

    void set_state_vector();
    void set_state_transition_matrix();
    void set_control_input_matrix();
    void set_process_error_matrix();
    void set_error_covariance_matrix();
    void set_measurement_to_state_matrix();
    void set_measurement_noise_matrix();
    void set_control_input_vector(Parser const &parser);
    Eigen::MatrixXd get_body_to_inertial_rotation();

    void predict();
    void update();
    void send_result();

public:
    void filter_loop();
    Kalman(int port = 8080, std::string handshake = "READY");
    ~Kalman();
};