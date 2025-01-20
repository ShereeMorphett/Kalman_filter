#include "Kalman.hpp"
#include <iomanip>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <iostream>
#include "colour.hpp"

/*
    state_vector(6),
    state_transition_matrix(6, 6),
    process_error_matrix(6, 6),
    measurement_noise_matrix(6, 6),
    measurement_to_state_matrix(6, 3),
    error_covariance_matrix(6, 6),
    measurement_vector(6),
    initalized(false)
*/

void Kalman::update()
{
    std::cout << "In update()" << std::endl;
    Eigen::MatrixXd InnovationCov = measurement_to_state_matrix;
    InnovationCov = InnovationCov * error_covariance_matrix;
    InnovationCov = InnovationCov * measurement_to_state_matrix.transpose();
    InnovationCov = InnovationCov + measurement_noise_matrix;

    Eigen::MatrixXd KalmanGain = error_covariance_matrix * measurement_noise_matrix.transpose() * InnovationCov.inverse();

    state_vector = (Eigen::MatrixXd::Identity(6, 6) - KalmanGain * measurement_to_state_matrix) * state_vector + KalmanGain * measurement_vector;
    error_covariance_matrix = (Eigen::MatrixXd::Identity(6, 6) - KalmanGain * measurement_to_state_matrix) * error_covariance_matrix;
}

void Kalman::predict()
{
    // Predict state: X = F * X + B * u
    state_vector = state_transition_matrix * state_vector + control_input_matrix * measurement_vector;
    // std::cout << "STATE VECTOR: " << state_vector << std::endl;
    // std::cout << "----------------------------------------" << std::endl;
    // std::cout << std::setprecision(5) << state_transition_matrix << std::endl;
    // Predict covariance: P = F * P * F^T + Q
    error_covariance_matrix = state_transition_matrix * error_covariance_matrix * state_transition_matrix.transpose() + process_error_matrix; // * dt;
}

void Kalman::send_result()
{

    std::stringstream ss;
    ss << std::fixed << std::setprecision(15)
       << state_vector(0) << " " << state_vector(1) << " " << state_vector(2);
    std::string estimation = ss.str();

    parser.print_data("ESTIMATION", state_vector.segment<3>(0));
    sent_predictions_mutex.lock();
    sent_predictions++;
    sent_predictions_mutex.unlock(); // this is now counting loops not predictions for thread testing
    client.send_estimation(estimation);
}

/* TODO:
    Current idea:
        - Implement the Kalman filter specific to measurements
            x get rid of large measurement to state matrix
            - instead implement measurement specific matrices that are produced on the spot
            x The idea is to have the loop continuously predict and update, when data comes in
            x Eg we get a new acceleration measurement. We do the prediction as always
              We then update the state vector with the new acceleration and the error covariance matrix
              and send the new estimation back to the server and do that for all incomming data.
        - Datatransfer:
            - Need to confirm how data comes in. Are packages dropped, if we are not ready to receive?
        x I don't think we need the control input matrix. We could keep it in and set it to 0. If we do the initiailisation more
          modularly, this whole thing could be more flexible for whatever future use.
*/

bool Kalman::filter_loop()
{
    int sock_fd = client.get_sock_fd();
    struct timeval timeout;
    fd_set sock_fds;

    const int timeout_duration_sec = 1;
    time_t last_activity = time(nullptr);

    FD_ZERO(&sock_fds);
    FD_SET(sock_fd, &sock_fds);

    predict();
    send_result();

    while (true)
    {
        timeout.tv_sec = timeout_duration_sec;
        timeout.tv_usec = 0;

        int activity = select(sock_fd + 1, &sock_fds, nullptr, nullptr, &timeout);

        if (activity > 0)
        {
            parser.read_data(sock_fd);
            set_control_input_vector(parser);
            predict();
            if (parser.position_set)
            {
                update();
            }
            send_result();
        }
        else if (activity == 0)
        {
            time_t current_time = time(nullptr);
            if (current_time - last_activity >= timeout_duration_sec)
            {
                std::cout << "Client disconnected due to inactivity." << std::endl;
                close(sock_fd);
                kalman_error = true;
                return false;
            }
        }
        else
        {
            std::cerr << "Error with select()." << std::endl;
            close(sock_fd);
            kalman_error = true;
            return false;
        }
    }
    return true;
}
int Kalman::get_sent_predictions()
{
    return sent_predictions;
};

bool Kalman::get_kalman_error()
{
    return kalman_error;
}

Kalman::Kalman(int port, std::string handshake) : client(port),
                                                  parser(),
                                                  state_vector(6),
                                                  state_transition_matrix(6, 6),
                                                  error_covariance_matrix(6, 6),
                                                  process_error_matrix(6, 6),
                                                  control_input_matrix(6, 6),
                                                  control_input_vector(6),
                                                  measurement_to_state_matrix(6, 3),
                                                  measurement_vector(6),
                                                  measurement_noise_matrix(6, 6)
{
    client.send_handshake(handshake);
    int sock_fd = client.get_sock_fd();
    parser.read_data(sock_fd);
    sent_predictions = 0;
    set_state_vector();
    set_state_transition_matrix();
    set_control_input_matrix();
    set_error_covariance_matrix();
    set_process_error_matrix();
    set_measurement_to_state_matrix();
    set_measurement_noise_matrix();
    kalman_error = false;
    measurement_vector.setZero();
    measurement_vector.segment<3>(0) = Eigen::Vector3d({1, 1, 1});
}

Kalman::~Kalman() {};
