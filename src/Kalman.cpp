#include "Kalman.hpp"
#include <iomanip>
#include <regex>

static constexpr double variance_accelerometer = 1e-9; // 1e-3 * 1e-3
static constexpr double variance_gyroscope = 1e-4;     // 1e-2 * 1e-2
static constexpr double variance_gps = 1e-2;           // 1e-1 * 1e-1

Eigen::Vector3d parse_eigen_vec3(std::istringstream &data)
{
    double x, y, z;
    std::string x_str, y_str, z_str;

    std::getline(data, x_str);
    std::getline(data, y_str);
    std::getline(data, z_str);

    std::stringstream(x_str) >> x;
    std::stringstream(y_str) >> y;
    std::stringstream(z_str) >> z;

    Eigen::Vector3d point;
    point << x, y, z;

    return point;
}

// void Kalman::print_matrices()
// {
//     // Print matrices and vectors
//     std::cout << "F (State transition matrix):\n"
//               << F << "\n\n";
//     std::cout << "B (Control input matrix):\n"
//               << B << "\n\n";
//     std::cout << "H (Measurement matrix):\n"
//               << H << "\n\n";
//     std::cout << "Q (Process noise covariance matrix):\n"
//               << Q << "\n\n";
//     std::cout << "R (Measurement noise covariance matrix):\n"
//               << R << "\n\n";
//     std::cout << "P (Error covariance matrix):\n"
//               << P << "\n\n";
//     std::cout << "X (State vector):\n"
//               << X << "\n\n";
//     std::cout << "Z (Measurement vector):\n"
//               << Z << "\n\n";
// }
// Eigen::MatrixXd F; // State transition matrix
// Eigen::MatrixXd B; // Control input matrix (acceleration)
// Eigen::MatrixXd H; // Measurement matrix
// Eigen::MatrixXd Q; // Process noise covariance matrix
// Eigen::MatrixXd R; // Measurement noise covariance matrix
// Eigen::MatrixXd P; // Error covariance matrix
// Eigen::VectorXd X; // State vector (position, velocity) X(0): Position x. X(1): Position y.  X(2): Position  z. X(3): Velocity  x. X(4): Velocity  y.  X(5): Velocity  z.
// Eigen::VectorXd Z; // Measurement vector (GPS position)

Kalman::MeasurementData Kalman::parse_measurement(std::string str_buffer)
{
    std::istringstream stream(str_buffer);
    std::string line;
    std::regex capital_regex("[A-Z]+");
    std::smatch match;
    MeasurementData data;

    while (std::getline(stream, line))
    {
        if (std::regex_search(line, match, capital_regex))
        {
            std::string capital_letters = match.str(0);
            std::cout << "[server] " << capital_letters << ":" << std::endl;
            data.values = parse_eigen_vec3(stream);
            std::cout << data.values(0) << ", " << data.values(1) << ", " << data.values(2) << std::endl;
            data.type = type_map[capital_letters];
        }
    }
    return data;
}

Kalman::MeasurementData Kalman::parse_data(std::string str_buffer)
{
    std::string line;
    std::istringstream stream(str_buffer);
    MeasurementData data;
    while (std::getline(stream, line))
    {
        data = parse_measurement(str_buffer);

        switch (data.type)
        {
        case Type::TruePosition:
            StateVector.segment<3>(0) = data.values;
            break;
        case Type::Velocity:
            data.values *= 0.277778; // convert km/h to m/s
            StateVector.segment<3>(3) = data.values;
            break;
        case Type::Direction:
            data.values = get_body_to_inertial_rotation(data.values);
        default:
            break;
        }
    }
    return data;
}

void Kalman::update(Eigen::Vector3d measurement, Eigen::MatrixXd MeasurementToStateMatrix)
{
    Eigen::MatrixXd InnovationCov = MeasurementToStateMatrix * ErrorCovarianceMatrix * MeasurementToStateMatrix.transpose() + MeasurementNoiseMatrix;
    Eigen::MatrixXd KalmanGain = ErrorCovarianceMatrix * MeasurementNoiseMatrix.transpose() * InnovationCov.inverse();

    StateVector = (Eigen::MatrixXd::Identity(12, 12) - KalmanGain * MeasurementToStateMatrix) * StateVector + KalmanGain * MeasurementVector;
    ErrorCovarianceMatrix = (Eigen::MatrixXd::Identity(12, 12) - KalmanGain * MeasurementToStateMatrix) * ErrorCovarianceMatrix;
}

void Kalman::predict(double dt)
{

    // Update the state transition matrix F with the current time step
    StateTransitionMatrix.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity(3, 3) * dt;
    StateTransitionMatrix.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity(3, 3) * dt;

    // Predict state: X = F * X
    StateVector = StateTransitionMatrix * StateVector;
    // Predict covariance: P = F * P * F^T + Q
    ErrorCovarianceMatrix = StateTransitionMatrix * ErrorCovarianceMatrix * StateTransitionMatrix.transpose() + ProcessErrorMatrix; //*dt?
}

Eigen::Vector3d Kalman::send_result()
{

    std::stringstream ss;
    Eigen::Vector3d last_position = StateVector.segment<3>(0);
    ss << std::fixed << std::setprecision(15)
       << StateVector(0) << " " << StateVector(1) << " " << StateVector(2);
    std::string estimation = ss.str();
    std::cout << std::fixed << std::setprecision(15) << "ESTIMATION SENT:  " << estimation << std::endl;

    client.send_estimation(estimation);

    return last_position;
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

// could be a static function/ we don't need to create them everytime. Could be another lookup table
Eigen::MatrixXd Kalman::get_mts_matrix(Type type)
{
    Eigen::MatrixXd MeasurementToStateMatrix = Eigen::MatrixXd::Zero(12, 3);
    switch (type)
    {
    case Type::Direction:
        MeasurementToStateMatrix.block<3, 3>(6, 0) = Eigen::Matrix3d::Identity(3, 3);
        break;
    case Type::Acceleration:
        MeasurementToStateMatrix.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity(3, 3);
        break;
    case Type::Position:
        MeasurementToStateMatrix.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(3, 3);
        // include velocity correction
        break;
    default:
        break;
    }
    return MeasurementToStateMatrix;
}

double Kalman::get_dt()
{
    auto current_time = std::chrono::steady_clock::now();
    double dt = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_update).count() / 1000.0; // in seconds
    return dt;
}

void Kalman::filter_loop()
{
    int sock_fd = client.get_sock_fd();
    sockaddr_in servaddr = client.get_servaddr();
    socklen_t len = client.get_sock_len();
    MeasurementData data;
    while (true)
    {
        double dt = get_dt();
        predict(dt);
        // TODO: check if we can safely return to recvfrom if we're not waiting for data

        int buff_len = recvfrom(sock_fd, buffer, MAXLINE, MSG_WAITALL,
                                reinterpret_cast<struct sockaddr *>(&servaddr), &len);
        if (buff_len == -1)
        {
            std::cerr << "Error receiving data" << std::endl;
            continue;
        }
        buffer[buff_len] = '\0';
        std::string str_buffer = buffer;
        data = parse_data(buffer);
        if (StateVector.size() != 0 && initalized)
        {
            Eigen::MatrixXd MeasurementToStateMatrix = get_mts_matrix(data.type);
            update(data.values, MeasurementToStateMatrix);
            Eigen::Vector3d estimation = send_result();
            (void)estimation;
        }
    }
}

void Kalman::filter_loop()
{
    struct timeval timeout;
    fd_set sock_fds;
    int sock_fd = client.get_sock_fd();
    sockaddr_in servaddr = client.get_servaddr();
    socklen_t len = client.get_sock_len();
    MeasurementData data;
    double dt;

    MeasurementData data;
    int buff_len;
    while (true)
    {
        FD_ZERO(&sock_fds);
        FD_SET(sock_fd, &sock_fds);

        // Set timeout to 0 for non-blocking check
        timeout.tv_sec = 0;
        timeout.tv_usec = 0;

        int activity = select(sock_fd + 1, &sock_fds, nullptr, nullptr, &timeout);

        if (activity > 0 && FD_ISSET(sock_fd, &sock_fds))
        {
            // Data is available to read
            buff_len = recvfrom(sock_fd, buffer, MAXLINE, MSG_WAITALL,
                                reinterpret_cast<struct sockaddr *>(&servaddr), &len);
            if (buff_len < 0)
            {
                std::cout << "Error receiving message" << std::endl;
            }
            if (buff_len == 0)
            {
                std::cout << "Connection closed" << std::endl;
                close(sock_fd);
                break;
            }
            buffer[buff_len] = '\0';
            std::string str_buffer = buffer;
            data = parse_data(buffer);
            if (StateVector.size() != 0 && initalized)
            {
                Eigen::MatrixXd MeasurementToStateMatrix = get_mts_matrix(data.type);
                update(data.values, MeasurementToStateMatrix);
                Eigen::Vector3d estimation = send_result();
                (void)estimation;
            }
        }
        // Perform the prediction step or other work
        dt = get_dt();
        predict(dt);
        send_result();
    }
}

Eigen::MatrixXd Kalman::get_body_to_inertial_rotation(Eigen::Vector3d angles)
{
    double sin_roll = sin(angles(0));
    double cos_roll = cos(angles(0));
    double sin_pitch = sin(angles(1));
    double cos_pitch = cos(angles(1));
    double sin_yaw = sin(angles(2));
    double cos_yaw = cos(angles(2));

    Eigen::Matrix3d r_roll{{1, 0, 0},
                           {0, cos_roll, sin_roll},
                           {0, -sin_roll, cos_roll}};

    Eigen::Matrix3d r_pitch{{cos_pitch, 0, -sin_pitch},
                            {0, 1, 0},
                            {sin_pitch, 0, cos_pitch}};

    Eigen::Matrix3d r_yaw{{cos_yaw, sin_yaw, 0},
                          {-sin_yaw, cos_yaw, 0},
                          {0, 0, 1}};

    /*
        As of now, I am faily confident we are in a rh coordinate system,
        and we might be dealing with a 1-2-3 rotation sequence.
        As in first roll, then pitch, then yaw. Which is not terribly common.
        Based on the description in the subject, we should have actual euler angles
        and not rates of change. So we should be able to use the above matrices.
    */
    return r_yaw * r_pitch * r_roll;
}

void Kalman::update_state_transition_matrix(double dt)
{
    // updates velocity in position and acceleration in velocity
    StateTransitionMatrix.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity(3, 3) * dt;
    StateTransitionMatrix.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity(3, 3) * dt;

    // updates acceleration in position
    StateTransitionMatrix.block<3, 3>(6, 0) = Eigen::Matrix3d::Identity(3, 3) * 0.5 * dt * dt;

    // update rotation in acceleration
    StateTransitionMatrix.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity(3, 3) * StateVector.segment(9, 3);
}

void Kalman::get_state_transition_matrix()
{
    double initial_dt = 0.1;
    StateTransitionMatrix = Eigen::MatrixXd::Identity(12, 12);
    update_state_transition_matrix(initial_dt);
}

void Kalman::get_process_error_matrix()
{
    ProcessErrorMatrix = Eigen::MatrixXd::Identity(12, 12);
    double dt = 0.1;

    double noise_acceleration = variance_accelerometer + 0.5 * variance_gyroscope * dt; // possibly (0.5*1/3)
    double noise_velocity = noise_acceleration * dt;
    double noise_position = variance_gps + noise_velocity + 0.5 * noise_acceleration * dt * dt;

    ProcessErrorMatrix.block<3, 3>(0, 0) *= noise_position;
    ProcessErrorMatrix.block<3, 3>(3, 3) *= noise_velocity;
    ProcessErrorMatrix.block<3, 3>(6, 6) *= noise_acceleration;
    ProcessErrorMatrix.block<3, 3>(9, 9) *= variance_gyroscope;
    // integrate (Riemansum)
}

Kalman::Kalman(int port, std::string handshake) : client(port),
                                                  StateTransitionMatrix(12, 12),
                                                  ProcessErrorMatrix(12, 12),
                                                  MeasurementNoiseMatrix(9, 9),
                                                  ErrorCovarianceMatrix(12, 12),
                                                  StateVector(12),
                                                  MeasurementVector(3),
                                                  initalized(false)
{
    client.send_handshake(handshake);

    socklen_t len = client.get_sock_len();
    for (int i = 0; i < 7; i++)
    {
        char buffer[MAXLINE];
        int buff_len = recvfrom(client.get_sock_fd(), buffer, MAXLINE, MSG_WAITALL,
                                reinterpret_cast<struct sockaddr *>(&client.get_servaddr()), &len);
        if (buff_len < 0)
        {
            std::cerr << "Error receiving message" << std::endl;
            exit(1);
        }
        MeasurementData data = parse_data(buffer);
    }
    initalized = true;

    get_state_transition_matrix();
    get_process_error_matrix();

    ErrorCovarianceMatrix.setIdentity();

    MeasurementNoiseMatrix.block<3, 3>(0, 0) *= variance_gps;
    MeasurementNoiseMatrix.block<3, 3>(3, 3) *= variance_accelerometer;
    MeasurementNoiseMatrix.block<3, 3>(6, 6) *= variance_gyroscope;

    last_update = std::chrono::steady_clock::now();
}

Kalman::~Kalman() {};
