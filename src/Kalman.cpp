#include "Kalman.hpp"
#include <iomanip>
#include <regex>

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

Kalman::MeasurementData Kalman::parse_measurement(std::string str_buffer)
{
    std::istringstream stream(str_buffer);
    std::string line;
    std::cout << "[server OG OUTPUT] " << str_buffer << std::endl;
    MeasurementData data;
    std::string type = str_buffer.substr(0, str_buffer.find(":"));
    size_t i = 0;
    while (std::getline(stream, line))
    {
        // std::cout << "[server] " << type << ":" << std::endl;
        data.values = parse_eigen_vec3(stream);
        // std::cout << data.values(0) << ", " << data.values(1) << ", " << data.values(2) << std::endl;
        // find the right type for data
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
            // Velocity is scalar
            break;
        }
    }
    return data;
}

void Kalman::set_measurement_vector(MeasurementData &data)
{
    MeasurementVector.setZero();
    switch (data.type)
    {
    case Type::Position:
        MeasurementVector.segment<3>(0) = data.values;
        break;
    case Type::Acceleration:
        MeasurementVector.segment<3>(6) = data.values;
        break;
    case Type::Direction:
        MeasurementVector.segment<3>(9) = data.values;
        break;
    default:
        break;
    }
}

void Kalman::update()
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
    StateTransitionMatrix.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity(3, 3) * 0.5 * dt * dt;

    // Predict state: X = F * X
    StateVector = StateTransitionMatrix * StateVector;
    // Predict covariance: P = F * P * F^T + Q
    ErrorCovarianceMatrix = StateTransitionMatrix * ErrorCovarianceMatrix * StateTransitionMatrix.transpose() + ProcessErrorMatrix; // * dt;
}

void Kalman::send_result()
{

    std::stringstream ss;
    ss << std::fixed << std::setprecision(15)
       << StateVector(0) << " " << StateVector(1) << " " << StateVector(2);
    std::string estimation = ss.str();
    std::cout << std::fixed << std::setprecision(15) << "ESTIMATION SENT:  " << estimation << std::endl;

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

double Kalman::get_dt()
{
    auto current_time = std::chrono::steady_clock::now();
    double dt = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_update).count() / 1000.0; // in seconds
    return dt;
}

// void Kalman::filter_loop()
// {
//     int sock_fd = client.get_sock_fd();
//     sockaddr_in servaddr = client.get_servaddr();
//     socklen_t len = client.get_sock_len();
//     MeasurementData data;
//     while (true)
//     {
//         double dt = get_dt();
//         predict(dt);
//         // TODO: check if we can safely return to recvfrom if we're not waiting for data

//         int buff_len = recvfrom(sock_fd, buffer, MAXLINE, MSG_WAITALL,
//                                 reinterpret_cast<struct sockaddr *>(&servaddr), &len);
//         if (buff_len == -1)
//         {
//             std::cerr << "Error receiving data" << std::endl;
//             continue;
//         }
//         buffer[buff_len] = '\0';
//         std::string str_buffer = buffer;
//         data = parse_data(buffer);
//         if (StateVector.size() != 0 && initalized)
//         {
//             Eigen::MatrixXd MeasurementToStateMatrix = get_mts_matrix(data.type);
//             update(data.values, MeasurementToStateMatrix);
//             Eigen::Vector3d estimation = send_result();
//             (void)estimation;
//         }
//     }
// }

void Kalman::filter_loop()
{
    struct timeval timeout = {0};
    fd_set sock_fds;
    int sock_fd = client.get_sock_fd();
    sockaddr_in servaddr = client.get_servaddr();
    socklen_t len = client.get_sock_len();
    MeasurementData data;
    double dt;
    int buff_len;
    FD_ZERO(&sock_fds);
    FD_SET(sock_fd, &sock_fds);

    while (true)
    {
        int activity = select(sock_fd + 1, &sock_fds, nullptr, nullptr, &timeout);
        dt = get_dt();
        predict(dt);
        if (activity > 0 && FD_ISSET(sock_fd, &sock_fds))
        {
            buff_len = recvfrom(sock_fd, buffer, MAXLINE, MSG_WAITALL,
                                reinterpret_cast<struct sockaddr *>(&servaddr), &len);
            if (buff_len < 0)
            {
                std::cout << "Error receiving message" << std::endl;
                close(sock_fd);
                exit(1);
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
                set_measurement_vector(data);
                update();
                last_update = std::chrono::steady_clock::now();
            }
        }
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
        As of now, I am fairly confident we are in a rh coordinate system,
        and we might be dealing with a 1-2-3 rotation sequence.
        As in first roll, then pitch, then yaw. Which is not terribly common. Could be first point of debug
        More common would be 3-2-1, which is yaw, pitch, roll (read from right to left).
        Based on the description in the subject, we should have actual euler angles
        and not rates of change. So we should be able to use the above matrices.
    */
    return r_yaw * r_pitch * r_roll; // Possibly switch yaw and roll
}

/*
    StateVector
    As row vector:
        [x, y, z, vx, vy, vz, ax, ay, az, roll, pitch, yaw]

*/

/*
    p_k = p_(k-1) + v_(k-1)*dt + 0.5*a_(k-1)*dt^2
    v_k = v_(k-1) + a_(k-1)*dt
    a_k = a_(k-1) * R_(k-1)
*/

/*
    StateMatrix:
     1 0 0 dt 0 0 0.5*dt^2 0 0 0 0 0
     0 1 0 0 dt 0 0 0.5*dt^2 0 0 0 0
     0 0 1 0 0 dt 0 0 0.5*dt^2 0 0 0
     0 0 0 1 0 0 dt 0 0 0 0 0
     0 0 0 0 1 0 0 dt 0 0 0 0
     0 0 0 0 0 1 0 0 dt 0 0 0
     0 0 0 0 0 0 1 0 0 R_11 R_12 R_13
     0 0 0 0 0 0 0 1 0 R_21 R_22 R_23
     0 0 0 0 0 0 0 0 1 R_31 R_32 R_33
     0 0 0 0 0 0 0 0 0 1 0 0
     0 0 0 0 0 0 0 0 0 0 1 0
     0 0 0 0 0 0 0 0 0 0 0 1
     */

void Kalman::set_state_transition_matrix()
{
    double dt = 0.1;
    StateTransitionMatrix.setIdentity(12, 12);

    // updates velocity in position and acceleration in velocity
    StateTransitionMatrix.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    StateTransitionMatrix.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * dt;

    // updates acceleration in position
    StateTransitionMatrix.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * 0.5 * dt * dt;
    std::cout << StateTransitionMatrix << std::endl;
}

void Kalman::set_process_error_matrix()
{
    ProcessErrorMatrix.setIdentity(12, 12);
    double dt = 0.1;

    double noise_acceleration = variance_accelerometer + 0.5 * variance_gyroscope * dt; // possibly (0.5*1/3)
    double noise_velocity = noise_acceleration * dt;
    double noise_position = variance_gps + noise_velocity + 0.5 * noise_acceleration * dt * dt;

    ProcessErrorMatrix.block<3, 3>(0, 0) *= noise_position; // integrate (Riemansum)
    ProcessErrorMatrix.block<3, 3>(3, 3) *= noise_velocity;
    ProcessErrorMatrix.block<3, 3>(6, 6) *= noise_acceleration;
    ProcessErrorMatrix.block<3, 3>(9, 9) *= variance_gyroscope;
    std::cout << ProcessErrorMatrix << std::endl;
}

void Kalman::set_measurement_to_state_matrix()
{
    MeasurementToStateMatrix.setZero(12, 3);
    // GPS
    MeasurementToStateMatrix.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(3, 3);
    // Accelerometer
    MeasurementToStateMatrix.block<3, 3>(6, 0) = Eigen::Matrix3d::Identity(3, 3);
    // Gyroscope
    MeasurementToStateMatrix.block<3, 3>(9, 0) = Eigen::Matrix3d::Identity(3, 3);
    std::cout << MeasurementToStateMatrix << std::endl;
}

Kalman::Kalman(int port, std::string handshake) : client(port),
                                                  StateTransitionMatrix(12, 12),
                                                  ProcessErrorMatrix(12, 12),
                                                  MeasurementNoiseMatrix(9, 9),
                                                  MeasurementToStateMatrix(12, 3),
                                                  ErrorCovarianceMatrix(12, 12),
                                                  StateVector(12),
                                                  MeasurementVector(12),
                                                  initalized(false)
{
    client.send_handshake(handshake);

    socklen_t len = client.get_sock_len();
    sockaddr_in servaddr = client.get_servaddr();
    std::string str_buffer;

    for (int i = 0; i < 7; i++)
    {
        char buffer[MAXLINE];
        int buff_len = recvfrom(client.get_sock_fd(), buffer, MAXLINE, MSG_WAITALL,
                                reinterpret_cast<struct sockaddr *>(&servaddr), &len);
        if (buff_len < 0)
        {
            std::cerr << "Error receiving message" << std::endl;
            exit(1);
        }
        buffer[buff_len] = '\0';
        str_buffer = buffer;
        for (auto [key, value] : type_map)
        {
            if (str_buffer.find(key) != std::string::npos)
            {
                MeasurementData data = parse_data(str_buffer);
                (void)data;
                break;
            }
        }
    }
    initalized = true;

    std::cout << StateVector << std::endl;
    set_state_transition_matrix();
    set_process_error_matrix();

    ErrorCovarianceMatrix.setIdentity();

    MeasurementNoiseMatrix.block<3, 3>(0, 0) *= variance_gps;
    MeasurementNoiseMatrix.block<3, 3>(3, 3) *= variance_accelerometer;
    MeasurementNoiseMatrix.block<3, 3>(6, 6) *= variance_gyroscope;

    set_measurement_to_state_matrix();

    last_update = std::chrono::steady_clock::now();
}

Kalman::~Kalman() {};
