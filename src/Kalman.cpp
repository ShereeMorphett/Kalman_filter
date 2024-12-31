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
            // need to figure out transormation from euler angle to quaternion?
            break;
        default:
            break;
        }
        if (!initalized) // TODO: currently happens possible too early
        {
            initalized = true;
            // print_matrices();
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
    ErrorCovarianceMatrix = StateTransitionMatrix * ErrorCovarianceMatrix * StateTransitionMatrix.transpose() + ProcessErrorMatrix;
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
            - Do packages come individually or potentially in batch, like the first contact?
        - I don't think we need the control input matrix. We could keep it in and set it to 0. If we do the initiailisation more
          modularly, this whole thing could be more flexible for whatever future use.
*/

// could be a static function/ we don't need to create them everytime. Could be another lookup table
void Kalman::get_mts_matrix(Type type)
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

double Kalman::get_time()
{
    auto current_time = std::chrono::steady_clock::now();
    double dt = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_update).count() / 1000.0; // in seconds
    last_update = current_time;
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
        int buff_len = recvfrom(sock_fd, buffer, MAXLINE, MSG_WAITALL,
                                reinterpret_cast<struct sockaddr *>(&servaddr), &len);
        buffer[buff_len] = '\0';
        std::string str_buffer = buffer;
        data = parse_data(buffer);
        double dt = get_time();
        Eigen::MatrixXd MeasurementToStateMatrix = get_mts_matrix(data.type);
        // after parsing calculate the measurement to state matrix depending on the data.
        if (StateVector.size() != 0 && initalized)
        {
            // Always execute both, when data comes in
            // Need to decide, whether we want to continuously predict and report or only when measurement corrected report.
            predict(dt);
            update(data.values, MeasurementToStateMatrix); // needs to take matrix
            Eigen::Vector3d estimation = send_result();
            (void)estimation;
        }
    }
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

    StateTransitionMatrix.setIdentity();
    StateTransitionMatrix.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity(3, 3);
    StateTransitionMatrix.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity(3, 3);
    // TODO: Influence of orientation is missing from the state transition matrix

    ProcessErrorMatrix.setIdentity() * 1e-3;

    ErrorCovarianceMatrix.setZero();

    MeasurementNoiseMatrix.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3) * variance_gps;
    MeasurementNoiseMatrix.block<3, 3>(3, 3) = Eigen::MatrixXd::Identity(3, 3) * variance_accelerometer;
    MeasurementNoiseMatrix.block<3, 3>(6, 6) = Eigen::MatrixXd::Identity(3, 3) * variance_gyroscope;

    last_update = std::chrono::steady_clock::now();
}

Kalman::~Kalman() {};
