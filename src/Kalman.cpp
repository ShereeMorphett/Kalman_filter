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

void Kalman::print_matrice()
{
    // Print matrices and vectors
    std::cout << "F (State transition matrix):\n"
              << F << "\n\n";
    std::cout << "B (Control input matrix):\n"
              << B << "\n\n";
    std::cout << "H (Measurement matrix):\n"
              << H << "\n\n";
    std::cout << "Q (Process noise covariance matrix):\n"
              << Q << "\n\n";
    std::cout << "R (Measurement noise covariance matrix):\n"
              << R << "\n\n";
    std::cout << "P (Error covariance matrix):\n"
              << P << "\n\n";
    std::cout << "X (State vector):\n"
              << X << "\n\n";
    std::cout << "Z (Measurement vector):\n"
              << Z << "\n\n";
}
// Eigen::MatrixXd F; // State transition matrix
// Eigen::MatrixXd B; // Control input matrix (acceleration)
// Eigen::MatrixXd H; // Measurement matrix
// Eigen::MatrixXd Q; // Process noise covariance matrix
// Eigen::MatrixXd R; // Measurement noise covariance matrix
// Eigen::MatrixXd P; // Error covariance matrix
// Eigen::VectorXd X; // State vector (position, velocity) X(0): Position x. X(1): Position y.  X(2): Position  z. X(3): Velocity  x. X(4): Velocity  y.  X(5): Velocity  z.
// Eigen::VectorXd Z; // Measurement vector (GPS position)

void Kalman::parse_data(std::string str_buffer)
{
    std::istringstream stream(str_buffer);
    std::string line;
    while (std::getline(stream, line))
    {
        if (line.find("TRUE POSITION") != std::string::npos)
        {
            Eigen::Vector3d position = parse_eigen_vec3(stream);
            X.segment<3>(0) = position;
            std::cout << std::fixed << std::setprecision(15)
                      << "[server] TRUE POSITION: "
                      << position(0) << ", " << position(1) << ", " << position(2) << std::endl;
        }
        else if (line.find("SPEED") != std::string::npos)
        {
            std::getline(stream, line);
            {
                speed = 0.277778 * (std::stod(line)); // convert km/h to m/s
                std::cout << "[server] SPEED: " << line << " km/h" << std::endl;
                std::cout << "[server] SPEED: " << speed << " m/s" << std::endl;
                std::cout << "[server] UPDATED" << std::endl;
            }
        }
        else if (line.find("ACCELERATION") != std::string::npos)
        {
            Eigen::Vector3d accel = parse_eigen_vec3(stream);

            acceleration = accel;

            X = F * X + B * acceleration;

            std::cout << "[server] ACCELERATION: "
                      << accel(0) << ", " << accel(1) << ", " << accel(2) << std::endl;
        }
        else if (line.find("DIRECTION") != std::string::npos)
        {
            Eigen::Matrix<double, 3, 1> direction = parse_eigen_vec3(stream);
            std::cout << "[server] DIRECTION: "
                      << direction(0) << ", " << direction(1) << ", " << direction(2) << std::endl;
            std::cout << "[server] UPDATED" << std::endl;

            if (!initalized)
            {
                initalized = true;
                print_matrice();
            }
        }
    }
}

void Kalman::update()
{
    Eigen::Vector3d Y = Z - H * X;
    Eigen::MatrixXd S = H * P * H.transpose() + R;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();

    X = X + K * Y;
    P = (Eigen::MatrixXd::Identity(6, 6) - K * H) * P;
}

void Kalman::predict(double dt)
{
    // Update the state transition matrix F with the current time step
    F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;

    // Predict state: X = F * X + B * U (where U is the acceleration)
    X = F * X + B * acceleration;

    // Predict covariance: P = F * P * F^T + Q
    P = F * P * F.transpose() + Q;
}

Eigen::Vector3d Kalman::calculate_estimation()
{

    std::stringstream ss;
    Eigen::Vector3d last_position = X.segment<3>(0);
    ss << std::fixed << std::setprecision(15)
       << X(0) << " " << X(1) << " " << X(2);
    std::string estimation = ss.str();
    std::cout << std::fixed << std::setprecision(15) << "ESTIMATION SENT:  " << estimation << std::endl;

    client.send_estimation(estimation);

    return last_position;
}

void Kalman::filter_loop()
{
    int sock_fd = client.get_sock_fd();
    sockaddr_in servaddr = client.get_servaddr();
    socklen_t len = client.get_sock_len();
    auto current_time = std::chrono::steady_clock::now();
    double dt = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_update).count() / 1000.0; // in seconds
    last_update = current_time;
    while (true)
    {
        int buff_len = recvfrom(sock_fd, buffer, MAXLINE, MSG_WAITALL,
                                reinterpret_cast<struct sockaddr *>(&servaddr), &len);
        buffer[buff_len] = '\0';
        std::string str_buffer = buffer;
        parse_data(buffer);
        if (X.size() != 0 && initalized)
        {
            predict(dt);
            update();
            Eigen::Vector3d estimation = calculate_estimation();
        }
    }
}

Kalman::Kalman(int port, std::string handshake) : client(port), acceleration(0, 0, 0), F(6, 6), B(6, 3), H(3, 6), Q(6, 6), R(3, 3), P(6, 6), X(6), Z(3), initalized(false)
{
    client.send_handshake(handshake);
    F.setIdentity(6, 6);
    F.block<3, 3>(0, 3).setIdentity(); // Positions evolve into velocities

    B.setZero(6, 3);
    B.block<3, 3>(3, 0).setIdentity(); // Accelerations affect velocity

    Q.setIdentity(6, 6); // Process noise
    Q.block<3, 3>(0, 0) *= gauss_accelerometer;
    Q.block<3, 3>(3, 3) *= gauss_gyroscope;

    R.setIdentity(3, 3); // Measurement noise
    R *= gauss_gps * gauss_gps;

    P.setIdentity(6, 6) * 1e-3; // Initial covariance
    last_update = std::chrono::steady_clock::now();
}

Kalman::~Kalman() {};
