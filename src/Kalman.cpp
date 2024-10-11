#include "Kalman.hpp"
#include <iomanip>

static constexpr double wn_accelerometer = 1e-3;
static constexpr double wn_gyroscope = 1e-2;
static constexpr double wn_gps = 1e-1;

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
                speed = std::stod(line);
                std::cout << "[server] SPEED: " << speed << " km/h" << std::endl;
            }
        }
        else if (line.find("ACCELERATION") != std::string::npos)
        {
            Eigen::Vector3d accel = parse_eigen_vec3(stream);
            X.segment<3>(3) = accel; // Set the velocity part of the state vector (X), or use it to update velocity, this will need a function to work out the velocity
            std::cout << "[server] ACCELERATION: "
                      << accel(0) << ", " << accel(1) << ", " << accel(2) << std::endl;
        }
        else if (line.find("DIRECTION") != std::string::npos)
        {
            Eigen::Matrix<double, 3, 1> direction = parse_eigen_vec3(stream);
            std::cout << "[server] DIRECTION: "
                      << direction(0) << ", " << direction(1) << ", " << direction(2) << std::endl;
        }
    }
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

    while (true)
    {
        int buff_len = recvfrom(sock_fd, buffer, MAXLINE, MSG_WAITALL,
                                reinterpret_cast<struct sockaddr *>(&servaddr), &len);
        buffer[buff_len] = '\0';
        std::cout << buffer << std::endl;
        std::string str_buffer = buffer;
        parse_data(buffer);
        if (X.size() != 0)
            Eigen::Vector3d estimation = calculate_estimation();
    }
}

Kalman::Kalman(int port, std::string handshake) : client(port), acceleration(0, 0, 0), F(6, 6), B(6, 3), H(3, 6), Q(6, 6), R(3, 3), P(6, 6), X(6), Z(3)
{
    client.send_handshake(handshake);
}

Kalman::~Kalman() {};
