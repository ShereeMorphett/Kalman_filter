#include "Kalman.hpp"
#include "vec3.hpp"
#include <iomanip>

void Kalman::parse_data(std::string str_buffer)
{
    std::istringstream stream(str_buffer);
    std::string line;
    while (std::getline(stream, line))
    {
        if (line.find("TRUE POSITION") != std::string::npos)
        {
            std::cout << "BUFFER IN FULL:" << str_buffer << std::endl;

            vec3<double> position = parse_vec3<double>(stream);
            std::cout << std::fixed << std::setprecision(15)
                      << " server TRUE POSITION: "
                      << position.x << ", " << position.y << ", " << position.z << std::endl;

            data.push_back(position);
        }
        else if (line.find("SPEED") != std::string::npos)
        {
            std::getline(stream, line);
            {
                std::cout << " server SPEED: " << speed << " km/h" << std::endl;
            }
        }
        else if (line.find("ACCELERATION") != std::string::npos)
        {
            std::cout << "BUFFER IN FULL:" << str_buffer << std::endl;
            vec3<double> acceleration = parse_vec3<double>(stream);
            std::cout << " server ACCELERATION: "
                      << acceleration.x << ", " << acceleration.y << ", " << acceleration.z << std::endl;
        }
        else if (line.find("DIRECTION") != std::string::npos)
        {
            vec3<double> direction = parse_vec3<double>(stream);
            std::cout << " server DIRECTION: "
                      << direction.x << ", " << direction.y << ", " << direction.z << std::endl;
        }
    }
}

vec3<double> Kalman::calculate_estimation()
{
    vec3<double> last_position = data.back();

    std::stringstream ss;
    ss << std::fixed << std::setprecision(15)
       << last_position.x << " " << last_position.y << " " << last_position.z;
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
        if (data.size() != 0)
            vec3<double> estimation = calculate_estimation();
    }
}

Kalman::Kalman(int port, std::string handshake) : client(port)
{
    client.send_handshake(handshake);
}

Kalman::~Kalman() {};
