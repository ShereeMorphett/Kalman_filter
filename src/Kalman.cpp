#include "Kalman.hpp"

void Kalman::parse_data(std::string str_buffer)
{
    std::istringstream stream(str_buffer);
    std::string line;
    
    while (std::getline(stream, line))
    {
        if (line.find("TRUE POSITION") != std::string::npos)
        {
            std::getline(stream, line);
            double x, y, z;
            if (sscanf(line.c_str(), "%lf %lf %lf", &x, &y, &z) == 3)
            {
                vec3<double> position = {x, y, z};
                std::cout << " server TRUE POSITION: "
                          << position.x << ", " << position.y << ", " << position.z << std::endl;
                data.push_back(position);
            }
            else
            {
                std::cerr << "Error parsing TRUE POSITION" << std::endl;
            }
        }
        else if (line.find("SPEED") != std::string::npos)
        {
            std::getline(stream, line);
            if (sscanf(line.c_str(), "%lf", &speed) == 1)
            {
                std::cout << " server SPEED: " << speed << " km/h" << std::endl;
            }
            else
            {
                std::cerr << "Error parsing SPEED" << std::endl;
            }
        }
        else if (line.find("ACCELERATION") != std::string::npos)
        {
            std::getline(stream, line);
            double ax, ay, az;
            if (sscanf(line.c_str(), "%lf %lf %lf", &ax, &ay, &az) == 3)
            {
                acceleration = {ax, ay, az};
                std::cout << " server ACCELERATION: "
                          << acceleration.x << ", " << acceleration.y << ", " << acceleration.z << std::endl;
            }
            else
            {
                std::cerr << "Error parsing ACCELERATION" << std::endl;
            }
        }
        else if (line.find("DIRECTION") != std::string::npos)
        {
            std::getline(stream, line);
            double dx, dy, dz;
            if (sscanf(line.c_str(), "%lf %lf %lf", &dx, &dy, &dz) == 3)
            {
                direction = {dx, dy, dz};
                std::cout << " server DIRECTION: "
                          << direction.x << ", " << direction.y << ", " << direction.z << std::endl;
            }
            else
            {
                std::cerr << "Error parsing DIRECTION" << std::endl;
            }
        }
    }
}

vec3<double> Kalman::calculate_estimation()
{
    vec3<double> last_position = data.back();

    std::stringstream ss;
    ss << last_position.x << " " << last_position.y << " " << last_position.z;

    std::string estimation = ss.str();
    std::cout << estimation << std::endl;

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
        vec3<double> estimation = calculate_estimation();
    }
}

Kalman::Kalman(int port, std::string handshake) : client(port)
{
    client.send_handshake(handshake);
}

Kalman::~Kalman() {};
