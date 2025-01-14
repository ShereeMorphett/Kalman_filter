#include "Server.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sstream>
#include <vector>
#include <iomanip>
#include "colour.hpp"







void Server::parse_data(std::string str_buffer)
{
    std::istringstream stream(str_buffer);
    std::string line;
    std::vector<double> values;

    std::cout << "BUFFER: " << str_buffer << std::endl;

    while (std::getline(stream, line))
    {
        if (line.find("TRUE POSITION") != std::string::npos)
        {
            for (int i = 0; i < 3; ++i)
            {
                if (std::getline(stream, line))
                {
                    try
                    {
                        // Convert the line to a double and store it
                        values.push_back(std::stod(line));
                    }
                    catch (const std::invalid_argument&)
                    {
                        std::cerr << "[server] ERROR: Invalid number format in TRUE POSITION" << std::endl;
                        return;
                    }
                }
                else
                {
                    std::cerr << "[server] ERROR: Not enough data for TRUE POSITION!" << std::endl;
                    return;
                }
            }

            if (values.size() == 3)
            {
                std::string true_position_str =
                    std::to_string(values[0]) + " " +
                    std::to_string(values[1]) + " " +
                    std::to_string(values[2]);

                // std::cout << std::fixed << std::setprecision(15)
                //     << "[server] TRUE POSITION: " << true_position_str << std::endl;

                client.send_estimation(true_position_str);

            }
            else
            {
                std::cout << "[server] ERROR: Invalid TRUE POSITION format!" << std::endl;
            }

            values.clear();
        }
    }
}

void Server::loop()
{
    socklen_t len = client.get_sock_len();
    sockaddr_in servaddr = client.get_servaddr();
    std::string str_buffer = "";
    int sock_fd = client.get_sock_fd();
    char buffer[MAXLINE];
    struct timeval timeout;
    fd_set sock_fds;

    FD_ZERO(&sock_fds);
    FD_SET(sock_fd, &sock_fds);

    const int timeout_duration_sec = 4; // Timeout duration in seconds
    time_t last_activity = time(nullptr); // Record the last activity timestamp

    while (true)
    {
        timeout.tv_sec = timeout_duration_sec;
        timeout.tv_usec = 0;

        int activity = select(sock_fd + 1, &sock_fds, nullptr, nullptr, &timeout);
        std::cout << COLOR_BRIGHT_CYAN << "[SERVER- DEBUG] "
            << "Activity: " << activity << COLOR_RESET << std::endl;

        if (activity > 0)
        {
            if (FD_ISSET(sock_fd, &sock_fds))
            {
                int buff_len = recvfrom(sock_fd, buffer, MAXLINE, 0, NULL, NULL);
                if (buff_len < 0)
                {
                    std::cerr << "Error receiving message." << std::endl;
                    close(sock_fd);
                    return;
                }
                if (buff_len == 0)
                {
                    std::cout << "Client disconnected." << std::endl;
                    close(sock_fd);
                    return;
                }

                buffer[buff_len] = '\0';
                parse_data(buffer);
                last_activity = time(nullptr);
            }
        }
        else if (activity == 0) // Timeout occurred
        {
            time_t current_time = time(nullptr);
            if (current_time - last_activity >= timeout_duration_sec)
            {
                std::cout << "Client disconnected due to inactivity." << std::endl;
                close(sock_fd);
                return;
            }
        }
        else
        {
            std::cerr << "Error with select()." << std::endl;
            close(sock_fd);
            return;
        }
    }
}



Server::Server(int port, std::string handshake) : client(port)
{
    client.send_handshake(handshake);

}

Server::~Server() {};
