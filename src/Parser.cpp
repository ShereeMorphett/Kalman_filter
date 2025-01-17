#include "Parser.hpp"
#include <unistd.h>
#include <sys/socket.h>

inline constexpr int MAXLINE = 1024;

MeasurementData Parser::get_true_position() const
{
    return true_position;
}

MeasurementData Parser::get_speed() const
{
    return speed;
}

MeasurementData Parser::get_acceleration() const
{
    return acceleration;
}

MeasurementData Parser::get_direction() const
{
    return direction;
}

MeasurementData Parser::get_position() const
{
    return position;
}

MeasurementData Parser::parse_eigen_vec3(std::istringstream &data)
{
    MeasurementData mes_data;

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

    mes_data = point;
    return mes_data;
}

void Parser::print_data(std::string const &label, MeasurementData const &data){

    std::cout << std::fixed << std::setprecision(15)
              << "[server] " << label << ": "
              << data(0) << ", " << data(1) << ", " << data(2) << std::endl;
}

void Parser::parse_data(std::string str_buffer)
{
    std::istringstream stream(str_buffer);
    std::string line;
    MeasurementData data;


    while (std::getline(stream, line))
    {
        if (line.find("ACCELERATION") != std::string::npos)
        {
            std::cout << "BUFFER: " << str_buffer << std::endl;
            acceleration = parse_eigen_vec3(stream);
            print_data("ACCELERATION", acceleration);
        }
        else if (line.find("DIRECTION") != std::string::npos)
        {
            direction = parse_eigen_vec3(stream);
            print_data("DIRECTION", direction);
        }
        else if (line.find("TRUE POSITION") != std::string::npos)
        {
            true_position = parse_eigen_vec3(stream);
            print_data("TRUE POSITION", true_position);
        }
        else if (line.find("POSITION") != std::string::npos)
        {
            position = parse_eigen_vec3(stream);
            print_data("Position", position);
            position_set = true;
        }
        else if (line.find("SPEED") != std::string::npos)
        {
            std::getline(stream, line);
            {
                double incoming_speed = kmh_to_ms * (std::stod(line));
                std::cout << "[server] SPEED: " << line << " km/h" << std::endl;
                std::cout << "[server] SPEED: " << incoming_speed << " m/s" << std::endl;
                std::cout << "[server] UPDATED" << std::endl;
                speed = {incoming_speed, 0, 0};
            }
        }
    }
}

void Parser::read_data(int sock_fd)
{
    bool end_message_received = false;
    position_set = false;
    int buff_len;
    char buffer[MAXLINE];
    MeasurementData data;

    while (!end_message_received)
    {
        buff_len = recvfrom(sock_fd, buffer, MAXLINE, 0, NULL, NULL);
        if (buff_len < 0)
        {
            std::cerr << "Error receiving message." << std::endl; // move into udpclient
            close(sock_fd);
            return;
        }

        buffer[buff_len] = '\0';
        std::string str_buffer = buffer; // STATIC CAST OR SOMETHING
        // TBF we could just not bother with the string and have it implicitly converted when passed to parse_data()
        std::cout << "BUFFER: " << buffer << std::endl;

        parse_data(str_buffer);
        // Check for MSG_END in the received buffer
        if (str_buffer.find("MSG_END") != std::string::npos)
        {
            end_message_received = true;
        }
    }
}