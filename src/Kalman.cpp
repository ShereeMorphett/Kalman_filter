#include "Kalman.hpp"

// 1Server: Trajectory Generated!
// Sending Info. . .

// Client: Position estimation sent.
// 2Server: MSG_START
// Client: Position estimation sent.
// 3Server: [00:00:00.000]TRUE POSITION
// 3.96078188763192
// -4.567674707852567
// 0.5

// Client: Position estimation sent.
// 4Server: [00:00:00.000]SPEED
// 57.911156558054685

// Client: Position estimation sent.
// 5Server: [00:00:00.000]ACCELERATION
// -0.0059844552782635976
// 0.004550056420937033
// -0.011442255948922328

// Client: Position estimation sent.
// 6Server: [00:00:00.000]DIRECTION
// 0.003859381882928678
// 0.003958111620245174
// 0.011566594139606983

// 1. Understanding the Inputs:
// Initial Position (X, Y, Z): Your starting true position in 3D space.
// Initial Speed: Your speed at the beginning (in km/h).
// Current Acceleration: The current acceleration in 3D space (in m/s²).
// Current Direction (Euler Angles): The direction the vehicle is heading, given in Euler angles. You’ll need to convert this to a rotation matrix or quaternion for position update calculations.
// GPS Position: Every 3 seconds, you get a GPS reading, which has noise and provides your position (X, Y, Z).
// Noise Characteristics: Gaussian white noise added to your sensors, which will need to be modeled in your Kalman filter.
void Kalman::filter_loop()
{
    int msg_count = 0;
    int sock_fd = client.get_sock_fd();
    sockaddr_in servaddr = client.get_servaddr();
    socklen_t len = client.get_sock_len();

    while (true)
    {
        int buff_len = recvfrom(sock_fd, buffer, MAXLINE, MSG_WAITALL,
                                reinterpret_cast<struct sockaddr *>(&servaddr), &len);
        buffer[buff_len] = '\0';
        msg_count++;
        std::string str_buffer = buffer;

        if (str_buffer.find("TRUE POSITION") != std::string::npos)
        {
            std::cout << msg_count << " server TRUE POSITION: " << buffer << std::endl;
        }
        else if (str_buffer.find("SPEED") != std::string::npos)
        {
            std::cout << msg_count << " server SPEED: " << buffer << std::endl;
        }
        else if (str_buffer.find("ACCELERATION") != std::string::npos)
        {
            std::cout << msg_count << " server ACCELERATION: " << buffer << std::endl;
        }
        else if (str_buffer.find("DIRECTION") != std::string::npos)
        {
            std::cout << msg_count << " server DIRECTION: " << buffer << std::endl;
        }
    }
}

Kalman::Kalman(int port, std::string handshake) : client(port)
{
    client.send_handshake(handshake);
}

Kalman::~Kalman() {};
