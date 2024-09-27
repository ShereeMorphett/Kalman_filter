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

// Client: Position estimat

// std::string initial_variables = client->receive_initialization();
//     std::cout << initial_variables << std::endl;;
//     const std::string initial_estimation = "1.7325073314060224 -2.2213777837034083 0.49999962025821726";
//     client->process_loop();

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
        std::cout << msg_count << " Server: " << buffer << std::endl;

        // Update your Kalman filter with the new data in buffer
        // Calculate and send the new estimation
        const std::string new_estimation = "0.7325073314060224 -9.2213777837034083 0.19999962025821726"; // Example
        client.send_estimation(new_estimation);
    }
}

Kalman::Kalman(int port, std::string handshake) : client(port)
{
    client.send_handshake(handshake);
}

Kalman::~Kalman() {};
