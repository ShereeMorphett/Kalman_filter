
#include <iostream>
#include <unistd.h>
#include <Eigen/Dense> //may only need the dense
#include "UDPclient.hpp"

/*
OPTIONS:
    -a, --accsig <acceleration_sigma>
            Manually specify the accelerometer's error sigma

    -d, --duration <trajectory_duration>
            Specify trajectory duration in minutes from unsigned integer

        --debug
            Enables debug mode which provides true information in addition to the noised information

        --delta
            Print the difference between real position and received position at each reception.

    -e, --entropy
            Generate seed from entropy.

        --filterspeed
            Print the mean of response time from the filter at the end of transmission.

    -g, --gpssig <gps_sigma>
            Manually specify the GPS' error sigma

    -h, --help
            Print help information

    -n, --noise <noise_increase>
            Multiply the noise applied to the accelerometer and GPS

    -p, --port <port>
            Manually specify port for the server. Otherwise defaults to 4242. If the port is closed,
            program defaults to closest open port.

    -s, --seed <manual_seed>
            Generate seed from unsigned integer

    -V, --version
            Print version information
*/
    // Create a UDP socket.
        // Send a message to the server.
        // Wait until a response from the server is received.
        // Process the reply and go back to step 2, if necessary.
        // Close socket descriptor and exit.
        // int socket(int domain, int type, int protocol)
        // Creates an unbound socket in the specified domain.
        // Returns socket file descriptor.

    // Arguments : 
        // domain – Specifies the communication 
        // domain ( AF_INET for IPv4/ AF_INET6 for IPv6 ) 
        // type – Type of socket to be created 
        // ( SOCK_STREAM for TCP / SOCK_DGRAM for UDP ) 
        // protocol – Protocol to be used by the socket. 
        // 0 means using the default protocol for the address family.

int main() {
    std::unique_ptr<UDPClient> client = std::make_unique<UDPClient>();

    client->send_handshake();
    client->receive_initialization();

    const std::string initial_estimation = "1.7325073314060224 -2.2213777837034083 0.49999962025821726";
    client->send_estimation(initial_estimation);

    client->process_loop();

    return 0;
}
