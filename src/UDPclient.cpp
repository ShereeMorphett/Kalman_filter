
#include "UDPclient.hpp"


UDPClient::UDPClient() : sock_fd(socket(AF_INET, SOCK_DGRAM, 0)), len(sizeof(servaddr))
{
    if (sock_fd < 0) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(PORT);
    servaddr.sin_addr.s_addr = INADDR_ANY;
}

UDPClient::~UDPClient()
{
    close(sock_fd);
}

void UDPClient::send_handshake()
{
    const std::string handshake = "READY";
    sendto(sock_fd, handshake.c_str(), handshake.length(), MSG_CONFIRM, 
           reinterpret_cast<const struct sockaddr*>(&servaddr), sizeof(servaddr));
    std::cout << "Client: Handshake message sent." << std::endl;
}

void UDPClient::receive_initialization()
{
    int buff_len = recvfrom(sock_fd, buffer, MAXLINE, MSG_WAITALL, 
                            reinterpret_cast<struct sockaddr*>(&servaddr), &len);
    buffer[buff_len] = '\0';
    std::cout << "Server: " << buffer << std::endl;
    
    // Initialize filter here with the data in buffer
}

void UDPClient::send_estimation(const std::string& estimation)
{
    sendto(sock_fd, estimation.c_str(), estimation.length(), MSG_CONFIRM, 
           reinterpret_cast<const struct sockaddr*>(&servaddr), sizeof(servaddr));
    std::cout << "Client: Position estimation sent." << std::endl;
}

void UDPClient::process_loop()
{
    while (true) {
        int buff_len = recvfrom(sock_fd, buffer, MAXLINE, MSG_WAITALL, 
                                reinterpret_cast<struct sockaddr*>(&servaddr), &len);
        buffer[buff_len] = '\0';
        std::cout << "Server: " << buffer << std::endl;

        // Update your Kalman filter with the new data in buffer
        // Calculate and send the new estimation
        const std::string new_estimation = "1.7325073314060224 -2.2213777837034083 0.49999962025821726"; // Example
        send_estimation(new_estimation);
    }
}
