
#include "UDPClient.hpp"

UDPClient::UDPClient(int port) : sock_fd(socket(AF_INET, SOCK_DGRAM, 0)), len(sizeof(servaddr))
{
    if (sock_fd < 0)
    {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);
    servaddr.sin_addr.s_addr = INADDR_ANY;
}

UDPClient::~UDPClient()
{
    close(sock_fd);
}

void UDPClient::send_handshake(std::string handshake)
{
    sendto(sock_fd, handshake.c_str(), handshake.length(), MSG_CONFIRM,
           reinterpret_cast<const struct sockaddr *>(&servaddr), sizeof(servaddr));
    std::cout << "Client: Handshake message sent." << std::endl;
}

int UDPClient::get_sock_fd()
{
    return sock_fd;
};
socklen_t UDPClient::get_sock_len()
{
    return len;
};
sockaddr_in UDPClient::get_servaddr()
{
    return servaddr;
};

void UDPClient::send_estimation(const std::string &estimation)
{
    sendto(sock_fd, estimation.c_str(), estimation.length(), MSG_CONFIRM,
           reinterpret_cast<const struct sockaddr *>(&servaddr), sizeof(servaddr));
    std::cout << "Client: Position estimation sent." << std::endl;
}
