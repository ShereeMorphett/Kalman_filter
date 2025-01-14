
#include "UDPClient.hpp"
#include "colour.hpp"

UDPClient::UDPClient(int port) : len(sizeof(servaddr))
{
#ifdef _WIN32
    WSADATA wsaData;
    int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (result != 0)
    {
        std::cout << "WSAStartup failed: " << result << std::endl;
        exit(EXIT_FAILURE);
    }
#endif

    sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0)
    {
#ifdef _WIN32
        std::cout << "Socket creation failed: " << WSAGetLastError() << std::endl;
#else
        perror("socket creation failed");
#endif
        exit(EXIT_FAILURE);
    }
    msg_sent = 0;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);
    servaddr.sin_addr.s_addr = INADDR_ANY;
}

UDPClient::~UDPClient()
{

#ifdef _WIN32
    closesocket(sock_fd);
    WSACleanup();
#else
    close(sock_fd);
#endif
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
    ssize_t bytes_sent = sendto(sock_fd, estimation.c_str(), estimation.length(), MSG_CONFIRM,
                                reinterpret_cast<const struct sockaddr *>(&servaddr), sizeof(servaddr));

    if (bytes_sent < 0)
    {
        if (errno == EPIPE || errno == ECONNRESET)
        {
            std::cout << COLOR_RED << "[Client] Error: Connection lost while sending estimation."
                      << COLOR_RESET << std::endl;
            close(sock_fd);
            exit(1); // NOPE
        }
        else
        {
            perror("Error sending estimation");
            close(sock_fd);
            exit(1); //NOPE
        }
    }
    else if (bytes_sent == 0)
    {
        std::cout << COLOR_YELLOW << "[Client] Warning: No data was sent." << COLOR_RESET << std::endl;
        close(sock_fd);
        exit(1); //Nope
    }
    else
    {
        msg_sent++;
        std::cout << COLOR_GREEN << "[Client] Position [" << msg_sent << "] estimation sent.\n"
                  << COLOR_RESET << std::endl;
    }
}