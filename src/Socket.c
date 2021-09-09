#include "helperFiles/Socket.h"
#include <sys/socket.h> // helperFiles, recvfrom, sendto
#include <arpa/inet.h> // htons, inet_addr, inet_ntoa
#include <string.h> // memset
#include <unistd.h> // close
#include <errno.h> // errno
#include <sys/fcntl.h>
#include <stdio.h>

#if embedded
#include <lwip/tcpip.h>
#include <lwip/udp.h>
#include <lwip/igmp.h>
#endif // embedded

#ifdef __linux__
#include <sys/select.h> // fd_set, timeval, select
#include <sys/ioctl.h>

#endif // __linux__

/**
 *
 * @brief Creates a helperFiles, dependent on the keyServer config stored in the object.
 *        It can create UDP and TCP sockets using IPv4 or IPv6.
 * @param socketRet return value of the helperFiles.
 * @param protocol protocol used either TCP or UDP is supported.
 * @param ipVersion use IPv4 or IPv6.
 * @param ipAddress IP address to bind.
 * @param port port to bind.
 * @return 0 if no error occurred.
 */
int PIL_CreateSocket(PIL_SOCKET *socketRet, PIL_TRANSPORT_PROTOCOL protocol, PIL_IP_VERSION ipVersion,
                     const char *ipAddress, const uint16_t port)
{
    if (!socketRet)
        return -1;

    int tProtocol = SOCK_DGRAM;
    if (protocol == 1 /*TCP*/)
        tProtocol = SOCK_STREAM;

    int ipProtocol = AF_INET;
    if (ipVersion == 1 /* IPv6 */)
        ipProtocol = AF_INET6;

    socketRet->m_protocol = tProtocol;
    socketRet->m_IPVersion = ipProtocol;
    strcpy((char *) socketRet->m_IPAddress, ipAddress);
    socketRet->m_port = port;

#ifndef embedded
    socketRet->m_socket = socket(ipProtocol, tProtocol, 0);
    if ( socketRet->m_socket == -1)
    {
        socketRet->m_IsOpen = 0;
        PIL_SetLastError(socketRet);
        return -1;
    }

    socketRet->m_IsOpen = 1;

#else // lwip
    socketRet->conn = udp_new();
#endif // linux
    return 0;
}

/**
 * @brief Closes a helperFiles if helperFiles is open.
 * @param socketRet handle corresponding to the helperFiles which should be closed.
 * @return 0 when no error occurred else return -1.
 */
int PIL_CloseSocket(PIL_SOCKET *socketRet)
{
    if(!socketRet)
        return -1;

#ifndef embedded
    if (socketRet->m_IsOpen)
    {
        socketRet->m_IsOpen = 0;
        if (close(socketRet->m_socket) != 0)
        {
            PIL_SetLastError(socketRet);
            return -1;
        }
    }
#else // lwip
    udp_remove(socketRet->conn);
#endif // linux
    return 0;
}

/**
 * @brief Bind helperFiles on address given in keyserver config.
 * @param socketRet handle to bind the helperFiles.
 * @return 0 if no error occurred else -1.
 */
int PIL_BindSocket(PIL_SOCKET *socketRet)
{
    if(!socketRet)
        return -1;

#ifndef embedded
    // Allow reuse of local addresses
    uint32_t reuse = 1;
    int sockOptRet = setsockopt(socketRet->m_socket,
                                SOL_SOCKET, SO_REUSEADDR, (char *) &reuse, sizeof(reuse));
    if (sockOptRet != 0)
    {
        PIL_SetLastError(socketRet);
        return -1;
    }

    // Set transport protocol (UDP/TCP), IP-version (IPv4, IPv6) and the address of the helperFiles to bound on
    struct sockaddr_in address = {0};
    memset(&address, 0, sizeof(address));

    address.sin_family = socketRet->m_IPVersion;
    address.sin_addr.s_addr = inet_addr(socketRet->m_IPAddress);
    address.sin_port = htons(socketRet->m_port);

    int bindRet = bind(socketRet->m_socket, (struct sockaddr *) &address, sizeof(struct sockaddr));
    if (bindRet != 0)
    {
        PIL_SetLastError(socketRet);
        return -1;
    }
#else // lwip
    err_t ret = udp_bind(socketRet->conn, &socketRet->m_SrcAddr, socketRet->m_port);
	if(ret != 0)
	{
        PIL_SetLastError(socketRet);
        return -1;
    	}
#endif // Linux
    return 0;
}

/**
 * @brief Perform listen operation on a helperFiles using a TCP connection.
 * @param socketRet helperFiles to listen.
 * @param sizeQueue size of the queue for listening operations.
 * @return 0 if no error occurs else -1.
 */
int PIL_Listen(PIL_SOCKET *socketRet, int sizeQueue)
{
#ifndef embedded
    if(socketRet == NULL)
        return -1;
    int listenRet = listen(socketRet->m_socket, sizeQueue);
    if(listenRet != 0)
    {
        PIL_SetLastError(socketRet);
        return -1;
    }
#endif // we use LWIP only for UDP

    return 0;
}

/**
 * @brief This function accepts a message when using TCP connections.
 * @param socket helperFiles to accept.
 * @param ipAddr ipAddress of the new accepted connection.
 * @return 0 if no error occurs else -1.
 */
int PIL_Accept(PIL_SOCKET *socket, char *ipAddr)
{
#ifndef embedded
    if(socket == NULL)
        return -1;
    struct sockaddr_in address;
    socklen_t addrLen = sizeof(address);
    int acceptRet = accept(socket->m_socket, (struct sockaddr*) &address, &addrLen);
    if(acceptRet < 0)
    {
        PIL_SetLastError(socket);
        return -1;
    }
    if(ipAddr != NULL)
        strcpy(ipAddr,inet_ntoa(address.sin_addr));
    return acceptRet;
#else // our LWIP implementation only supports UDP!
    return 0;
#endif // Linux
}

/**
 * @brief Connect function if a TCP helperFiles is used.
 * @param socket helperFiles on which the connect function is called.
 * @param ipAddr ip address of the connection.
 * @param port port of the new connection.
 * @return 0 if no error occured.
 */
int PIL_Connect(PIL_SOCKET *socket, const char *ipAddr, uint16_t port)
{
    if(socket == NULL || ipAddr == NULL)
        return -1;

#ifndef embedded
    struct sockaddr_in address;
    address.sin_family = socket->m_IPVersion;
    address.sin_port = htons(port);
    address.sin_addr.s_addr = inet_addr(ipAddr);
    int connectRet = connect(socket->m_socket, (struct sockaddr*) &address, sizeof(address));
    if(connectRet != 0)
    {
        PIL_SetLastError(socket);
        return -1;
    }
#endif // On LWIP we only support UDP!
    return 0;
}

/**
 * @brief Wait until timeout if data is available at the helperFiles.
 * @param timeoutMS timeout to wait until data is available.
 * @return -1 if error occurs, 0 if no data is available and timeout occurs else data to read is available.
 */
int PIL_WaitTillDataAvail(PIL_SOCKET *socketRet, int timeoutMS)
{
#ifndef embedded
    if(!socketRet)
        return -1;

    fd_set readFD;
    struct timeval timeout = { 0 };
    // set select write handle to zero
    FD_ZERO(&readFD);
    FD_SET(socketRet->m_socket, &readFD);

    timeout.tv_usec = (timeoutMS % 1000) * 1000;
    timeout.tv_sec = (timeoutMS - (timeoutMS % 1000)) / 1000;
    int ret = select(socketRet->m_socket +1 , &readFD, NULL, NULL, &timeout);
    if (ret < 0)
{
        PIL_SetLastError(socketRet);
return -1;
}
    else
{
       if( FD_ISSET(socketRet->m_socket, &readFD))
	       return ret;
	return 0;
}
   return 0;
#else
    return 0;
#endif // !embedded
}

int PIL_Receive(PIL_SOCKET *socketRet ,uint8_t *buffer, uint16_t *bufferLen)
{
    if (!socketRet)
      {  return -1;}

        if (recv(socketRet->m_socket, buffer, *bufferLen, 0) == -1)
        {
            PIL_SetLastError(socketRet);
            return -1;
        }
        return 0;
}


/**
 * @brief Receive Data in blocking mode. Stores IPAddress and port of sending participant.
 * (WaitTillDataAvail must be called previously to avoid deadlocks)
 * @param buffer m_ReceiveBuffer to store the received data.
 * @param bufferLen size of the m_ReceiveBuffer, after method call, this variable contains the size of the sent data.
 * @return 0 if no error is occurred else -1 is returned.
 */
int PIL_ReceiveFrom(PIL_SOCKET *socketRet ,uint8_t *buffer, uint16_t *bufferLen, char *ipAddr, int *port)
{
    if(!socketRet)
        return -1;

#ifndef embedded
    struct sockaddr_in addr;
    memset(&addr, 0x00, sizeof(struct sockaddr_in));
    addr.sin_family = socketRet->m_protocol;
    addr.sin_port = htons(socketRet->m_port);
    addr.sin_addr.s_addr = INADDR_ANY;
    socklen_t senderAddrLen = sizeof(addr);


    int ret = recvfrom(socketRet->m_socket, buffer, *bufferLen, MSG_WAITALL,
                       (struct sockaddr*) &addr, &senderAddrLen);
    if (ret < 0)
    {
        PIL_SetLastError(socketRet);
        return -1;
    }
    if(ipAddr != NULL)
        strcpy(ipAddr, inet_ntoa(addr.sin_addr));
    if(port != NULL)
        *port =  ntohs(addr.sin_port);

    *bufferLen = ret;
#else // LWIP
    struct udp_pcb buff;
   // err_t ret = udp_recv(buff, );
//    if(ret != 0)
    {
        PIL_SetLastError(socketRet);
        return -1;
    }
#endif // LINUX
    return 0;
}


/**
 * @brief Method sends data to participant stored in m_SrcAddr (set by receive function).
 * @param buffer m_ReceiveBuffer to send.
 * @param bufferLen length of the m_ReceiveBuffer, after this method, the sent data are stored in this variable.
 * @return 0 if no error is occurred else -1 is returned.
 */
int PIL_Send(PIL_SOCKET *socketRet, uint8_t *buffer, uint32_t *bufferLen)
{
    if(!socketRet)
        return -1;

#ifndef embedded
    //  socklen_t senderAddrLen = sizeof(socketRet->m_SrcAddr);
    int ret = send(socketRet->m_socket, buffer, *bufferLen, 0/*, &socketRet->m_SrcAddr, senderAddrLen*/);
    if (ret < 0)
    {
        PIL_SetLastError(socketRet);
        return -1;
    }
    *bufferLen = ret;
#else // LWIP
    struct pbuf *newBuff;
	buffer = pbuf_alloc(PBUF_TRANSPORT, *bufferLen, PBUF_RAM);
	err_t ret = udp_send(socketRet->conn, newBuff);
	if(ret != 0)
	{
        PIL_SetLastError(socketRet);
        return -1;
    	}
#endif // Linux
    return 0;
}

/**
 * @brief Method sends data to participant stored in m_SrcAddr (set by receive function).
 * @param buffer m_ReceiveBuffer to send.
 * @param bufferLen length of the m_ReceiveBuffer, after this method, the sent data are stored in this variable.
 * @return 0 if no error is occurred else -1 is returned.
 */
int PIL_SendTo(PIL_SOCKET *socketRet, const char* destAddr, const uint16_t port, uint8_t *buffer, uint32_t *bufferLen)
{
#ifndef embedded
    if(destAddr == NULL || buffer == NULL)
        return -1;

    ((struct sockaddr_in*)&socketRet->m_SrcAddr)->sin_family = socketRet->m_IPVersion;
    ((struct sockaddr_in*)&socketRet->m_SrcAddr)->sin_addr.s_addr = inet_addr(destAddr);
    ((struct sockaddr_in*)&socketRet->m_SrcAddr)->sin_port = htons(port);

    socklen_t senderAddrLen = sizeof(socketRet->m_SrcAddr);
    int ret = sendto(socketRet->m_socket, buffer, *bufferLen, 0, &socketRet->m_SrcAddr, senderAddrLen);
    if (ret < 0)
    {
        PIL_SetLastError(socketRet);
        return -1;
    }
    *bufferLen = ret;
#else // LWIP
    struct pbuf *newbuff;
	buffer = pbuf_alloc(PBUF_TRANSPORT, *bufferLen, PBUF_RAM);
	if(ipaddr_aton(&socketRet->m_SrcAddr, &socketRet->m_SrcAddr) != 0)
	    return -1;
	err_t ret = udp_sendto(socketRet->conn, newbuff, &socketRet->m_SrcAddr, socketRet->m_port);
	if(ret != 0)
            return -1;
#endif // Linux
    return 0;
}

/**
 * @brief Method can be called after receive operation and
 * returns the IP-address of the sender of the last received package.
 * @return IP-address of the sender.
 */
const char *PIL_GetSenderIP(PIL_SOCKET *socketRet)
{
#ifndef embedded
    if(!socketRet)
        return NULL;

    return inet_ntoa(((struct sockaddr_in *) &socketRet->m_SrcAddr)->sin_addr);
#endif // Not supported by LWIP
}

/**
 * Return the last error as integer. (Can be converted to text via strerror function).
 * @return last error code.
 */
int PIL_GetLastError(PIL_SOCKET *socketRet)
{
    if(!socketRet)
        return -1;

    return socketRet->m_LastError;
}

const char* PIL_GetLastErrorStr(PIL_SOCKET *socketRet)
{
    if(!socketRet)
        return "Socket == nullptr";
#if __linux__
    return strerror(socketRet->m_LastError);
#else // __WIN32__

#endif // __linux

}

/**
 * @brief Set error of an helperFiles.
 * This function must be called directly after the corresponding helperFiles function and only if their result is -1.
 * @param socketRet
 */
void PIL_SetLastError(PIL_SOCKET *socketRet)
{

    if(!socketRet)
        return;
#ifndef embedded
    socketRet->m_LastError = errno;
#endif // only on linux
}
