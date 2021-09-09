/**
 * @file File contains a wrapper class Socked, which wraps the platform dependent implementation of sockets.
 * @addtogroup ASOACryptMiddleware
 */

#ifndef ASOA_CORE_SOCKET_H
#define ASOA_CORE_SOCKET_H

#ifndef embedded

#include <sys/socket.h>

#else // LWIP
#include <lwip/tcpip.h>
#include <lwip/udp.h>
#endif // !embedded

#include <stdint.h>


struct PIL_Socket
{
#ifndef embedded // Linux
    /** Socket handle. */
    int m_socket;
#else // lwip
    struct udp_pcb *conn;
#endif // Linux

    u_int16_t m_port;

    char m_IPAddress[128];

    int m_protocol;

    int m_IPVersion;

#ifndef embedded // Linux
    /** SocketAddr is used to identify the
     * sender of an received datagram or message. */
    struct sockaddr m_SrcAddr;
#else // LWIP
    ip_addr_t m_SrcAddr;
#endif // !embedded
    int m_IsOpen;
    int m_LastError;
} typedef PIL_SOCKET;

/**
 * @brief Enum contains the possible IP version defines.
 */
enum PIL_IP_VERSION
{
    /** Internet protocol version 4*/
    IP_VERSION_4 = 0, /** Internet protocol version 6*/
    IP_VERSION_6 = 6
} typedef PIL_IP_VERSION;

/**
 * @brief Enum contains the possible transport protocol defines.
 */
enum PIL_TRANSPORT_PROTOCOL
{
    /** User Datagram protocol */
    TRANSPORT_UDP = 0, /** Transmission Control Protocol */
    TRANSPORT_TCP = 1
} typedef PIL_TRANSPORT_PROTOCOL;


int PIL_CreateSocket(PIL_SOCKET *socketRet, PIL_TRANSPORT_PROTOCOL protocol, PIL_IP_VERSION ipVersion,
                     const char *ipAddress, u_int16_t port);

int PIL_CloseSocket(PIL_SOCKET *socketRet);

int PIL_BindSocket(PIL_SOCKET *socketRet);

int PIL_Listen(PIL_SOCKET *socketRet, int sizeQueue);

int PIL_Accept(PIL_SOCKET *socketRet, char *ipAddr);

int PIL_Connect(PIL_SOCKET *socketRet, const char *ipAddr, uint16_t port);

int PIL_WaitTillDataAvail(PIL_SOCKET *socketRet, int timeoutMS);

int PIL_Receive(PIL_SOCKET *socketRet, uint8_t *buffer, uint16_t *bufferLen);

int PIL_ReceiveFrom(PIL_SOCKET *socketRet, uint8_t *buffer, uint16_t *bufferLen, char *ipAddr, int *port);

int PIL_Send(PIL_SOCKET *socketRet, uint8_t *buffer, uint32_t *bufferLen);

int PIL_SendTo(PIL_SOCKET *socketRet, const char *destAddr, uint16_t port, uint8_t *buffer, uint32_t *bufferLen);

const char *PIL_GetSenderIP(PIL_SOCKET *socketRet);

int PIL_IsOpen(PIL_SOCKET *socketRet);

int PIL_GetLastError(PIL_SOCKET *socketRet);

const char *PIL_GetLastErrorStr(PIL_SOCKET *socketRet);

void PIL_SetLastError(PIL_SOCKET *socketRet);


/**
* @}
* @}
*/

#endif //ASOA_CORE_SOCKET_H
