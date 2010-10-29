#ifndef WBCNET_UDP_UTIL_HPP
#define WBCNET_UDP_UTIL_HPP

#ifdef DISABLE_NETWORKING
# error Networking is DISABLED, do not use this header.
#endif // DISABLE_NETWORKING

#include <iosfwd>

extern "C" {
#include <sys/types.h>
#include <sys/socket.h>
}

namespace wbcnet {
  
  /** calls exit() in case something goes wrong */
  int create_udp_server(/** port specification, will get passed to getaddrinfo() */
			char const * port,
			/** use AF_UNSPEC for allowing IPv4 or IPv6,
			    AF_INET for IPv4, or AF_INET6 for IPv6 */
			int ai_family);
  
  int udp_server_recvfrom(int sfd, void * buf, size_t buf_len, int flags,
			  struct sockaddr * addr, socklen_t * addr_len);
  
  int udp_server_sendto(int sfd, void const * buf, size_t buf_len, int flags,
			struct sockaddr const * addr, socklen_t addr_len);
  
  /** calls exit() in case something goes wrong */
  int create_udp_client(/** Host to connect to. Gets passed to getaddrinfo(). */
			char const * host,
			/** Port to connect to. Gets passed to getaddrinfo(). */
			char const * port,
			/** use AF_UNSPEC for allowing IPv4 or IPv6,
			    AF_INET for IPv4, or AF_INET6 for IPv6 */
			int ai_family);

  int udp_client_write(int cfd, void const * buf, size_t buf_len);
  
  int udp_client_read(int cfd, void * buf, size_t buf_len);
  
#ifdef LINUX
  int udp_disable_mtu(int fd);
  int udp_max_priority(int fd);
#endif /* LINUX */

  int udp_tos_lowdelay(int fd);
  
}

#endif // WBCNET_UDP_UTIL_HPP
