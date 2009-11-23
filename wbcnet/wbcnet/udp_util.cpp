// based on code originally copied from the getaddrinfo(3) Linux man page

#include "udp_util.hpp"
#include <wbcnet/log.hpp>

extern "C" {
#include <netdb.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <err.h>
#include <errno.h>
#ifdef OPENBSD
# include <netinet/in.h>
# include <netinet/in_systm.h>
#endif // OPENBSD
#include <netinet/ip.h>
}

static wbcnet::logger_t logger(wbcnet::get_logger("wbcnet"));

namespace wbcnet {

  
  int create_udp_server(char const * port, int ai_family)
  {
    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = ai_family;
    hints.ai_socktype = SOCK_DGRAM; /* Datagram socket */
    hints.ai_flags = AI_PASSIVE;    /* For wildcard IP address */
    hints.ai_protocol = 0;          /* Any protocol */
    hints.ai_canonname = NULL;
    hints.ai_addr = NULL;
    hints.ai_next = NULL;
    
    struct addrinfo * result;
    int const status(getaddrinfo(NULL, port, &hints, &result));
    if (status != 0)
      errx(EXIT_FAILURE, "create_udp_server(%s): getaddrinfo: %s", port, gai_strerror(status));
    
    struct addrinfo * rp;
    int sfd;
    for (rp = result; rp != NULL; rp = rp->ai_next) {
      sfd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
      if (sfd == -1)
	continue;
      if (bind(sfd, rp->ai_addr, rp->ai_addrlen) == 0) {
	LOG_DEBUG (logger, "create_udp_server(): bound to port " << port);
	break;
      }
      close(sfd);
    }
    if (rp == NULL)
      errx(EXIT_FAILURE, "create_udp_server(%s): could not bind", port);
    
    freeaddrinfo(result);           /* No longer needed */
    
    return sfd;
  }
  
  
  int udp_server_recvfrom(int sfd, void * buf, size_t buf_len, int flags,
			  struct sockaddr * addr, socklen_t * addr_len)
  {
    int const nread(recvfrom(sfd, buf, buf_len, flags, addr, addr_len));
    if (-1 == nread) {
      LOG_DEBUG (logger, "udp_server_recvfrom(): recvfrom: " << strerror(errno));
      return -1;
    }
    
    if (logger->isDebugEnabled()) {
      char host[NI_MAXHOST], service[NI_MAXSERV];
      int const status(getnameinfo(addr, *addr_len, host, NI_MAXHOST,
				   service, NI_MAXSERV, NI_NUMERICSERV));
      if (0 == status) {
	LOG_DEBUG (logger,
		       "udp_server_recvfrom(): received " << nread << " bytes from "
		       << host << ":" << service);
      }
      else {
	LOG_DEBUG (logger, "udp_server_recvfrom(): getnameinfo: " << gai_strerror(status));
      }
    }
    
    return nread;
  }
  
  
  int udp_server_sendto(int sfd, void const * buf, size_t buf_len, int flags,
			struct sockaddr const * addr, socklen_t addr_len)
  {
    int const nsent(sendto(sfd, buf, buf_len, flags, addr, addr_len));
    if (-1 == nsent) {
      LOG_DEBUG (logger, "udp_server_sendto(): sendto: " << strerror(errno));
      return -1;
    }
    
    if (logger->isDebugEnabled()) {
      char host[NI_MAXHOST], service[NI_MAXSERV];
      int const status(getnameinfo(addr, addr_len, host, NI_MAXHOST,
				   service, NI_MAXSERV, NI_NUMERICSERV));
      if (0 == status) {
	LOG_DEBUG (logger,
		       "udp_server_sendto(): sent " << nsent << " of " << buf_len << " bytes to "
		       << host << ":" << service);
      }
      else {
	LOG_DEBUG (logger,
		       "udp_server_recvfrom(): getnameinfo: " << gai_strerror(status)
		       << " (but sent " << nsent << " of " << buf_len << " bytes)");
      }
    }
    
    return nsent;
  }
  
  
  int create_udp_client(char const * host, char const * port, int ai_family)
  {
    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = ai_family;
    hints.ai_socktype = SOCK_DGRAM; /* Datagram socket */
    hints.ai_flags = 0;
    hints.ai_protocol = 0;          /* Any protocol */
    
    struct addrinfo * result;
    int const status(getaddrinfo(host, port, &hints, &result));
    if (status != 0)
      errx(EXIT_FAILURE, "create_udp_client(%s, %s): getaddrinfo: %s",
	   host, port, gai_strerror(status));
    
    struct addrinfo * rp;
    int cfd;
    for (rp = result; rp != NULL; rp = rp->ai_next) {
      cfd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
      if (cfd == -1)
	continue;
      if (connect(cfd, rp->ai_addr, rp->ai_addrlen) != -1) {
	LOG_DEBUG (logger, "create_udp_client(): connected to port " << port << " on " << host);
	break;
      }
      close(cfd);
    }
    if (rp == NULL)
      errx(EXIT_FAILURE, "create_udp_client(%s, %s): getaddrinfo: could not connect", host, port);
    
    freeaddrinfo(result);           /* No longer needed */
    
    return cfd;
  }
  
  
  int udp_client_write(int cfd, void const * buf, size_t buf_len)
  {
    int const nwritten(write(cfd, buf, buf_len));
    if (-1 == nwritten) {
      LOG_DEBUG (logger, "udp_client_write(): write: " << strerror(errno));
      return -1;
    }
    
    LOG_DEBUG (logger,
		   "udp_client_write(): wrote " << nwritten << " of " << buf_len << " bytes");
    
    return nwritten;
  }
  
  
  int udp_client_read(int cfd, void * buf, size_t buf_len)
  {
    int const nread(read(cfd, buf, buf_len));
    if (-1 == nread) {
      LOG_DEBUG (logger, "udp_client_read(): read: " << strerror(errno));
      return -1;
    }
    
    LOG_DEBUG (logger, "udp_client_read(): read " << nread << " of " << buf_len << " bytes");
    
    return nread;
  }
  
#ifdef LINUX
  int udp_disable_mtu(int fd)
  {
    int const flag(IP_PMTUDISC_DONT);
    return setsockopt(fd, IPPROTO_IP, IP_MTU_DISCOVER, &flag, sizeof(flag));
  }
  
  int udp_max_priority(int fd)
  {
    int prio(6);
    if (0 == geteuid())
      prio = 12;
    return setsockopt(fd, SOL_SOCKET, SO_PRIORITY, &prio, sizeof(prio));
  }
#endif /* LINUX */  
  
  
  int udp_tos_lowdelay(int fd)
  {
    int const flag(IPTOS_LOWDELAY);
    return setsockopt(fd, IPPROTO_IP, IP_TOS, &flag, sizeof(flag));
  }

}
