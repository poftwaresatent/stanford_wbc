#ifndef WBCNET_WIN32_COMPAT_HPP
#define WBCNET_WIN32_COMPAT_HPP

#ifndef WIN32
# error dude
#endif

#include <winsock.h>
#include "win32/win32_stdint.hpp"
#include "win32/win32_time.hpp"

template<typename foo_t>
void bzero (foo_t * foo, size_t nbytes)
{
	memset(foo, 0, nbytes);
}

template<typename foo_t>
void usleep(foo_t usecs)
{
	Sleep(usecs / 1000);
}

typedef uint16_t in_port_t;

#endif // WBCNET_WIN32_COMPAT_HPP
