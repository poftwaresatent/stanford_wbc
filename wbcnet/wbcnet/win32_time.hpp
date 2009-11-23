#ifndef EXTRAS_H
#define EXTRAS_H

#ifndef WIN32
#error Only for Windows!!!
#endif

#define ssize_t SSIZE_T

#include "winsock.h"
#include < time.h >
#include <windows.h> //I've ommited this line.

static void usleep(int usecs) { Sleep(usecs/1000); }

#if defined(_MSC_VER) || defined(_MSC_EXTENSIONS)
  #define DELTA_EPOCH_IN_MICROSECS  11644473600000000Ui64
#else
  #define DELTA_EPOCH_IN_MICROSECS  11644473600000000ULL
#endif
 
struct timezone 
{
  int  tz_minuteswest; /* minutes W of Greenwich */
  int  tz_dsttime;     /* type of dst correction */
};
 
int gettimeofday(struct timeval *tv, struct timezone *tz);

#endif
