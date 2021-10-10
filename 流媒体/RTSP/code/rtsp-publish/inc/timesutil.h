#ifndef TIMEUTIL_H
#define TIMEUTIL_H

#include <stdint.h>
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>

#ifdef _MSC_VER /* MSVC */
//#define snprintf _snprintf
#define strcasecmp stricmp
#define strncasecmp strnicmp
//#define vsnprintf _vsnprintf
#endif

#define GetSockError() WSAGetLastError()
#define SetSockError(e) WSASetLastError(e)
#define setsockopt(a, b, c, d, e) (setsockopt)(a, b, c, (const char *)d, (int)e)
#define EWOULDBLOCK WSAETIMEDOUT /* we don't use nonblocking, but we do use timeouts */
#define sleep(n) Sleep(n * 1000)
#define msleep(n) Sleep(n)
#define SET_RCVTIMEO(tv, s) int tv = s * 1000
#else /* !_WIN32 */
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/times.h>
#include <sys/types.h>
#include <unistd.h>
#define GetSockError() errno
#define SetSockError(e) errno = e
#undef closesocket
#define closesocket(s) close(s)
#define msleep(n) usleep(n * 1000)
#define SET_RCVTIMEO(tv, s) struct timeval tv = {s, 0}
#endif

#include <chrono>
using namespace std;
using namespace std::chrono;

#ifdef _WIN32
#pragma comment(lib, "ws2_32.lib")
#endif

class TimesUtil {
 public:
    static inline int64_t GetTimeMillisecond() {
#ifdef _WIN32
        return (int64_t)GetTickCount();
#else
        struct timeval tv;
        gettimeofday(&tv, NULL);
        return ((int64_t)tv.tv_sec * 1000 + (unsigned long long)tv.tv_usec / 1000);
#endif

        //        return duration_cast<chrono::milliseconds>(high_resolution_clock::now() - m_begin).count();
    }
    // private:
    //    static time_point<high_resolution_clock> m_begin;
};

#endif  // TIMEUTIL_H
