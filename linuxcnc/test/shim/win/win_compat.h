/*
 * Just enough POSIX for linuxcnc/tiggy.c to be compiled, unchanged, with
 * the Microsoft compiler for the driver test rig.  The headers beside this
 * one (unistd.h, poll.h, pthread.h ...) all lead here.
 */

#ifndef TIGGY_TEST_WIN_COMPAT_H
#define TIGGY_TEST_WIN_COMPAT_H

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <process.h>
#include <errno.h>
#include <stdint.h>
#include <time.h>

#define tg_barrier()        MemoryBarrier()

/* ---- time ---------------------------------------------------------- */

#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC 1
#endif

static __inline int shim_clock_gettime(int id, struct timespec *ts)
{
    LARGE_INTEGER f, c;
    (void)id;
    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&c);
    ts->tv_sec = (time_t)(c.QuadPart / f.QuadPart);
    ts->tv_nsec = (long)(((c.QuadPart % f.QuadPart) * 1000000000LL) / f.QuadPart);
    return 0;
}
#define clock_gettime(id, ts)   shim_clock_gettime((id), (ts))

static __inline int shim_usleep(unsigned us)
{
    Sleep(us >= 1000 ? us / 1000 : 1);
    return 0;
}
#define usleep(us)              shim_usleep(us)

/* ---- sockets ------------------------------------------------------- */

#define MSG_NOSIGNAL    0
#define O_NONBLOCK      0x4000
#define F_GETFL         3
#define F_SETFL         4
#ifndef IFF_UP
#define IFF_UP          0x1
#define IFF_BROADCAST   0x2
#endif

static __inline int shim_error(int r)
{
    if (r < 0) {
        int e = WSAGetLastError();
        errno = (e == WSAEWOULDBLOCK) ? EWOULDBLOCK
              : (e == WSAEINPROGRESS || e == WSAEALREADY) ? EINPROGRESS
              : (e == WSAEINTR) ? EINTR : EIO;
    }
    return r;
}

static __inline int shim_socket(int family, int type, int protocol)
{
    SOCKET s = socket(family, type, protocol);
    if (s == INVALID_SOCKET) return -1;
    if (type == SOCK_DGRAM) {
        /* Windows reports "port unreachable" as an error on the next
         * receive of a UDP socket; Linux does not. */
        DWORD off = 0, bytes = 0;
        WSAIoctl(s, _WSAIOW(IOC_VENDOR, 12), &off, sizeof(off), NULL, 0, &bytes, NULL, NULL);
    }
    return (int)s;
}

static __inline int shim_fcntl(int fd, int cmd, int arg)
{
    if (cmd == F_SETFL) {
        u_long on = (arg & O_NONBLOCK) ? 1 : 0;
        return ioctlsocket((SOCKET)fd, FIONBIO, &on) == 0 ? 0 : -1;
    }
    return 0;
}

#define socket(f, t, p)                 shim_socket((f), (t), (p))
#define close(fd)                       closesocket((SOCKET)(fd))
#define fcntl(fd, cmd, arg)             shim_fcntl((fd), (cmd), (arg))
#define poll(p, n, t)                   WSAPoll((p), (n), (t))
#define recv(s, b, l, f)                shim_error(recv((s), (b), (l), (f)))
#define send(s, b, l, f)                shim_error(send((s), (b), (l), (f)))
#define connect(s, a, l)                shim_error(connect((s), (a), (l)))
#define recvfrom(s, b, l, f, a, al)     shim_error(recvfrom((s), (b), (l), (f), (a), (al)))

/* No list of network interfaces: the search then uses the general
 * broadcast address, this computer and the last known address. */
struct ifaddrs {
    struct ifaddrs  *ifa_next;
    unsigned         ifa_flags;
    struct sockaddr *ifa_addr;
    struct sockaddr *ifa_broadaddr;
};
static __inline int  getifaddrs(struct ifaddrs **list) { *list = NULL; return -1; }
static __inline void freeifaddrs(struct ifaddrs *list) { (void)list; }

/* ---- threads ------------------------------------------------------- */

typedef HANDLE pthread_t;
typedef int    pthread_attr_t;

static __inline int pthread_attr_init(pthread_attr_t *a)                    { *a = 0; return 0; }
static __inline int pthread_attr_destroy(pthread_attr_t *a)                 { (void)a; return 0; }
static __inline int pthread_attr_setstacksize(pthread_attr_t *a, size_t n)  { (void)a; (void)n; return 0; }

typedef struct { void *(*fn)(void *); void *arg; } shim_thread_start_t;

static unsigned __stdcall shim_thread_entry(void *p)
{
    shim_thread_start_t s = *(shim_thread_start_t *)p;
    free(p);
    s.fn(s.arg);
    return 0;
}

static __inline int pthread_create(pthread_t *t, const pthread_attr_t *a, void *(*fn)(void *), void *arg)
{
    shim_thread_start_t *s = (shim_thread_start_t *)malloc(sizeof(*s));
    (void)a;
    if (!s) return -1;
    s->fn = fn;
    s->arg = arg;
    *t = (HANDLE)_beginthreadex(NULL, 0, shim_thread_entry, s, 0, NULL);
    if (!*t) { free(s); return -1; }
    return 0;
}

static __inline int pthread_join(pthread_t t, void **result)
{
    (void)result;
    WaitForSingleObject(t, INFINITE);
    CloseHandle(t);
    return 0;
}

#endif /* TIGGY_TEST_WIN_COMPAT_H */
