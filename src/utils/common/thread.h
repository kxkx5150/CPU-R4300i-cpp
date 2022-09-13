
#ifndef CEN64_OS_POSIX_THREAD
#define CEN64_OS_POSIX_THREAD
#include "../common.h"
#include <errno.h>
#include <pthread.h>
typedef pthread_t n64_thread;
typedef void *(*n64_thread_func)(void *arg);
typedef pthread_mutex_t n64_mutex;
typedef pthread_cond_t  n64_cv;
static inline int       n64_thread_create(n64_thread *t, n64_thread_func f, void *arg)
{
    return pthread_create(t, NULL, f, arg);
}
static inline int n64_thread_join(n64_thread *t)
{
    return pthread_join(*t, NULL);
}
#ifdef __APPLE__
int pthread_setname_np(const char *);
#elif __NETBSD__
int pthread_setname_np(n64_thread *, const char *, const char *);
#else
int pthread_setname_np(n64_thread *, const char *);
#endif
static inline int n64_thread_setname(n64_thread *t, const char *name)
{
    if (t != NULL)
        return pthread_setname_np(t, name);
    return ENOSYS;
}
static inline int n64_mutex_create(n64_mutex *m)
{
    return pthread_mutex_init(m, NULL);
}
static inline int n64_mutex_destroy(n64_mutex *m)
{
    return pthread_mutex_destroy(m);
}
static inline int n64_mutex_lock(n64_mutex *m)
{
    return pthread_mutex_lock(m);
}
static inline int n64_mutex_unlock(n64_mutex *m)
{
    return pthread_mutex_unlock(m);
}
static inline int n64_cv_create(n64_cv *cv)
{
    return pthread_cond_init(cv, NULL);
}
static inline int n64_cv_destroy(n64_cv *cv)
{
    return pthread_cond_destroy(cv);
}
static inline int n64_cv_wait(n64_cv *cv, n64_mutex *m)
{
    return pthread_cond_wait(cv, m) || pthread_mutex_unlock(m);
}
static inline int n64_cv_signal(n64_cv *cv)
{
    return pthread_cond_signal(cv);
}
#endif
