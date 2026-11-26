/*
 * Stand-in for LinuxCNC's HAL and RTAPI, for the driver test rig only.
 * See hal.h.
 */

#include "hal.h"
#include "rtapi.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
static CRITICAL_SECTION s_lock;
static int s_lock_ready;
static void lock(void)   { if (!s_lock_ready) { InitializeCriticalSection(&s_lock); s_lock_ready = 1; } EnterCriticalSection(&s_lock); }
static void unlock(void) { LeaveCriticalSection(&s_lock); }
#else
#include <pthread.h>
static pthread_mutex_t s_lock = PTHREAD_MUTEX_INITIALIZER;
static void lock(void)   { pthread_mutex_lock(&s_lock); }
static void unlock(void) { pthread_mutex_unlock(&s_lock); }
#endif

enum { T_BIT, T_FLOAT, T_U32, T_S32 };

typedef struct {
    char  name[64];
    int   type;
    void *addr;
} item_t;

typedef struct {
    char  name[64];
    void (*funct)(void *, long);
    void *arg;
} funct_t;

#define MAX_ITEMS   512
#define MAX_FUNCTS  8

static item_t  s_items[MAX_ITEMS];
static int     s_n_items;
static funct_t s_functs[MAX_FUNCTS];
static int     s_n_functs;

static char    s_messages[16384];
static int     s_message_len;

/* ---- HAL ----------------------------------------------------------- */

int hal_init(const char *name) { (void)name; lock(); unlock(); return 1; }
int hal_ready(int comp_id)     { (void)comp_id; return 0; }

int hal_exit(int comp_id)
{
    (void)comp_id;
    s_n_items = 0;
    s_n_functs = 0;
    return 0;
}

void *hal_malloc(long size)
{
    return calloc(1, (size_t)size);
}

static int add_item(const char *fmt, va_list ap, int type, void *addr)
{
    if (s_n_items >= MAX_ITEMS) return -1;
    item_t *it = &s_items[s_n_items];
    vsnprintf(it->name, sizeof(it->name), fmt, ap);
    for (int i = 0; i < s_n_items; i++)
        if (strcmp(s_items[i].name, it->name) == 0) return -1;      /* HAL refuses a second pin of one name */
    it->type = type;
    it->addr = addr;
    s_n_items++;
    return 0;
}

/* A pin's value lives in HAL's memory; the component only holds a pointer. */
#define PIN_NEWF(fn, ctype, tcode)                                                              \
    int fn(hal_pin_dir_t dir, ctype **data_ptr_addr, int comp_id, const char *fmt, ...)         \
    {                                                                                           \
        (void)dir; (void)comp_id;                                                               \
        ctype *cell = (ctype *)calloc(1, sizeof(double));                                       \
        if (!cell) return -1;                                                                   \
        *data_ptr_addr = cell;                                                                  \
        va_list ap;                                                                             \
        va_start(ap, fmt);                                                                      \
        int r = add_item(fmt, ap, tcode, (void *)cell);                                         \
        va_end(ap);                                                                             \
        return r;                                                                               \
    }

#define PARAM_NEWF(fn, ctype, tcode)                                                            \
    int fn(hal_param_dir_t dir, ctype *data_addr, int comp_id, const char *fmt, ...)            \
    {                                                                                           \
        (void)dir; (void)comp_id;                                                               \
        va_list ap;                                                                             \
        va_start(ap, fmt);                                                                      \
        int r = add_item(fmt, ap, tcode, (void *)data_addr);                                    \
        va_end(ap);                                                                             \
        return r;                                                                               \
    }

PIN_NEWF(hal_pin_bit_newf, hal_bit_t, T_BIT)
PIN_NEWF(hal_pin_float_newf, hal_float_t, T_FLOAT)
PIN_NEWF(hal_pin_u32_newf, hal_u32_t, T_U32)
PIN_NEWF(hal_pin_s32_newf, hal_s32_t, T_S32)
PARAM_NEWF(hal_param_bit_newf, hal_bit_t, T_BIT)
PARAM_NEWF(hal_param_float_newf, hal_float_t, T_FLOAT)
PARAM_NEWF(hal_param_u32_newf, hal_u32_t, T_U32)
PARAM_NEWF(hal_param_s32_newf, hal_s32_t, T_S32)

int hal_export_funct(const char *name, void (*funct)(void *, long), void *arg,
                     int uses_fp, int reentrant, int comp_id)
{
    (void)uses_fp; (void)reentrant; (void)comp_id;
    if (s_n_functs >= MAX_FUNCTS) return -1;
    snprintf(s_functs[s_n_functs].name, sizeof(s_functs[s_n_functs].name), "%s", name);
    s_functs[s_n_functs].funct = funct;
    s_functs[s_n_functs].arg = arg;
    s_n_functs++;
    return 0;
}

/* ---- for the rig --------------------------------------------------- */

static item_t *find(const char *name)
{
    for (int i = 0; i < s_n_items; i++)
        if (strcmp(s_items[i].name, name) == 0) return &s_items[i];
    return NULL;
}

int shim_get(const char *name, double *value)
{
    item_t *it = find(name);
    if (!it) return 0;
    switch (it->type) {
    case T_BIT:   *value = *(hal_bit_t *)it->addr ? 1.0 : 0.0; break;
    case T_FLOAT: *value = *(hal_float_t *)it->addr; break;
    case T_U32:   *value = (double)*(hal_u32_t *)it->addr; break;
    default:      *value = (double)*(hal_s32_t *)it->addr; break;
    }
    return 1;
}

int shim_set(const char *name, double value)
{
    item_t *it = find(name);
    if (!it) return 0;
    switch (it->type) {
    case T_BIT:   *(hal_bit_t *)it->addr = value != 0.0; break;
    case T_FLOAT: *(hal_float_t *)it->addr = value; break;
    case T_U32:   *(hal_u32_t *)it->addr = (uint32_t)value; break;
    default:      *(hal_s32_t *)it->addr = (int32_t)value; break;
    }
    return 1;
}

int shim_funct(const char *name, void (**funct)(void *, long), void **arg)
{
    for (int i = 0; i < s_n_functs; i++) {
        if (strcmp(s_functs[i].name, name) == 0) {
            *funct = s_functs[i].funct;
            *arg = s_functs[i].arg;
            return 1;
        }
    }
    return 0;
}

int shim_messages(char *out, int out_size)
{
    lock();
    int n = s_message_len < out_size - 1 ? s_message_len : out_size - 1;
    if (n < 0) n = 0;
    memcpy(out, s_messages, (size_t)n);
    if (out_size > 0) out[n] = 0;
    s_message_len = 0;
    unlock();
    return n;
}

/* ---- RTAPI --------------------------------------------------------- */

static void keep(const char *prefix, const char *fmt, va_list ap)
{
    char line[400];
    int n = snprintf(line, sizeof(line), "%s", prefix);
    vsnprintf(line + n, sizeof(line) - (size_t)n, fmt, ap);
    lock();
    int len = (int)strlen(line);
    if (s_message_len + len < (int)sizeof(s_messages)) {
        memcpy(s_messages + s_message_len, line, (size_t)len);
        s_message_len += len;
    }
    unlock();
}

void rtapi_print(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    keep("", fmt, ap);
    va_end(ap);
}

void rtapi_print_msg(msg_level_t level, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    keep(level == RTAPI_MSG_ERR ? "ERROR: " : "", fmt, ap);
    va_end(ap);
}
