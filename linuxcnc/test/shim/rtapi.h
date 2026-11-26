/*
 * Stand-in for LinuxCNC's rtapi.h, for the driver test rig only (see hal.h).
 */

#ifndef TIGGY_TEST_RTAPI_H
#define TIGGY_TEST_RTAPI_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    RTAPI_MSG_NONE = 0,
    RTAPI_MSG_ERR,
    RTAPI_MSG_WARN,
    RTAPI_MSG_INFO,
    RTAPI_MSG_DBG,
    RTAPI_MSG_ALL
} msg_level_t;

void rtapi_print(const char *fmt, ...);
void rtapi_print_msg(msg_level_t level, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* TIGGY_TEST_RTAPI_H */
