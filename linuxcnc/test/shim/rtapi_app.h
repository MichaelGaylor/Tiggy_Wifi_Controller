/*
 * Stand-in for LinuxCNC's rtapi_app.h, for the driver test rig only (see
 * hal.h).  Module parameters become functions the rig can reach them by.
 */

#ifndef TIGGY_TEST_RTAPI_APP_H
#define TIGGY_TEST_RTAPI_APP_H

#define MODULE_AUTHOR(text)
#define MODULE_DESCRIPTION(text)
#define MODULE_LICENSE(text)

#define RTAPI_MP_STRING(var, description)   char **shim_mp_##var(void) { return &var; }
#define RTAPI_MP_INT(var, description)      int *shim_mp_##var(void) { return &var; }

int  rtapi_app_main(void);
void rtapi_app_exit(void);

#endif /* TIGGY_TEST_RTAPI_APP_H */
