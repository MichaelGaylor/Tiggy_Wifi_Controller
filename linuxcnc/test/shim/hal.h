/*
 * Stand-in for LinuxCNC's hal.h, for the driver test rig only.
 *
 * It lets linuxcnc/tiggy.c be compiled and run on a PC without LinuxCNC:
 * pins and parameters are plain memory, found by name through the shim_*
 * functions below.  Only what the driver uses is here, with the same
 * signatures as the real thing.
 */

#ifndef TIGGY_TEST_HAL_H
#define TIGGY_TEST_HAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef volatile bool     hal_bit_t;
typedef volatile uint32_t hal_u32_t;
typedef volatile int32_t  hal_s32_t;
typedef volatile double   hal_float_t;

typedef enum { HAL_IN = 16, HAL_OUT = 32, HAL_IO = 48 } hal_pin_dir_t;
typedef enum { HAL_RO = 64, HAL_RW = 192 } hal_param_dir_t;

int   hal_init(const char *name);
int   hal_ready(int comp_id);
int   hal_exit(int comp_id);
void *hal_malloc(long size);

int hal_pin_bit_newf(hal_pin_dir_t dir, hal_bit_t **data_ptr_addr, int comp_id, const char *fmt, ...);
int hal_pin_float_newf(hal_pin_dir_t dir, hal_float_t **data_ptr_addr, int comp_id, const char *fmt, ...);
int hal_pin_u32_newf(hal_pin_dir_t dir, hal_u32_t **data_ptr_addr, int comp_id, const char *fmt, ...);
int hal_pin_s32_newf(hal_pin_dir_t dir, hal_s32_t **data_ptr_addr, int comp_id, const char *fmt, ...);

int hal_param_bit_newf(hal_param_dir_t dir, hal_bit_t *data_addr, int comp_id, const char *fmt, ...);
int hal_param_float_newf(hal_param_dir_t dir, hal_float_t *data_addr, int comp_id, const char *fmt, ...);
int hal_param_u32_newf(hal_param_dir_t dir, hal_u32_t *data_addr, int comp_id, const char *fmt, ...);
int hal_param_s32_newf(hal_param_dir_t dir, hal_s32_t *data_addr, int comp_id, const char *fmt, ...);

int hal_export_funct(const char *name, void (*funct)(void *, long), void *arg,
                     int uses_fp, int reentrant, int comp_id);

/* --- for the rig, not part of HAL --- */

/* Value of a pin or parameter as a number.  Returns 0 if there is no such name. */
int  shim_get(const char *name, double *value);
int  shim_set(const char *name, double value);
/* An exported function and its argument. */
int  shim_funct(const char *name, void (**funct)(void *, long), void **arg);
/* Everything the component has printed since the last call (one line each,
 * errors prefixed "ERROR: ").  Returns the number of characters copied. */
int  shim_messages(char *out, int out_size);

#ifdef __cplusplus
}
#endif

#endif /* TIGGY_TEST_HAL_H */
