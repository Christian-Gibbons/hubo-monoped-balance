#ifndef HUBO_SLEEP_SIM
#define HUBO_SLEEP_SIM

#ifdef __cplusplus
extern "C" {
#endif

/* Required Hubo Headers */
#include <hubo.h>

/* For Ach IPC */
#include <errno.h>
#include <fcntl.h>
#include <assert.h>
#include <unistd.h>
#include <pthread.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
#include "ach.h"

#define DEBUG_SLEEP 0

void hubo_sleep(double t, struct hubo_state *H_state, size_t fs);

#ifdef __cplusplus
}
#endif

#endif //HUBO_SLEEP_SIM
