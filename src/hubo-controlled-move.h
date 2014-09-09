#ifndef HUBO_CONTROLLED_MOVE
#define HUBO_CONTROLLED_MOVE

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

#include "hubo-defines.h"

#if TIME_BASE == SIM_TIME
#include "hubo-sleep-sim.h"
#elif TIME_BASE == REAL_TIME
//put in real-time sleep function in the future
#endif

#define DEBUG_CONTROLLED_MOVE 0

typedef struct {
	int j; //joint
	double p; //position
} joint_pos;

void controlled_move(joint_pos *p, int joint_num, int step_num, struct hubo_state *H_state, struct hubo_ref *H_ref, size_t fs);

#ifdef __cplusplus
}
#endif

#endif //HUBO_CONTROLLED_MOVE
