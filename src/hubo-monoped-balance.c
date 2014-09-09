/* Standard Stuff */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

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

/* For system time */
#include <time.h>

#include "hubo-defines.h"

#include "hubo-controlled-move.h"

#define TIME_BASE SIM_TIME //use SIM_TIME for simulation, and REAL_TIME for a physical robot.

#if TIME_BASE == SIM_TIME
/* For sleep */
#include "hubo-sleep-sim.h"
#elif TIME_BASE == REAL_TIME
//include a real-time sleep function here in the future
#endif
/* Ach Channel IDs */
ach_channel_t chan_hubo_ref;      // Feed-Forward (Reference)
ach_channel_t chan_hubo_state;    // Feed-Back (State)

int main(int argc, char **argv) {

    /* Open Ach Channel */
    int r = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME , NULL);
    assert( ACH_OK == r );

    r = ach_open(&chan_hubo_state, HUBO_CHAN_STATE_NAME , NULL);
    assert( ACH_OK == r );



    /* Create initial structures to read and write from */
    struct hubo_ref H_ref;
    struct hubo_state H_state;
    memset( &H_ref,   0, sizeof(H_ref));
    memset( &H_state, 0, sizeof(H_state));

    /* for size check */
    size_t fs;

    ach_put( &chan_hubo_ref, &H_ref, sizeof(H_ref));

    /* Get the current feed-forward (state) */
    r = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );
    if(ACH_OK != r) {
        assert( sizeof(H_state) == fs );
    }


#if 0
	struct timespec t[2];
	clock_gettime(CLOCK_MONOTONIC, &t[0]);
	usleep(500000);
	sleep(2);
	clock_gettime(CLOCK_MONOTONIC, &t[1]);
	long int tdiffn = t[1].tv_nsec - t[0].tv_nsec;
	time_t tdiffs = t[1].tv_sec - t[0].tv_sec;
	printf("time elapsed: %d.%ld seconds\n", (int) tdiffs, tdiffn);
#endif

	joint_pos *jp;
#if 1
	printf("Moving hips into position\n");
	double hip_angle = -0.8;
	jp = (joint_pos *) malloc(sizeof(joint_pos)*4);
	jp[0].j = RHR;
	jp[0].p = hip_angle;
	jp[1].j = LHR;
	jp[1].p = hip_angle;
	jp[2].j = RAR;
	jp[2].p = -1.0 * hip_angle;
	jp[3].j = LAR;
	jp[3].p = -1.0* hip_angle + 0.2;
	controlled_move(jp, 4, 12, &H_state, &H_ref, fs);
	hubo_sleep(1.5, &H_state, fs);
#endif
#if 1
	printf("balance on one leg\n");
	double leg_angle_r = 2.5;
	jp[0].j = RHP;
	jp[0].p = -1.0 * leg_angle_r;
	jp[1].j = RKN;
	jp[1].p = leg_angle_r;
	controlled_move(jp, 2, 20, &H_state, &H_ref, fs);
#endif
	printf("squat down on one leg\n");
	hubo_sleep(1.5, &H_state, fs);
	jp[0].j = LHP;
	jp[0].p = -1.0;
	jp[1].j = LKN;
	jp[1].p = 2.0;
	jp[2].j = LAP;
	jp[2].p = -1.0;
	controlled_move(jp, 3, 10, &H_state, &H_ref, fs);
	

#if 0
    /* Set Left Elbow Bend (LEB) and Right Shoulder Pitch (RSP) to  -0.2 rad and 0.1 rad respectively*/
    H_ref.ref[LEB] = -0.2;
    H_ref.ref[RSP] = 0.1;

    /* Print out the actual position of the LEB */
    double posLEB = H_state.joint[LEB].pos;
    printf("Joint = %f\r\n",posLEB);

    /* Print out the Left foot torque in X */
    double mxLeftFT = H_state.ft[HUBO_FT_L_FOOT].m_x;
    printf("Mx = %f\r\n", mxLeftFT);

    /* Write to the feed-forward channel */
#endif
}
