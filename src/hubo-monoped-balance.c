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

#include "final.h"

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

ach_channel_t chan_hubo_final;	// interprocess

int main(int argc, char **argv) {

    /* Open Ach Channel */
    int r = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME , NULL);
    assert( ACH_OK == r );

    r = ach_open(&chan_hubo_state, HUBO_CHAN_STATE_NAME , NULL);
    assert( ACH_OK == r );

	r = ach_open(&chan_hubo_final, FINAL_CHAN_NAME , NULL);
	assert( ACH_OK == r );

    /* Create initial structures to read and write from */
    struct hubo_ref H_ref;
    struct hubo_state H_state;
	struct hubo_final H_final;
//    memset( &H_ref,   0, sizeof(H_ref));
//    memset( &H_state, 0, sizeof(H_state));
	memset( &H_final, 0, sizeof(H_final));

    /* for size check */
    size_t fs;

 //   ach_put( &chan_hubo_ref, &H_ref, sizeof(H_ref));

    /* Get the current feed-forward (state) */
    r = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );
    if(ACH_OK != r) {
        assert( sizeof(H_state) == fs );
    }

	
	joint_pos *jp;
	double hip_angle = -0.14;
	jp = (joint_pos *) malloc(sizeof(joint_pos)*4);

	hubo_sleep(1.0, &H_state, fs);

	printf("Moving hips into position over right foot\n");
	jp[0].j = LHR;
	jp[0].p = -1.0 * hip_angle;
	jp[1].j = RHR;
	jp[1].p = -1.0 * hip_angle;
	jp[2].j = LAR;
	jp[2].p = hip_angle;
	jp[3].j = RAR;
	jp[3].p = hip_angle;
	controlled_move(jp, 4, 12, &H_state, &H_ref, fs, 1);
	hubo_sleep(1.0, &H_state, fs);

	printf("balance on right leg\n");
	double leg_bend_left = -0.8;
	jp[0].j = LHP;
	jp[0].p = leg_bend_left;
	jp[1].j = LKN;
	jp[1].p = -2.0 * leg_bend_left;
	jp[2].j = LAP;
	jp[2].p = leg_bend_left;
	controlled_move(jp, 3, 8, &H_state, &H_ref, fs, 1);
	hubo_sleep(1.0, &H_state, fs);	
	
	printf("squat down on right leg\n");
	double leg_bend_right = -0.65;
	for(int i=0; i<3; i++){ //move up and down three times on one leg
		jp[0].j = RHP;
		jp[0].p = leg_bend_right;
		jp[1].j = RKN;
		jp[1].p = -2.0 * leg_bend_right;
		jp[2].j = RAP;
		jp[2].p = leg_bend_right;
		printf("Timestamp at top: %lf\n", H_state.time);
		controlled_move(jp, 3, 20, &H_state, &H_ref, fs, 1);
		hubo_sleep(0.5, &H_state, fs);
		jp[0].p = -0.05;
		jp[1].p = 0.1;
		jp[2].p = -0.05;
		printf("Timestamp at bottom: %lf\n", H_state.time);
		controlled_move(jp, 3, 20, &H_state, &H_ref, fs, 1);
		hubo_sleep(0.5, &H_state, fs);
	}
	hubo_sleep(3.0, &H_state, fs); //wait three seconds after movement ended
	H_final.stop = 1;
	ach_put( &chan_hubo_final, &H_final, sizeof(H_final)); //send stop signal to hand-waiving process

	printf("Put left leg back down\n");
	jp[0].j = LHP;
	jp[1].j = LKN;
	jp[2].j = LAP;
	controlled_move(jp, 3, 10, &H_state, &H_ref, fs, 1);
	hubo_sleep(0.5, &H_state, fs);

	printf("Back to neutral position\n");
	jp[0].j = RHR;
	jp[0].p = 0.0;
	jp[1].j = LHR;
	jp[1].p = 0.0;
	jp[2].j = RAR;
	jp[2].p = 0.0;
	jp[3].j = LAR;
	jp[3].p = 0.0;
	controlled_move(jp, 4, 24, &H_state, &H_ref, fs, 1);
	hubo_sleep(3.0, &H_state, fs);
	ach_put(&chan_hubo_ref, &H_ref, sizeof(H_ref));
}
