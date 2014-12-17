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

#include "hubo-controlled-move.h"
#include "final.h"
/* Ach Channel IDs */
ach_channel_t chan_hubo_ref;      // Feed-Forward (Reference)
ach_channel_t chan_hubo_state;    // Feed-Back (State)

ach_channel_t chan_hubo_final;

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
    memset( &H_ref,   0, sizeof(H_ref));
 //   memset( &H_state, 0, sizeof(H_state));
	memset( &H_final, 0, sizeof(H_final));
    /* for size check */
    size_t fs;
	size_t ff;
	H_final.stop = 0;
	ach_put( &chan_hubo_final, &H_final, sizeof(H_final));

    /* Get the current feed-forward (state) */
    r = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );
    if(ACH_OK != r) {
        assert( sizeof(H_state) == fs );
    }

	joint_pos *jp;
	jp = (joint_pos *) malloc(sizeof(joint_pos)*3);
	jp[0].j = LEB;
	jp[0].p = -1.8;
	jp[1].j = LSP;
	jp[1].p = -1.0;
	controlled_move(jp, 2, 12, &H_state, &H_ref, fs, 1);
	hubo_sleep(0.5, &H_state, fs);
	while(1){
		r = ach_get( &chan_hubo_final, &H_final, sizeof(H_final), &ff, NULL, ACH_O_LAST);
		if(ACH_OK != r) {
			assert( sizeof(H_final) == ff );
		}
		if(H_final.stop){
			break;
		}
		jp[0].j = LSY;
		jp[0].p = -0.8;
		controlled_move(jp, 1, 12, &H_state, &H_ref, fs, 1);
		hubo_sleep(0.5, &H_state, fs);
		jp[0].p = 0.0;
		controlled_move(jp, 1, 12, &H_state, &H_ref, fs, 1);
		hubo_sleep(0.5, &H_state, fs);
	}
	jp[0].j = LEB;
	jp[0].p = 0.0;
	jp[1].j = LSP;
	jp[1].p = 0.0;
	jp[2].j = LSY;
	jp[2].p = 0.0;
	controlled_move(jp, 3, 12, &H_state, &H_ref, fs, 1);
}

