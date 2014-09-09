#include <string.h>
#include <stdio.h>

#include "hubo-controlled-move.h"


extern ach_channel_t chan_hubo_ref;      // Feed-Forward (Reference)
extern ach_channel_t chan_hubo_state;    // Feed-Back (State)

/* cuts movements into smaller steps to help prevent sudden movements */
void controlled_move(joint_pos *p, int joint_num, int step_num, struct hubo_state *H_state, struct hubo_ref *H_ref, size_t fs){
	double step_size[joint_num];
	int r = ach_get( &chan_hubo_state, H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );
	if(ACH_OK != r) {
		assert( sizeof(*H_state) == fs );
	}
	double next_step[joint_num];
	for(int i=0; i<joint_num; i++){
		step_size[i] = (p[i].p-H_state->joint[p[i].j].pos)/step_num;
		next_step[i] = H_state->joint[p[i].j].pos + step_size[i];
#if DEBUG_CONTROLLED_MOVE == 1
		printf("step_size[%d] = %lf\n", i, step_size[i]);
#endif
	}

	int flag = joint_num;
	for(int j=0; j<step_num; j++){
		for(int i=0; i<joint_num; i++){
			next_step[i] += step_size[i];
			H_ref->ref[p[i].j] = next_step[i];
#if DEBUG_CONTROLLED_MOVE == 1
			printf("next_step[%d] = %lf\n", i, next_step[i]);
#endif
		}
		ach_put( &chan_hubo_ref, H_ref, sizeof(*H_ref));
		for(int i=0; i<joint_num; i++){		
			while(fabs(fabs(H_state->joint[p[i].j].pos)-fabs(next_step[i])) > 0.08){
#if DEBUG_CONTROLLED_MOVE == 1
				printf("target pos[%d]: %lf, current pos: %lf current sim-time: %lf\n", i, next_step[i], H_state->joint[p[i].j].pos, H_state->time);
#endif
				usleep(15000);
				int r = ach_get( &chan_hubo_state, H_state, sizeof(*H_state), &fs, NULL, ACH_O_LAST );
				if(ACH_OK != r) {
					assert( sizeof(*H_state) == fs );
				}
			}
		}
	}
}
