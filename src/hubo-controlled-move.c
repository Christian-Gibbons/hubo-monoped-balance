#include <string.h>
#include <stdio.h>

#include "hubo-controlled-move.h"
#include "hubo-sleep-sim.h"

extern ach_channel_t chan_hubo_ref;      // Feed-Forward (Reference)
extern ach_channel_t chan_hubo_state;    // Feed-Back (State)

/* cuts movements into smaller steps to help prevent sudden movements */
void controlled_move(joint_pos *p, int joint_num, int step_num, struct hubo_state *H_state, struct hubo_ref *H_ref, size_t fs, char chan_put){
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
	for(int j=0; j<step_num; j++){
		if(chan_put == 1){
			int r = ach_get( &chan_hubo_state, H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );
			if(ACH_OK != r) {
				assert( sizeof(*H_state) == fs );
			}
			for(int i=0; i<HUBO_JOINT_COUNT; i++){
				for(int k=0; k<joint_num; k++){
					if(p[k].j == i){
						goto OUT;
					}
				}
				H_ref->ref[i] = H_state->joint[i].ref;
				OUT:;
			}
		}
	}
	for(int j=0; j<step_num; j++){
		for(int i=0; i<joint_num; i++){
			next_step[i] += step_size[i];
			H_ref->ref[p[i].j] = next_step[i];
#if DEBUG_CONTROLLED_MOVE == 1
			printf("current_pos[%d] = %lf, next_step[%d] = %lf\n",i, H_state->joint[p[i].j].ref, i, H_ref->ref[p[i].j]);
#endif
		}
		ach_put( &chan_hubo_ref, H_ref, sizeof(*H_ref));
		hubo_sleep(0.05, H_state, fs);
	}
}

