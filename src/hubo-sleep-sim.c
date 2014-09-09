#include <string.h>
#include <stdio.h>

#include "hubo-sleep-sim.h"

//#define DEBUG_SLEEP 0

extern ach_channel_t chan_hubo_ref;      // Feed-Forward (Reference)
extern ach_channel_t chan_hubo_state;    // Feed-Back (State)

/*sleeps for <t> seconds of sim time.*/
void hubo_sleep(double t, struct hubo_state *H_state, size_t fs){
	int r = ach_get( &chan_hubo_state, H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );
	if(ACH_OK != r) {
		assert( sizeof(*H_state) == fs );
	}
	double target = t + H_state->time;
#if DEBUG_SLEEP > 0
	double start = H_state->time;
	printf("Target = %lf\n", target);
#endif
	while(H_state->time < target){
#if DEBUG_SLEEP > 0
		printf("current = %lf\n", H_state->time);
#endif
		usleep(15000);  //waiting 5000 would repeat same sim time thrice, so usleep(15000) seems a good starting place
		int r = ach_get( &chan_hubo_state, H_state, sizeof(*H_state), &fs, NULL, ACH_O_LAST );
		if(ACH_OK != r) {
			assert( sizeof(*H_state) == fs );
		}
	}
#if DEBUG_SLEEP > 0
	double elapsed = H_state->time - start;
	printf("time %lf\n", elapsed);
#endif
}
