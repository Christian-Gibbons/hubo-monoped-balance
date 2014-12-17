==========================
ECE 499 Final Exam Program
==========================

Implementation strategy: The upper body and lower body are being asked to carry out independent tasks, so my approach was to have one process to control the waving hand, and another process to control the legs.  In order to keep these processes from interfering with each other by overwriting each other's commands, I wanted to make use of the feedforward channel to pass in the most recent joint references and only replace the specific joint references the process is concerned with.  In order to make the hand stop waving based on when the lower body stops moving up and down, I created a channel between the upper body and lower body processes so that the lower body could tell the upper body when it had finished moving up and down and waited the required three seconds, allowing the upper body to break out of the waving loop.

For reasons I have been unable to determine, Hubo does not move every joint that it is supposed to.  The lower-body algorithm calls for Hubo to push it's hips over one foot, however only the right leg and ankle move while the left leg and ankle remain stationary.  I enabled the debugging mode in my controlled move algorithm, and it confirms that the H_ref buffer is being updated for all the necessary joints before being put on the ach channel, so I don't know where else to try to debug the problem.  Due to this bug, Hubo is unable to balance on one foot and tips over.

Instructions for use
compiling: navigate to base directory "hubo-monoped-balance" and run:
	$ make
	$ make chan
Start Hubo simulator:
	$ hubo-ach sim openhubo physics
run upper body process:
	$ ./hubo-hello-world
run lower body process:
	$ ./hubo-monoped-balance

To clean up source files:
	$ make clean
To clean up interprocess channel:
	$ make chan_clean
