default: all

CFLAGS := -I./include -g --std=gnu99
CC := gcc

DIR := src
_BAL_OBJS := hubo-sleep-sim.o hubo-monoped-balance.o hubo-controlled-move.o
BAL_OBJS := $(patsubst %,$(DIR)/%,$(_BAL_OBJS))
_WAVE_OBJS := hubo-sleep-sim.o hubo-hello-world.o hubo-controlled-move.o 
WAVE_OBJS := $(patsubst %,$(DIR)/%,$(_WAVE_OBJS))

DEPS :=  src/hubo-sleep-sim.h src/hubo-controlled-move.h src/hubo-defines.h src/final.h 
BAL_BINARIES := hubo-monoped-balance
WAVE_BINARIES := hubo-hello-world
LIBS := -lach -lm

FINAL_CHAN='final-chan'

all : $(BAL_OBJS) $(WAVE_OBJS)
	$(CC) $(FLAGS) -o $(BAL_BINARIES) $(BAL_OBJS) $(LIBS)
	$(CC) $(FLAGS) -o $(WAVE_BINARIES) $(WAVE_OBJS) $(LIBS)

$(DIR)/%.o: %.c $(DEPS)
	$(CC) $(CFLAGS) -c $<

chan:
	ach -1 -C $(FINAL_CHAN) -m 30 -n 3000000
	sudo chmod 777 /dev/shm/achshm-*

chan_clean:
	sudo rm -r /dev/shm/achshm*

clean:
	rm -f $(BAL_BINARIES) $(WAVE_BINARIES) src/*.o src/*~


