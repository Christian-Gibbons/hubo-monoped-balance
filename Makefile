default: all

CFLAGS := -I./include -g --std=gnu99
CC := gcc

DIR := src
_OBJS := hubo-sleep-sim.o hubo-monoped-balance.o hubo-controlled-move.o
OBJS := $(patsubst %,$(DIR)/%,$(_OBJS))

DEPS := src/hubo-sleep-sim.h src/hubo-controlled-move.h src/hubo-defines.h
BINARIES := hubo-monoped-balance
LIBS := -lach -lm

all : $(OBJS)
	$(CC) $(FLAGS) -o $(BINARIES) $(OBJS) $(LIBS)

$(DIR)/%.o: %.c $(DEPS)
	$(CC) $(CFLAGS) -c $<

clean:
	rm -f $(BINARIES) src/*.o src/*~
