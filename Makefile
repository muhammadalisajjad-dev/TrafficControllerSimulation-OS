# OS Project — pthread simulation + optional SFML 2.6 GUI
# Core objects stay C (gcc); main.c is compiled as C++; link with g++ for libstdc++ + SFML.

CC      = gcc
CXX     = g++
CFLAGS  = -std=c11 -Wall -Wextra -D_GNU_SOURCE -pthread -O1
CXXFLAGS = -std=c++17 -Wall -Wextra -pthread -O1

PKG_CFG_SFML = sfml-graphics sfml-window sfml-system
SFML_CFLAGS := $(shell pkg-config --cflags $(PKG_CFG_SFML) 2>/dev/null)
SFML_LIBS   := $(shell pkg-config --libs $(PKG_CFG_SFML) 2>/dev/null)

C_OBJS   = simulation.o controller.o shm.o
CXX_OBJS = main.o gui.o

.PHONY: all clean run

all: simulation

# main.c is C++ source (extern "C" bridge) — force C++ compile of .c file
main.o: main.c
	$(CXX) $(CXXFLAGS) -x c++ -c -o $@ $<

gui.o: gui.cpp gui.h shared_state.h common.h
	$(CXX) $(CXXFLAGS) $(SFML_CFLAGS) -c -o $@ gui.cpp

simulation.o: simulation.c common.h controller.h gui.h shm.h shared_state.h ipc.h
controller.o: controller.c controller.h ipc.h shared_state.h common.h
shm.o: shm.c shm.h shared_state.h common.h

simulation: $(C_OBJS) $(CXX_OBJS)
	@if [ -z "$(SFML_LIBS)" ]; then echo "pkg-config: SFML not found. Install libsfml-dev (2.6)."; exit 1; fi
	$(CXX) $(C_OBJS) $(CXX_OBJS) -pthread -lrt $(SFML_LIBS) -o $@

clean:
	rm -f $(C_OBJS) $(CXX_OBJS) simulation

run: simulation
	./simulation
