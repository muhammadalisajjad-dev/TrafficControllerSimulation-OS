/*
 * Program entry. Built as C++ (Makefile uses $(CXX) -x c++ on this file) so the linker
 * can pull in gui.cpp + SFML. Forwards to run_simulation() implemented in simulation.c.
 */
extern "C" int run_simulation(int argc, char **argv);

int main(int argc, char **argv) { return run_simulation(argc, argv); }
