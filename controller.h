#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "common.h"
#include "shared_state.h"

_Noreturn void controller_main(enum intersection_id self, struct shared_state *shm,
                               int fd_read_peer, int fd_write_peer);

#endif /* CONTROLLER_H */
