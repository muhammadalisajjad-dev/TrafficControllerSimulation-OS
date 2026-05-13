#ifndef SHM_H
#define SHM_H

#include "shared_state.h"

int shared_state_init(struct shared_state *s);
void shared_state_destroy(struct shared_state *s);

#endif /* SHM_H */
