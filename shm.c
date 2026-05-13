/*
 * Initialize / tear down mmap'd shared state (process-shared mutex, cond, semaphores).
 */
#include "shm.h"
#include <errno.h>
#include <stdlib.h>
#include <string.h>

static void shared_state_zero(struct shared_state *s) { memset(s, 0, sizeof(*s)); }

int shared_state_init(struct shared_state *s) {
  pthread_mutexattr_t mattr;
  pthread_condattr_t cattr;
  int rc;

  shared_state_zero(s);

  pthread_mutexattr_init(&mattr);
  pthread_mutexattr_setpshared(&mattr, PTHREAD_PROCESS_SHARED);
  rc = pthread_mutex_init(&s->mtx, &mattr);
  pthread_mutexattr_destroy(&mattr);
  if (rc != 0)
    return rc;

  pthread_condattr_init(&cattr);
  pthread_condattr_setpshared(&cattr, PTHREAD_PROCESS_SHARED);
  rc = pthread_cond_init(&s->cv_light, &cattr);
  pthread_condattr_destroy(&cattr);
  if (rc != 0)
    return rc;

  for (int i = 0; i < IX_COUNT; i++) {
    s->ix[i].phase = LIGHT_PHASE_NS;
    s->ix[i].active_move_mask = 0;
    atomic_store(&s->ix[i].vehicles_waiting, 0);

    atomic_store(&s->park[i].spots_free, PARKING_SPOTS);
    atomic_store(&s->park[i].queue_free_slots, PARKING_QUEUE_SLOTS);

    atomic_store(&s->emergency_clear_pending[i], 0);
    atomic_store(&s->emergency_hold[i], 0);

    if (sem_init(&s->emergency_path_clear_sem[i], 1, 0) != 0)
      return -1;
  }

  atomic_store(&s->shutdown_requested, 0);
  return 0;
}

void shared_state_destroy(struct shared_state *s) {
  for (int i = 0; i < IX_COUNT; i++)
    sem_destroy(&s->emergency_path_clear_sem[i]);

  pthread_cond_destroy(&s->cv_light);
  pthread_mutex_destroy(&s->mtx);
}
