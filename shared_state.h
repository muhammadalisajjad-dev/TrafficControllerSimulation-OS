/*
 * mmap(MAP_SHARED) layout used by parent + both controller processes + vehicle threads (parent only).
 */
#ifndef SHARED_STATE_H
#define SHARED_STATE_H

#include "common.h"
#include <pthread.h>
#include <semaphore.h>
#include <stdatomic.h>
#include <stdint.h>

enum light_phase {
  LIGHT_PHASE_NS = 0,
  LIGHT_PHASE_EW,
  LIGHT_PHASE_ALL_RED,
};

struct ix_snapshot {
  enum light_phase phase;
  uint32_t active_move_mask; /* movements currently occupying ix */
  _Atomic int vehicles_waiting;
};

struct parking_snap {
  /* Mirrors parking semaphores for console visibility */
  _Atomic int spots_free;
  _Atomic int queue_free_slots;
};

struct shared_state {
  pthread_mutex_t mtx;
  pthread_cond_t cv_light; /* controllers broadcast phase change */

  struct ix_snapshot ix[IX_COUNT];

  /* Emergency coordination flags (set by vehicle threads in parent, observed by controllers) */
  _Atomic int shutdown_requested;
  _Atomic int emergency_clear_pending[IX_COUNT]; /* receiver must go ALL_RED until clear */
  _Atomic int emergency_hold[IX_COUNT]; /* ix frozen ALL_RED until released */
  /*
   * Posted by receiver controller once intersection has no active movements.
   * Initialized with process-shared sem_init (second argument 1), initial value 0.
   */
  sem_t emergency_path_clear_sem[IX_COUNT];

  struct parking_snap park[IX_COUNT];

  /* Stats (parent threads increment; printed at shutdown) */
  _Atomic uint64_t stat_crossings_done;
  _Atomic uint64_t stat_park_success;
  _Atomic uint64_t stat_park_queue_wait;
  _Atomic uint64_t stat_park_failed_full;
  _Atomic uint64_t stat_emergency_preemptions;

  /*
   * Emergency routing helpers – vehicle threads set; originating controller forwards on pipe.
   * Values are vehicle IDs (non-zero) while a request is pending.
   */
  _Atomic int em_request_clear_f11;
  _Atomic int em_release_f11;
  _Atomic int em_request_clear_f10;
  _Atomic int em_release_f10;

  /* Incremented while a bus thread is blocked waiting to cross (hint for longer green slice). */
  _Atomic int bus_waiting_hint[IX_COUNT];
};

#define SHARED_STATE_SIZE sizeof(struct shared_state)

#endif /* SHARED_STATE_H */
