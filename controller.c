/*
 * Traffic controller process – one per intersection (F10 / F11).
 */
#include "controller.h"
#include "ipc.h"
#include "shared_state.h"
#include <errno.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

static ssize_t read_full(int fd, void *buf, size_t n) {
  char *p = (char *)buf;
  size_t left = n;
  while (left > 0) {
    ssize_t r = read(fd, p, left);
    if (r < 0) {
      if (errno == EINTR)
        continue;
      return -1;
    }
    if (r == 0)
      return (ssize_t)(n - left);
    p += (size_t)r;
    left -= (size_t)r;
  }
  return (ssize_t)n;
}

static ssize_t write_full(int fd, const void *buf, size_t n) {
  const char *p = (const char *)buf;
  size_t left = n;
  while (left > 0) {
    ssize_t w = write(fd, p, left);
    if (w < 0) {
      if (errno == EINTR)
        continue;
      return -1;
    }
    if (w == 0)
      return -1;
    p += (size_t)w;
    left -= (size_t)w;
  }
  return (ssize_t)n;
}

static void broadcast_phase(struct shared_state *shm) {
  pthread_mutex_lock(&shm->mtx);
  pthread_cond_broadcast(&shm->cv_light);
  pthread_mutex_unlock(&shm->mtx);
}

static void handle_incoming_msg(struct shared_state *shm, enum intersection_id self,
                               int fd_read_peer, int fd_write_peer, const struct ctrl_msg *m) {
  (void)fd_read_peer;

  switch (m->type) {
  case MSG_CLEAR_PATH_FOR_EMERGENCY:
    printf("[CTRL %s] PIPE recv CLEAR_PATH_FOR_EMERGENCY (veh %d → ix %d)\n",
           self == IX_F10 ? "F10" : "F11", (int)m->vehicle_id, (int)m->toward_ix);
    fflush(stdout);

    atomic_store(&shm->emergency_hold[self], 1);
    pthread_mutex_lock(&shm->mtx);
    shm->ix[self].phase = LIGHT_PHASE_ALL_RED;
    pthread_mutex_unlock(&shm->mtx);
    broadcast_phase(shm);

    for (;;) {
      pthread_mutex_lock(&shm->mtx);
      uint32_t am = shm->ix[self].active_move_mask;
      pthread_mutex_unlock(&shm->mtx);
      if (am == 0)
        break;
      usleep(5000);
      if (atomic_load(&shm->shutdown_requested))
        return;
    }

    printf("[CTRL %s] intersection empty — posting emergency_path_clear_sem\n",
           self == IX_F10 ? "F10" : "F11");
    fflush(stdout);
    sem_post(&shm->emergency_path_clear_sem[self]);

    atomic_fetch_add(&shm->stat_emergency_preemptions, 1);

    {
      struct ctrl_msg ack = {.type = MSG_CLEAR_PATH_ACK,
                             .vehicle_id = m->vehicle_id,
                             .toward_ix = (int32_t)self};
      if (write_full(fd_write_peer, &ack, sizeof(ack)) < 0)
        perror("[CTRL] write_full CLEAR_PATH_ACK");
    }
    break;

  case MSG_CLEAR_PATH_ACK:
    printf("[CTRL %s] PIPE recv CLEAR_PATH_ACK (veh %d)\n",
           self == IX_F10 ? "F10" : "F11", (int)m->vehicle_id);
    fflush(stdout);
    break;

  case MSG_RELEASE_EMERGENCY_HOLD:
    printf("[CTRL %s] PIPE recv RELEASE_EMERGENCY_HOLD\n",
           self == IX_F10 ? "F10" : "F11");
    fflush(stdout);
    atomic_store(&shm->emergency_hold[self], 0);
    pthread_mutex_lock(&shm->mtx);
    if (shm->ix[self].phase == LIGHT_PHASE_ALL_RED)
      shm->ix[self].phase = LIGHT_PHASE_NS;
    pthread_mutex_unlock(&shm->mtx);
    broadcast_phase(shm);
    break;

  case MSG_SHUTDOWN:
    printf("[CTRL %s] received MSG_SHUTDOWN\n", self == IX_F10 ? "F10" : "F11");
    fflush(stdout);
    atomic_store(&shm->shutdown_requested, 1);
    break;

  default:
    break;
  }
}

static void maybe_forward_emergency_requests(struct shared_state *shm, enum intersection_id self,
                                             int fd_write_peer) {
  if (self == IX_F10) {
    int v = atomic_exchange(&shm->em_request_clear_f11, 0);
    if (v != 0) {
      struct ctrl_msg m = {.type = MSG_CLEAR_PATH_FOR_EMERGENCY,
                           .vehicle_id = v,
                           .toward_ix = IX_F11};
      if (write_full(fd_write_peer, &m, sizeof(m)) < 0)
        perror("[CTRL F10] write_full CLEAR toward F11");
      printf("[CTRL F10] PIPE send CLEAR toward F11 (veh %d)\n", v);
      fflush(stdout);
    }
    int rel = atomic_exchange(&shm->em_release_f11, 0);
    if (rel != 0) {
      struct ctrl_msg m = {.type = MSG_RELEASE_EMERGENCY_HOLD, .vehicle_id = rel, .toward_ix = IX_F11};
      if (write_full(fd_write_peer, &m, sizeof(m)) < 0)
        perror("[CTRL F10] write_full RELEASE_HOLD toward F11");
      printf("[CTRL F10] PIPE send RELEASE_HOLD toward F11\n");
      fflush(stdout);
    }
  } else {
    int v = atomic_exchange(&shm->em_request_clear_f10, 0);
    if (v != 0) {
      struct ctrl_msg m = {.type = MSG_CLEAR_PATH_FOR_EMERGENCY,
                           .vehicle_id = v,
                           .toward_ix = IX_F10};
      if (write_full(fd_write_peer, &m, sizeof(m)) < 0)
        perror("[CTRL F11] write_full CLEAR toward F10");
      printf("[CTRL F11] PIPE send CLEAR toward F10 (veh %d)\n", v);
      fflush(stdout);
    }
    int rel = atomic_exchange(&shm->em_release_f10, 0);
    if (rel != 0) {
      struct ctrl_msg m = {.type = MSG_RELEASE_EMERGENCY_HOLD, .vehicle_id = rel, .toward_ix = IX_F10};
      if (write_full(fd_write_peer, &m, sizeof(m)) < 0)
        perror("[CTRL F11] write_full RELEASE_HOLD toward F10");
      printf("[CTRL F11] PIPE send RELEASE_HOLD toward F10\n");
      fflush(stdout);
    }
  }
}

static uint64_t now_us_monotonic(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}

_Noreturn void controller_main(enum intersection_id self, struct shared_state *shm,
                               int fd_read_peer, int fd_write_peer) {
  struct pollfd pfd = {.fd = fd_read_peer, .events = POLLIN};

  uint64_t next_phase_us = now_us_monotonic() + PHASE_MS_DEFAULT;

  printf("[CTRL %s] started pid=%d\n", self == IX_F10 ? "F10" : "F11", (int)getpid());
  fflush(stdout);

  for (;;) {
    if (atomic_load(&shm->shutdown_requested))
      break;

    maybe_forward_emergency_requests(shm, self, fd_write_peer);

    (void)poll(&pfd, 1, 25);
    if (pfd.revents & POLLIN) {
      struct ctrl_msg m;
      ssize_t r = read_full(fd_read_peer, &m, sizeof(m));
      if (r == (ssize_t)sizeof(m))
        handle_incoming_msg(shm, self, fd_read_peer, fd_write_peer, &m);
      else
        break;
    }

    uint64_t now = now_us_monotonic();

    int bonus_us = atomic_load(&shm->bus_waiting_hint[self]) * 120000;
    if (bonus_us > 700000)
      bonus_us = 700000;

    if (now >= next_phase_us && !atomic_load(&shm->emergency_hold[self])) {
      pthread_mutex_lock(&shm->mtx);
      enum light_phase ph = shm->ix[self].phase;
      if (ph == LIGHT_PHASE_NS)
        shm->ix[self].phase = LIGHT_PHASE_EW;
      else if (ph == LIGHT_PHASE_EW)
        shm->ix[self].phase = LIGHT_PHASE_NS;
      /* ALL_RED unchanged until RELEASE */
      enum light_phase newph = shm->ix[self].phase;
      pthread_mutex_unlock(&shm->mtx);

      broadcast_phase(shm);

      printf("[CTRL %s] phase -> %s\n", self == IX_F10 ? "F10" : "F11",
             newph == LIGHT_PHASE_NS ? "NS_GREEN" :
             newph == LIGHT_PHASE_EW ? "EW_GREEN" :
                                     "ALL_RED");
      fflush(stdout);

      next_phase_us = now + (uint64_t)PHASE_MS_DEFAULT + (uint64_t)bonus_us;
    }
  }

  printf("[CTRL %s] exiting\n", self == IX_F10 ? "F10" : "F11");
  fflush(stdout);
  _exit(0);
}
