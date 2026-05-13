/* Parent simulation – pthread vehicles; parking semaphores; fork+pipes controllers. */
#include "common.h"
#include "controller.h"
#include "gui.h"
#include "ipc.h"
#include "shm.h"
#include <errno.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

typedef struct {
  sem_t spots;
  sem_t queue;
} parking_lot_t;

typedef struct {
  int id;
  enum vehicle_type vt;
  enum approach_dir origin;
  enum dest_rel dest_rel;
  int movement;
  int priority;
  unsigned start_delay_us;
  enum intersection_id first_ix;
  enum intersection_id second_ix;
  int uses_connector;
} vehicle_spec;

typedef struct {
  vehicle_spec spec;
  struct shared_state *shm;
  parking_lot_t *lots;
} vehicle_args;

static volatile sig_atomic_t g_shutdown;
static struct shared_state *g_shm;
static pid_t g_pid_f10;
static pid_t g_pid_f11;

static int phase_allows_movement(enum light_phase ph, int mv) {
  if (ph == LIGHT_PHASE_ALL_RED)
    return 0;
  int ap = mv / 3;
  if (ph == LIGHT_PHASE_NS)
    return ap == AP_N || ap == AP_S;
  if (ph == LIGHT_PHASE_EW)
    return ap == AP_E || ap == AP_W;
  return 0;
}

static uint32_t try_grant_movement(uint32_t active, int mv) {
  unsigned bit = MOV_BITS(mv);
  if (active == 0)
    return bit;
  if (is_straight_mv(mv)) {
    int opp = movement_opposite_straight(mv);
    if (opp >= 0 && active == MOV_BITS(opp))
      return active | bit;
  }
  return 0;
}

static int is_emergency_vt(enum vehicle_type t) {
  return t == VT_AMBULANCE || t == VT_FIRETRUCK;
}

static int may_try_parking(enum vehicle_type t) {
  return t == VT_BUS || t == VT_CAR || t == VT_BIKE || t == VT_TRACTOR;
}

static void on_sigint(int sig) {
  (void)sig;
  g_shutdown = 1;
  if (g_shm)
    atomic_store(&g_shm->shutdown_requested, 1);
  /* Unsafe to pthread_cond_broadcast from a signal handler; vehicles use
   * pthread_cond_timedwait so they observe shutdown within ~200ms. */
}

/* Like pthread_cond_wait but wakes periodically so SIGINT + shutdown_requested
 * cannot strand threads forever without an async-unsafe broadcast. */
static void cond_wait_light_timed(struct shared_state *shm) {
  struct timespec abstime;
  if (clock_gettime(CLOCK_REALTIME, &abstime) != 0) {
    pthread_cond_wait(&shm->cv_light, &shm->mtx);
    return;
  }
  abstime.tv_nsec += 200000000L; /* 200 ms */
  if (abstime.tv_nsec >= 1000000000L) {
    abstime.tv_sec += abstime.tv_nsec / 1000000000L;
    abstime.tv_nsec %= 1000000000L;
  }
  (void)pthread_cond_timedwait(&shm->cv_light, &shm->mtx, &abstime);
}

/* Shutdown-aware wait for a parking spot (avoids indefinite sem_wait on SIGINT). */
static int parking_wait_for_spot(sem_t *sem, struct shared_state *shm) {
  while (!g_shutdown && !atomic_load(&shm->shutdown_requested)) {
    if (sem_trywait(sem) == 0)
      return 1;
    usleep(50000); /* 50ms poll interval */
  }
  return 0; /* shutdown happened */
}

static int parking_attempt(struct shared_state *shm, parking_lot_t *lot, enum intersection_id ix,
                           const vehicle_spec *sp) {
  if (!may_try_parking(sp->vt))
    return 0;

  if ((rand() % 100) >= 42)
    return 0;

  printf("[V%02d %s] Parking TRY at %s before crossing\n", sp->id,
         vehicle_type_name(sp->vt), ix == IX_F10 ? "F10" : "F11");
  fflush(stdout);

  if (sem_trywait(&lot[ix].spots) == 0) {
    atomic_fetch_sub(&shm->park[ix].spots_free, 1);
    atomic_fetch_add(&shm->stat_park_success, 1);
    {
      int occ = PARKING_SPOTS - (int)atomic_load(&shm->park[ix].spots_free);
      float px, py;
      gui_layout_parking_spot(ix, occ > 0 ? occ - 1 : 0, &px, &py);
      gui_vehicle_target(sp->id, px, py, (int)sp->vt, 1);
    }
    printf("[V%02d %s] Parking ENTER immediate spot (spots_free~%d queue~%d)\n", sp->id,
           vehicle_type_name(sp->vt), atomic_load(&shm->park[ix].spots_free),
           atomic_load(&shm->park[ix].queue_free_slots));
    fflush(stdout);
    /* Slow pacing for parking dwell (immediate spot path) */
    usleep((useconds_t)(400000 + rand() % 400000));
    sem_post(&lot[ix].spots);
    atomic_fetch_add(&shm->park[ix].spots_free, 1);
    printf("[V%02d %s] Parking LEAVE\n", sp->id, vehicle_type_name(sp->vt));
    fflush(stdout);
    {
      float ax, ay;
      gui_layout_spawn(ix, sp->origin, &ax, &ay);
      gui_vehicle_target(sp->id, ax, ay, (int)sp->vt, 1);
    }
    return 1;
  }

  if (sem_trywait(&lot[ix].queue) != 0) {
    atomic_fetch_add(&shm->stat_park_failed_full, 1);
    printf("[V%02d %s] Parking FAIL - bounded queue full\n", sp->id,
           vehicle_type_name(sp->vt));
    fflush(stdout);
    return -1;
  }

  atomic_fetch_sub(&shm->park[ix].queue_free_slots, 1);
  atomic_fetch_add(&shm->stat_park_queue_wait, 1);
  {
    float qx, qy;
    gui_layout_parking_queue(ix, &qx, &qy);
    gui_vehicle_target(sp->id, qx, qy, (int)sp->vt, 1);
  }
  printf("[V%02d %s] Parking QUEUE slot acquired; waiting for spot (queue~%d)\n", sp->id,
         vehicle_type_name(sp->vt), atomic_load(&shm->park[ix].queue_free_slots));
  fflush(stdout);

  if (!parking_wait_for_spot(&lot[ix].spots, shm)) {
    /* Shutdown while waiting for a spot — release queue slot and exit */
    sem_post(&lot[ix].queue);
    atomic_fetch_add(&shm->park[ix].queue_free_slots, 1);
    return -1;
  }
  atomic_fetch_sub(&shm->park[ix].spots_free, 1);
  sem_post(&lot[ix].queue);
  atomic_fetch_add(&shm->park[ix].queue_free_slots, 1);

  atomic_fetch_add(&shm->stat_park_success, 1);
  {
    int occ = PARKING_SPOTS - (int)atomic_load(&shm->park[ix].spots_free);
    float px, py;
    gui_layout_parking_spot(ix, occ > 0 ? occ - 1 : 0, &px, &py);
    gui_vehicle_target(sp->id, px, py, (int)sp->vt, 1);
  }
  printf("[V%02d %s] Parking ENTER from queue\n", sp->id, vehicle_type_name(sp->vt));
  fflush(stdout);

  /* Slow pacing for parking dwell (entered from queue) */
  usleep((useconds_t)(400000 + rand() % 400000));

  sem_post(&lot[ix].spots);
  atomic_fetch_add(&shm->park[ix].spots_free, 1);
  printf("[V%02d %s] Parking LEAVE\n", sp->id, vehicle_type_name(sp->vt));
  fflush(stdout);
  {
    float ax, ay;
    gui_layout_spawn(ix, sp->origin, &ax, &ay);
    gui_vehicle_target(sp->id, ax, ay, (int)sp->vt, 1);
  }
  return 1;
}

static void cross_intersection(struct shared_state *shm, enum intersection_id ix,
                               const vehicle_spec *sp, int emergency_mode) {
  int mv = sp->movement;

  pthread_mutex_lock(&shm->mtx);
  atomic_fetch_add(&shm->ix[ix].vehicles_waiting, 1);
  if (sp->vt == VT_BUS)
    atomic_fetch_add(&shm->bus_waiting_hint[ix], 1);

  while (!g_shutdown && !atomic_load(&shm->shutdown_requested)) {
    enum light_phase ph = shm->ix[ix].phase;

    int phase_ok = emergency_mode ? 1 : phase_allows_movement(ph, mv);
    if (!emergency_mode && atomic_load(&shm->emergency_hold[ix]))
      phase_ok = 0;

    uint32_t active = shm->ix[ix].active_move_mask;
    uint32_t granted = 0;

    if (emergency_mode)
      granted = try_grant_movement(active, mv);
    else if (phase_ok && !atomic_load(&shm->emergency_hold[ix]))
      granted = try_grant_movement(active, mv);

    if (granted != 0) {
      shm->ix[ix].active_move_mask = granted;
      atomic_fetch_sub(&shm->ix[ix].vehicles_waiting, 1);
      if (sp->vt == VT_BUS)
        atomic_fetch_sub(&shm->bus_waiting_hint[ix], 1);
      pthread_cond_broadcast(&shm->cv_light);
      pthread_mutex_unlock(&shm->mtx);
      goto crossing;
    }

    cond_wait_light_timed(shm);
  }

  atomic_fetch_sub(&shm->ix[ix].vehicles_waiting, 1);
  if (sp->vt == VT_BUS)
    atomic_fetch_sub(&shm->bus_waiting_hint[ix], 1);
  pthread_mutex_unlock(&shm->mtx);
  return;

crossing:
  printf("[V%02d %s] IX %s CROSS START mv=%d (%s)\n", sp->id, vehicle_type_name(sp->vt),
         ix == IX_F10 ? "F10" : "F11", mv,
         emergency_mode ? "EMERGENCY" : "normal");
  fflush(stdout);
  {
    float cx, cy;
    gui_layout_center(ix, &cx, &cy);
    gui_vehicle_target(sp->id, cx, cy, (int)sp->vt, 1);
  }

  /* Slow pacing for crossing time inside intersection */
  usleep((useconds_t)(300000 + rand() % 200000));
  pthread_mutex_lock(&shm->mtx);
  shm->ix[ix].active_move_mask &= ~MOV_BITS(mv);
  pthread_cond_broadcast(&shm->cv_light);
  pthread_mutex_unlock(&shm->mtx);

  atomic_fetch_add(&shm->stat_crossings_done, 1);

  printf("[V%02d %s] IX %s CROSS DONE\n", sp->id, vehicle_type_name(sp->vt),
         ix == IX_F10 ? "F10" : "F11");
  fflush(stdout);
  {
    float ex, ey;
    gui_layout_post_cross(ix, sp->origin, sp->dest_rel, &ex, &ey);
    gui_vehicle_target(sp->id, ex, ey, (int)sp->vt, 1);
  }
}

/* Shutdown-safe: never block indefinitely on emergency_path_clear_sem. */
static void emergency_wait_path_clear(struct shared_state *shm, enum intersection_id ix) {
  while (!g_shutdown && !atomic_load(&shm->shutdown_requested)) {
    if (sem_trywait(&shm->emergency_path_clear_sem[ix]) == 0)
      return;
    usleep(50000);
  }
}

static void emergency_preclear_peer_if_needed(struct shared_state *shm, const vehicle_spec *sp,
                                              enum intersection_id coming_from_ix,
                                              enum intersection_id target_ix) {
  if (!is_emergency_vt(sp->vt))
    return;
  if (target_ix == IX_COUNT)
    return;

  if (coming_from_ix == IX_F10 && target_ix == IX_F11) {
    printf("[V%02d %s] Emergency preclear: requesting F11 cleared via pipe (Ctrl F10->F11)\n",
           sp->id, vehicle_type_name(sp->vt));
    fflush(stdout);
    gui_emergency_line("EMERGENCY: preclear F11 (F10->F11 pipe)");
    atomic_store(&shm->em_request_clear_f11, sp->id);
    emergency_wait_path_clear(shm, IX_F11);
    if (g_shutdown || atomic_load(&shm->shutdown_requested)) {
      printf("[V%02d %s] Emergency preclear: aborted (shutdown)\n", sp->id,
             vehicle_type_name(sp->vt));
      fflush(stdout);
      return;
    }
    printf("[V%02d %s] Emergency preclear: F11 semaphore signaled - path clear\n", sp->id,
           vehicle_type_name(sp->vt));
    fflush(stdout);
    gui_emergency_line("EMERGENCY: F11 path clear (proceed)");
  } else if (coming_from_ix == IX_F11 && target_ix == IX_F10) {
    printf("[V%02d %s] Emergency preclear: requesting F10 cleared via pipe (Ctrl F11->F10)\n",
           sp->id, vehicle_type_name(sp->vt));
    fflush(stdout);
    gui_emergency_line("EMERGENCY: preclear F10 (F11->F10 pipe)");
    atomic_store(&shm->em_request_clear_f10, sp->id);
    emergency_wait_path_clear(shm, IX_F10);
    if (g_shutdown || atomic_load(&shm->shutdown_requested)) {
      printf("[V%02d %s] Emergency preclear: aborted (shutdown)\n", sp->id,
             vehicle_type_name(sp->vt));
      fflush(stdout);
      return;
    }
    printf("[V%02d %s] Emergency preclear: F10 semaphore signaled - path clear\n", sp->id,
           vehicle_type_name(sp->vt));
    fflush(stdout);
    gui_emergency_line("EMERGENCY: F10 path clear (proceed)");
  }
}

static void emergency_release_peer(struct shared_state *shm, const vehicle_spec *sp,
                                   enum intersection_id released_ix) {
  if (!is_emergency_vt(sp->vt))
    return;
  if (released_ix == IX_F11)
    atomic_store(&shm->em_release_f11, sp->id);
  else if (released_ix == IX_F10)
    atomic_store(&shm->em_release_f10, sp->id);
}

static void *vehicle_thread_main(void *arg) {
  vehicle_args *va = (vehicle_args *)arg;
  vehicle_spec sp = va->spec;
  struct shared_state *shm = va->shm;
  parking_lot_t *lots = va->lots;

  usleep(sp.start_delay_us);

  if (g_shutdown || atomic_load(&shm->shutdown_requested))
    return NULL;

  printf("[V%02d %s] SPAWN origin=%d dest=%s prio=%d route ix:%d%s%s\n", sp.id,
         vehicle_type_name(sp.vt), sp.origin,
         sp.dest_rel == DEST_STRAIGHT ? "S" : sp.dest_rel == DEST_LEFT ? "L" : "R", sp.priority,
         sp.first_ix,
         sp.uses_connector ? "->" : "",
         sp.uses_connector ? (sp.second_ix == IX_F11 ? "F11" : "F10") : "");
  fflush(stdout);
  {
    float sx, sy;
    gui_layout_spawn(sp.first_ix, sp.origin, &sx, &sy);
    gui_vehicle_target(sp.id, sx, sy, (int)sp.vt, 1);
  }

  if (!is_emergency_vt(sp.vt))
    (void)parking_attempt(shm, lots, sp.first_ix, &sp);

  if (sp.uses_connector && is_emergency_vt(sp.vt))
    emergency_preclear_peer_if_needed(shm, &sp, sp.first_ix, sp.second_ix);

  cross_intersection(shm, sp.first_ix, &sp, is_emergency_vt(sp.vt));

  if (!sp.uses_connector) {
    gui_vehicle_target(sp.id, 0.f, 0.f, (int)sp.vt, 0);
    return NULL;
  }

  printf("[V%02d %s] Connector travel F10<->F11\n", sp.id, vehicle_type_name(sp.vt));
  fflush(stdout);
  {
    float cx, cy;
    gui_layout_connector(0.35f, &cx, &cy);
    gui_vehicle_target(sp.id, cx, cy, (int)sp.vt, 1);
  }
  /* Slow pacing: connector midpoint wait */
  usleep((useconds_t)(200000 + rand() % 100000));
  {
    float cx, cy;
    gui_layout_connector(0.85f, &cx, &cy);
    gui_vehicle_target(sp.id, cx, cy, (int)sp.vt, 1);
  }
  /* Slow pacing: connector approach to second IX */
  usleep((useconds_t)(200000 + rand() % 100000));

  if (!is_emergency_vt(sp.vt))
    (void)parking_attempt(shm, lots, sp.second_ix, &sp);

  cross_intersection(shm, sp.second_ix, &sp, is_emergency_vt(sp.vt));

  if (is_emergency_vt(sp.vt))
    emergency_release_peer(shm, &sp, sp.second_ix);

  gui_vehicle_target(sp.id, 0.f, 0.f, (int)sp.vt, 0);
  return NULL;
}

static void random_route(vehicle_spec *sp) {
  memset(sp, 0, sizeof(*sp));
  sp->origin = (enum approach_dir)(rand() % AP_COUNT);
  sp->dest_rel = (enum dest_rel)(rand() % 3);
  sp->movement = movement_from_parts(sp->origin, sp->dest_rel);

  sp->uses_connector = rand() % 2;
  if (sp->uses_connector) {
    sp->first_ix = (rand() % 2) ? IX_F10 : IX_F11;
    sp->second_ix = sp->first_ix == IX_F10 ? IX_F11 : IX_F10;
  } else {
    sp->first_ix = (rand() % 2) ? IX_F10 : IX_F11;
    sp->second_ix = IX_COUNT;
  }
}

static void build_vehicle_specs(int n, vehicle_spec *out) {
  int assigned_emergency = 0;

  for (int i = 0; i < n; i++) {
    vehicle_spec *sp = &out[i];
    random_route(sp);
    sp->id = i + 1;

    sp->vt = (enum vehicle_type)(i % VT_COUNT);

    if (is_emergency_vt(sp->vt))
      sp->priority = 100;
    else if (sp->vt == VT_BUS)
      sp->priority = 50;
    else if (sp->vt == VT_TRACTOR || sp->vt == VT_BIKE)
      sp->priority = 10;
    else
      sp->priority = 25;

    /* Staggered spawns spread over longer wall time */
    sp->start_delay_us = (unsigned)(rand() % 1500000);

    if (is_emergency_vt(sp->vt)) {
      if (!assigned_emergency && sp->uses_connector) {
        sp->first_ix = IX_F10;
        sp->second_ix = IX_F11;
        assigned_emergency = 1;
      } else if (!assigned_emergency && i == n - 1) {
        sp->vt = VT_AMBULANCE;
        sp->uses_connector = 1;
        sp->first_ix = IX_F10;
        sp->second_ix = IX_F11;
        assigned_emergency = 1;
      }
    }
  }
}

static void parking_init(parking_lot_t *lots, struct shared_state *shm) {
  for (int i = 0; i < IX_COUNT; i++) {
    if (sem_init(&lots[i].spots, 0, PARKING_SPOTS) != 0) {
      perror("sem_init spots");
      exit(EXIT_FAILURE);
    }
    if (sem_init(&lots[i].queue, 0, PARKING_QUEUE_SLOTS) != 0) {
      perror("sem_init queue");
      exit(EXIT_FAILURE);
    }
    atomic_store(&shm->park[i].spots_free, PARKING_SPOTS);
    atomic_store(&shm->park[i].queue_free_slots, PARKING_QUEUE_SLOTS);
  }
}

static void parking_destroy(parking_lot_t *lots) {
  for (int i = 0; i < IX_COUNT; i++) {
    sem_destroy(&lots[i].spots);
    sem_destroy(&lots[i].queue);
  }
}

/* ---- GUI C accessors (implemented here so gui.cpp avoids C11 atomics in headers) ---- */
int gui_c_park_spots_free(struct shared_state *s, enum intersection_id ix) {
  return atomic_load(&s->park[ix].spots_free);
}

int gui_c_park_queue_slots_free(struct shared_state *s, enum intersection_id ix) {
  return atomic_load(&s->park[ix].queue_free_slots);
}

int gui_c_ix_phase(struct shared_state *s, enum intersection_id ix) {
  int ph;
  pthread_mutex_lock(&s->mtx);
  ph = (int)s->ix[ix].phase;
  pthread_mutex_unlock(&s->mtx);
  return ph;
}

int gui_c_shutdown_requested(struct shared_state *s) {
  return atomic_load(&s->shutdown_requested) != 0;
}

void gui_c_broadcast_shutdown(struct shared_state *s) {
  atomic_store(&s->shutdown_requested, 1);
  pthread_mutex_lock(&s->mtx);
  pthread_cond_broadcast(&s->cv_light);
  pthread_mutex_unlock(&s->mtx);
}

static void *status_printer(void *arg) {
  struct shared_state *shm = (struct shared_state *)arg;
  while (!g_shutdown && !atomic_load(&shm->shutdown_requested)) {
    printf(
        "--- STATUS park[F10] spots=%d queue_free=%d | park[F11] spots=%d queue_free=%d | "
        "wait[F10]=%d wait[F11]=%d ---\n",
        atomic_load(&shm->park[IX_F10].spots_free),
        atomic_load(&shm->park[IX_F10].queue_free_slots),
        atomic_load(&shm->park[IX_F11].spots_free),
        atomic_load(&shm->park[IX_F11].queue_free_slots),
        atomic_load(&shm->ix[IX_F10].vehicles_waiting),
        atomic_load(&shm->ix[IX_F11].vehicles_waiting));
    fflush(stdout);
    for (int k = 0; k < 20 && !g_shutdown && !atomic_load(&shm->shutdown_requested); k++)
      usleep(500000); /* ~0.5s per tick → ~10s between STATUS lines */
  }
  return NULL;
}

int run_simulation(int argc, char **argv) {
  int nveh = NUM_VEHICLES;
  if (argc >= 2) {
    nveh = atoi(argv[1]);
    if (nveh <= 0 || nveh > NUM_VEHICLES)
      nveh = NUM_VEHICLES;
  }

  srand((unsigned)time(NULL) ^ (unsigned)getpid());

  signal(SIGINT, on_sigint);

  struct shared_state *shm =
      mmap(NULL, sizeof(*shm), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
  if (shm == MAP_FAILED) {
    perror("mmap");
    return EXIT_FAILURE;
  }
  g_shm = shm;

  if (shared_state_init(shm) != 0) {
    fprintf(stderr, "shared_state_init failed\n");
    g_shm = NULL;
    munmap(shm, sizeof(*shm));
    return EXIT_FAILURE;
  }

  atomic_store(&shm->em_request_clear_f11, 0);
  atomic_store(&shm->em_release_f11, 0);
  atomic_store(&shm->em_request_clear_f10, 0);
  atomic_store(&shm->em_release_f10, 0);
  atomic_store(&shm->bus_waiting_hint[IX_F10], 0);
  atomic_store(&shm->bus_waiting_hint[IX_F11], 0);

  parking_lot_t lots[IX_COUNT];
  parking_init(lots, shm);

  /* GUI render thread: reads phases from shm; never touches parking semaphores or pipes. */
  gui_start_render_thread(shm);

  int p_to_f11[2];
  int p_to_f10[2];
  if (pipe(p_to_f11) != 0) {
    perror("pipe");
    goto fail_after_gui_parking;
  }
  if (pipe(p_to_f10) != 0) {
    perror("pipe");
    close(p_to_f11[0]);
    close(p_to_f11[1]);
    goto fail_after_gui_parking;
  }

  pid_t pid_f10 = fork();
  if (pid_f10 == 0) {
    close(p_to_f11[0]);
    close(p_to_f10[1]);
    controller_main(IX_F10, shm, p_to_f10[0], p_to_f11[1]);
  }
  if (pid_f10 < 0) {
    perror("fork");
    close(p_to_f11[0]);
    close(p_to_f11[1]);
    close(p_to_f10[0]);
    close(p_to_f10[1]);
    goto fail_after_gui_parking;
  }

  pid_t pid_f11 = fork();
  if (pid_f11 == 0) {
    close(p_to_f11[1]);
    close(p_to_f10[0]);
    controller_main(IX_F11, shm, p_to_f11[0], p_to_f10[1]);
  }
  if (pid_f11 < 0) {
    perror("fork");
    {
      struct ctrl_msg shutdown_msg = {.type = MSG_SHUTDOWN, .vehicle_id = 0, .toward_ix = 0};
      ssize_t ws = write(p_to_f10[1], &shutdown_msg, sizeof(shutdown_msg));
      (void)ws;
    }
    close(p_to_f11[0]);
    close(p_to_f11[1]);
    close(p_to_f10[0]);
    close(p_to_f10[1]);
    (void)waitpid(pid_f10, NULL, 0);
    goto fail_after_gui_parking;
  }

  /* Parent: close read ends only; keep write ends open for MSG_SHUTDOWN to controllers */
  close(p_to_f11[0]);
  close(p_to_f10[0]);
  int fd_to_f10 = p_to_f10[1];
  int fd_to_f11 = p_to_f11[1];
  g_pid_f10 = pid_f10;
  g_pid_f11 = pid_f11;

  vehicle_spec *specs = calloc((size_t)nveh, sizeof(vehicle_spec));
  pthread_t *threads = calloc((size_t)nveh, sizeof(pthread_t));
  vehicle_args *args = calloc((size_t)nveh, sizeof(vehicle_args));
  if (!specs || !threads || !args) {
    perror("calloc");
    {
      struct ctrl_msg shutdown_msg = {.type = MSG_SHUTDOWN, .vehicle_id = 0, .toward_ix = 0};
      ssize_t wa = write(fd_to_f10, &shutdown_msg, sizeof(shutdown_msg));
      ssize_t wb = write(fd_to_f11, &shutdown_msg, sizeof(shutdown_msg));
      (void)wa;
      (void)wb;
    }
    close(fd_to_f10);
    close(fd_to_f11);
    (void)waitpid(pid_f10, NULL, 0);
    (void)waitpid(pid_f11, NULL, 0);
    free(specs);
    free(threads);
    free(args);
    goto fail_after_gui_parking;
  }

  build_vehicle_specs(nveh, specs);

  pthread_t printer;
  pthread_create(&printer, NULL, status_printer, shm);

  for (int i = 0; i < nveh; i++) {
    args[i].spec = specs[i];
    args[i].shm = shm;
    args[i].lots = lots;
    pthread_create(&threads[i], NULL, vehicle_thread_main, &args[i]);
  }

  for (int i = 0; i < nveh; i++)
    pthread_join(threads[i], NULL);

  printf("All %d vehicle threads joined - shutting down controllers.\n", nveh);
  fflush(stdout);

  /* Explicit pipe shutdown so controllers exit without waiting on poll latency only */
  {
    struct ctrl_msg shutdown_msg = {.type = MSG_SHUTDOWN, .vehicle_id = 0, .toward_ix = 0};
    ssize_t w10 = write(fd_to_f10, &shutdown_msg, sizeof(shutdown_msg));
    ssize_t w11 = write(fd_to_f11, &shutdown_msg, sizeof(shutdown_msg));
    (void)w10;
    (void)w11;
  }
  close(fd_to_f10);
  close(fd_to_f11);

  waitpid(g_pid_f10, NULL, 0);
  waitpid(g_pid_f11, NULL, 0);

  gui_stop_join();

  g_shutdown = 1;
  pthread_join(printer, NULL);

  printf("\n===== SUMMARY =====\n");
  printf("Crossings completed : %llu\n", (unsigned long long)atomic_load(&shm->stat_crossings_done));
  printf("Parking successes   : %llu\n", (unsigned long long)atomic_load(&shm->stat_park_success));
  printf("Parking waited queue: %llu\n", (unsigned long long)atomic_load(&shm->stat_park_queue_wait));
  printf("Parking failed full : %llu\n", (unsigned long long)atomic_load(&shm->stat_park_failed_full));
  printf("Emergency pipe clears: %llu\n",
         (unsigned long long)atomic_load(&shm->stat_emergency_preemptions));
  fflush(stdout);

  parking_destroy(lots);
  shared_state_destroy(shm);
  munmap(shm, sizeof(*shm));
  g_shm = NULL;
  free(specs);
  free(threads);
  free(args);
  return EXIT_SUCCESS;

fail_after_gui_parking:
  gui_stop_join();
  parking_destroy(lots);
  shared_state_destroy(shm);
  munmap(shm, sizeof(*shm));
  g_shm = NULL;
  return EXIT_FAILURE;
}
