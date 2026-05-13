/*
 * SFML 2.6 GUI — C API callable from simulation.c (pthread vehicle threads).
 * Rendering runs on a dedicated thread; updates use g_gui_mtx, not shm->mtx,
 * so vehicle threads never nest GUI locks with intersection mutex rules.
 */
#ifndef GUI_H
#define GUI_H

#include "common.h"

struct shared_state;

#ifdef __cplusplus
extern "C" {
#endif

/* C accessors in simulation.c — keeps C++ GUI away from C11 _Atomics in shared_state.h */
int gui_c_park_spots_free(struct shared_state *shm, enum intersection_id ix);
int gui_c_park_queue_slots_free(struct shared_state *shm, enum intersection_id ix);
int gui_c_ix_phase(struct shared_state *shm, enum intersection_id ix);
int gui_c_shutdown_requested(struct shared_state *shm);
void gui_c_broadcast_shutdown(struct shared_state *shm);

void gui_start_render_thread(struct shared_state *shm);
void gui_stop_join(void);
void gui_vehicle_target(int vehicle_id, float x, float y, int vehicle_type, int visible);
void gui_emergency_line(const char *text);

void gui_layout_spawn(enum intersection_id ix, enum approach_dir origin, float *out_x,
                      float *out_y);
void gui_layout_center(enum intersection_id ix, float *out_x, float *out_y);
void gui_layout_post_cross(enum intersection_id ix, enum approach_dir origin, enum dest_rel dr,
                           float *out_x, float *out_y);
void gui_layout_parking_spot(enum intersection_id ix, int slot_index, float *out_x, float *out_y);
void gui_layout_connector(float progress_0_1, float *out_x, float *out_y);
void gui_layout_parking_queue(enum intersection_id ix, float *out_x, float *out_y);

#ifdef __cplusplus
}
#endif

#endif /* GUI_H */
