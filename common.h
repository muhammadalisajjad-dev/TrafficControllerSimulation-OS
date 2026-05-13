/*
 * OS Project Spring 2026 – shared definitions (vehicles, movements, IPC).
 */
#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>

#define NUM_VEHICLES 15
#define PARKING_SPOTS 10
#define PARKING_QUEUE_SLOTS 5

#define PHASE_MS_DEFAULT 800000 /* controller cycle slice (microseconds) */

enum vehicle_type {
  VT_AMBULANCE = 0,
  VT_FIRETRUCK,
  VT_BUS,
  VT_CAR,
  VT_BIKE,
  VT_TRACTOR,
  VT_COUNT
};

enum approach_dir {
  AP_N = 0,
  AP_S,
  AP_E,
  AP_W,
  AP_COUNT
};

/* Relative turn when entering the intersection from approach_dir. */
enum dest_rel {
  DEST_STRAIGHT = 0,
  DEST_LEFT,
  DEST_RIGHT
};

enum intersection_id {
  IX_F10 = 0,
  IX_F11,
  IX_COUNT
};

/*
 * Encode movement as index 0..11: approach * 3 + dest_rel
 */
#define MOV_INDEX(ap, dr) ((int)((ap)*3 + (dr)))
#define MOV_BITS(mv) (1u << (mv))

static inline int movement_opposite_straight(int mv) {
  int ap = mv / 3;
  int dr = mv % 3;
  if (dr != DEST_STRAIGHT)
    return -1;
  switch (ap) {
  case AP_N:
    return MOV_INDEX(AP_S, DEST_STRAIGHT);
  case AP_S:
    return MOV_INDEX(AP_N, DEST_STRAIGHT);
  case AP_E:
    return MOV_INDEX(AP_W, DEST_STRAIGHT);
  case AP_W:
    return MOV_INDEX(AP_E, DEST_STRAIGHT);
  default:
    return -1;
  }
}

static inline int movement_from_parts(enum approach_dir ap, enum dest_rel dr) {
  return MOV_INDEX(ap, dr);
}

static inline int is_straight_mv(int mv) {
  return (mv >= 0 && mv < 12 && (mv % 3) == DEST_STRAIGHT);
}

static inline const char *vehicle_type_name(enum vehicle_type t) {
  static const char *names[] = {"Ambulance", "Firetruck", "Bus",
                                "Car",       "Bike",      "Tractor"};
  if ((unsigned)t >= VT_COUNT)
    return "?";
  return names[t];
}

#endif /* COMMON_H */
