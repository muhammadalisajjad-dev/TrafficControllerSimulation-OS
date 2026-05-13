/*
 * Bidirectional pipe protocol between F10 and F11 controllers.
 */
#ifndef IPC_H
#define IPC_H

#include <stdint.h>

enum ctrl_msg_type {
  MSG_NONE = 0,
  /* F10 -> F11 or F11 -> F10: clear all conflicting movements before emergency enters other ix */
  MSG_CLEAR_PATH_FOR_EMERGENCY,
  /* Receiver acknowledges intersection is clear */
  MSG_CLEAR_PATH_ACK,
  /* Sender releases emergency hold on receiver */
  MSG_RELEASE_EMERGENCY_HOLD,
  MSG_SHUTDOWN /* parent asks controller to exit loop */
};

struct ctrl_msg {
  enum ctrl_msg_type type;
  int32_t vehicle_id;
  /* IX_F10 or IX_F11 – which intersection the emergency is heading toward */
  int32_t toward_ix;
};

#endif /* IPC_H */
