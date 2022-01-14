#ifndef __CF_PACKET_H__
#define __CF_PACKET_H__

namespace packet {
  struct Packet {
    uint8_t slot;
    uint8_t total_slots;
    uint16_t delay_millis;
  };
}

#endif
