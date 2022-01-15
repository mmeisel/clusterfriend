#ifndef __CF_PACKET_H__
#define __CF_PACKET_H__

namespace packet {
  struct Packet {
    uint8_t slot;
    uint8_t delay_millis;
    uint8_t flags;
    uint8_t reserved;
  };
}

#endif
