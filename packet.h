#ifndef __CF_PACKET_H__
#define __CF_PACKET_H__

#define PACKET_MAX_DELAY_MICROS (0xffUL << 8)

namespace packet {
  struct Packet {
    uint8_t slot;
    uint8_t delay;
    uint8_t flags;
    uint8_t reserved;
  };

  inline unsigned long getDelayMicros(const Packet& packet) {
    return (unsigned long) packet.delay << 8;
  }

  inline void setDelayMicros(Packet& packet, unsigned long micros) {
    // We discard the lowest 8 bits, giving a resolution of 256 microseconds, which should be
    // precise enough for our purposes.
    packet.delay = (micros >> 8) & 0xffUL;
  }
}

#endif
