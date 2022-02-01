#ifndef __CF_PACKET_H__
#define __CF_PACKET_H__

#define PACKET_MAX_DELAY_MICROS (0xffUL << 8)

namespace packet {

struct Packet {
public:
  uint8_t getSlot() const;
  void setSlot(uint8_t slot);

  unsigned long getDelayTicks() const;
  void setDelayTicks(unsigned long ticks);

  uint8_t getGroupSize() const;
  void setGroupSize(uint8_t groupSize);

  void debugPrint() const;

private:
  uint8_t flags_ = 0;
  uint8_t slot_ = 0;
  // In increments of 256 microseconds (shift left by 8 to get microseconds)
  uint8_t delay_ = 0;
  uint8_t groupSize_ = 0;
  uint8_t reserved_ = 0;
};

} // namespace packet

#endif
