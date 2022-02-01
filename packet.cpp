#include <Arduino.h>

#include "clock.h"
#include "debug.h"
#include "packet.h"



namespace packet {

uint8_t Packet::getSlot() const {
  return this->slot_;
}

void Packet::setSlot(uint8_t slot) {
  this->slot_ = slot;
}



unsigned long Packet::getDelayTicks() const {
  return CLOCK_MICROS_TO_TICKS((unsigned long) this->delay_ << 8);
}

void Packet::setDelayTicks(unsigned long ticks) {
  // We discard the lowest 8 bits, giving a resolution of 256 microseconds, which should be
  // precise enough for our purposes.
  this->delay_ = (CLOCK_TICKS_TO_MICROS(ticks) >> 8) & 0xffUL;
}



uint8_t Packet::getGroupSize() const {
  return this->groupSize_;
}

void Packet::setGroupSize(uint8_t groupSize) {
  this->groupSize_ = groupSize;
}



void Packet::debugPrint() const {
  DEBUG_PRINT(F("f:0x"));
  DEBUG_PRINT(this->flags_, HEX);
  DEBUG_PRINT(F(" s:"));
  DEBUG_PRINT(this->slot_);
  DEBUG_PRINT(F(" d:"));
  DEBUG_PRINT(this->delay_ << 8);
  DEBUG_PRINT(F("us gs:"));
  DEBUG_PRINT(this->groupSize_);
  DEBUG_PRINT(F(" r:"));
  DEBUG_PRINT(this->reserved_);
}

} // namespace packet
