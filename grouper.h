#ifndef __CF_GROUPER_H__
#define __CF_GROUPER_H__

#include "packet.h"

namespace grouper {

// Call when a packet arrives
void processPacket(const packet::Packet& packet, float snr);

// Call at the end of each cycle
void completeCycle();

// Get the data collected in the previous cycle
uint8_t getClosestSlot();
uint8_t getClosestDistance();
uint8_t getGroupSize();

} // namespace grouper

#endif
