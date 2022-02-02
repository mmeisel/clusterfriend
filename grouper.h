#ifndef __CF_GROUPER_H__
#define __CF_GROUPER_H__

#include "packet.h"

namespace grouper {

// Call when a packet arrives
void processPacket(const packet::Packet& packet, float snr);

// Call at the end of each cycle
void completeCycle();

// Get the data collected in the previous cycle
int getNearestGroupDelta();
uint8_t getNearestGroupDistance();
uint8_t getGroupSize();
bool isInLargestGroup();

} // namespace grouper

#endif
