#include <Arduino.h>

#include "clock.h"
#include "debug.h"
#include "grouper.h"



#define GROUPER_MIN_SNR -20.
#define GROUPER_MAX_SNR 10.
#define GROUPER_GROUPING_DISTANCE 25
#define GROUPER_NONE 255
#define GROUPER_MAX_NEIGHBORS 70 // This should match the TDMA setup



namespace {

struct NeighborInfo {
  uint8_t distance = 255;
  uint8_t lastUpdate = 0;
  uint8_t neighbor = GROUPER_NONE;
  uint8_t distanceToNeighbor = 255;
};

NeighborInfo neighbors[GROUPER_MAX_NEIGHBORS];

uint8_t cycleCounter = 0;
uint8_t prevClosestSlot = GROUPER_NONE;
uint8_t prevGroupSize = 1;
uint8_t closestSlot = GROUPER_NONE;
uint8_t closestGroup = GROUPER_NONE;
uint8_t groupSize = 1;



uint8_t snrToDistance(float snr) {
  if (snr <= GROUPER_MIN_SNR) {
    return 255;
  }
  if (snr >= GROUPER_MAX_SNR) {
    return 0;
  }
  return 255. - (
    (snr - GROUPER_MIN_SNR) * 255. / (GROUPER_MAX_SNR - GROUPER_MIN_SNR)
  ) + 0.5; // For rounding
}

} // namespace



namespace grouper {

void processPacket(const packet::Packet& packet, float snr) {
  if (packet.getSlot() >= GROUPER_MAX_NEIGHBORS) {
    DEBUG_PRINT(F("GROUPER no such slot "));
    DEBUG_PRINTLN(packet.getSlot());
    return;
  }

  uint8_t slot = packet.getSlot();
  uint8_t distance = snrToDistance(snr);
  NeighborInfo& entry = neighbors[slot];

  entry.distance = distance; // TODO: hysteresis?
  entry.lastUpdate = cycleCounter;
  entry.neighbor = packet.getClosestSlot();
  entry.distanceToNeighbor = packet.getClosestDistance();

  if (closestSlot == GROUPER_NONE || distance < neighbors[closestSlot].distance) {
    closestSlot = slot;
  }

  if (packet.getGroupSize() > prevGroupSize && (
    closestGroup == GROUPER_NONE || distance < neighbors[closestGroup].distance
  )) {
    closestGroup = slot;
  }

  if (distance <= GROUPER_GROUPING_DISTANCE) {
    groupSize++;
  }
}



void completeCycle() {
  DEBUG_PRINT(clock::ticks());
  DEBUG_PRINT(F(" GROUPER "));

  if (closestGroup == GROUPER_NONE) {
    DEBUG_PRINT(F(" in largest group size:"));
    DEBUG_PRINTLN(groupSize);
  }
  else if (neighbors[closestGroup].neighbor != GROUPER_NONE) {
    NeighborInfo& otherNeighbor = neighbors[neighbors[closestGroup].neighbor];
    double r1 = neighbors[closestGroup].distance;
    double r2 = otherNeighbor.distance;
    double baseline = neighbors[closestGroup].distanceToNeighbor;

    // Only use data from this cycle or the previous
    // TODO: how to tell the difference between initialized to zero and updated at zero?
    if (cycleCounter - otherNeighbor.lastUpdate < 2) {
      double x = (r1 * r1 - r2 * r2 + baseline * baseline) / (2. * baseline);
      double y = sqrt(r1 * r1 - x * x);

      DEBUG_PRINT(F("closest group has "));
      DEBUG_PRINT(closestGroup);
      DEBUG_PRINT(F(" and "));
      DEBUG_PRINT(neighbors[closestGroup].neighbor);
      DEBUG_PRINT(F(" r1:"));
      DEBUG_PRINT(r1);
      DEBUG_PRINT(F(" r2:"));
      DEBUG_PRINT(r2);
      DEBUG_PRINT(F(" U:"));
      DEBUG_PRINT(baseline);
      DEBUG_PRINT(F(" coordinate:"));
      DEBUG_PRINT(x);
      DEBUG_PRINT(F(","));
      DEBUG_PRINTLN(y);
    }
    else {
      DEBUG_PRINTLN(F("data too stale for trilateration"));
    }
  }
  else {
      DEBUG_PRINTLN(F("not enough data for trilateration"));
  }

  prevClosestSlot = closestSlot;
  prevGroupSize = groupSize;
  closestSlot = GROUPER_NONE;
  groupSize = 1;
  closestGroup = GROUPER_NONE;
  cycleCounter++;
}



uint8_t getClosestSlot() {
  return prevClosestSlot;
}



uint8_t getClosestDistance() {
  return neighbors[prevClosestSlot].distance;
}



uint8_t getGroupSize() {
  return prevGroupSize;
}

} // namespace grouper
