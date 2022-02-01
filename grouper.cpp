#include <Arduino.h>

#include "clock.h"
#include "debug.h"
#include "grouper.h"



#define GROUPER_MIN_SNR -20.
#define GROUPER_MAX_SNR 10.
#define GROUPER_GROUPING_DISTANCE 25
#define GROUPER_NONE 255
#define GROUPER_PREV_WEIGHT 0.25



namespace {

uint8_t prevGroupSize = 1;
uint8_t prevNearestGroupDistance = 255;
uint8_t prevNearestGroupSlot = GROUPER_NONE;
int nearestGroupDelta = 0;
uint8_t groupSize = 1;
uint8_t nearestGroupSlot = GROUPER_NONE;
uint8_t nearestGroupDistance = 0;



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
  uint8_t packetGroupSize = packet.getGroupSize();
  uint8_t distance = snrToDistance(snr);

  // TODO: better grouping algorithm?
  if (distance <= GROUPER_GROUPING_DISTANCE) {
    groupSize++;
  }
  else if (nearestGroupSlot == GROUPER_NONE || (
    distance < nearestGroupDistance && packetGroupSize >= prevGroupSize
  )) {
    nearestGroupDistance = distance;
    nearestGroupSlot = packet.getSlot();
  }
}



void completeCycle() {
  if (prevNearestGroupSlot != GROUPER_NONE && nearestGroupSlot != GROUPER_NONE) {
    uint8_t prevPrevDistance = prevNearestGroupDistance;

    prevNearestGroupDistance = (uint8_t) (
      GROUPER_PREV_WEIGHT * prevNearestGroupDistance +
      (1. - GROUPER_PREV_WEIGHT) * nearestGroupDistance +
      0.5 // For rounding
    );
    nearestGroupDelta = (int) prevNearestGroupDistance - prevPrevDistance;
  }
  else if (nearestGroupSlot != GROUPER_NONE) {
    prevNearestGroupDistance = nearestGroupDistance;
    nearestGroupDelta = 0;
  }

  prevGroupSize = groupSize;
  prevNearestGroupSlot = nearestGroupSlot;
  groupSize = 1;
  nearestGroupDistance = 255;
  nearestGroupSlot = GROUPER_NONE;

  DEBUG_PRINT(clock::ticks());
  DEBUG_PRINT(F(" GROUPER group of "));
  DEBUG_PRINT(prevGroupSize);

  if (prevNearestGroupSlot == GROUPER_NONE) {
    DEBUG_PRINTLN(F(" largest"));
  }
  else {
    DEBUG_PRINT(F(" near "));
    DEBUG_PRINT(prevNearestGroupSlot);
    DEBUG_PRINT(F(" distance:"));
    DEBUG_PRINT(prevNearestGroupDistance);
    DEBUG_PRINT(F(" delta:"));
    DEBUG_PRINTLN(nearestGroupDelta);
  }
}



int getNearestGroupDelta() {
  return nearestGroupDelta;
}

uint8_t getNearestGroupDistance() {
  return prevNearestGroupDistance;
}

uint8_t getGroupSize() {
  return prevGroupSize;
}

} // namespace grouper
