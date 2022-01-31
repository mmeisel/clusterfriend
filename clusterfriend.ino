// Required libraries:
// - Low-Power v1.81.0 (https://github.com/rocketscream/Low-Power)
// - RadioLib v5.1.0 (https://github.com/jgromes/RadioLib)
#include <LowPower.h>
#include <RadioLib.h>

#include "debug.h"
#include "clock.h"
#include "packet.h"
#include "tdma.h"



// Networking parameters
#define RFM_FREQ 915.0      // In MHz
#define RFM_BW 125.0        // In kHz, default 125
#define RFM_OUTPUT_POWER 2  // 2..20 dBm, 10 dBm is default
#define LORA_SF 7           // 7-12, default 9
#define LORA_PREAMBLE 8     // Default is 8 (or maybe 16?)
#define LORA_CR 7           // Translates to 4/x, default 7
#define LORA_SYNC_WORD 0xcf // Default is RADIOLIB_SX127X_SYNC_WORD (0x12)
#define LORA_ENABLE_CRC true

// Pins
#define RFM_RST_PIN 2
#define RFM_G0_PIN 3
#define RFM_CS_PIN 4
#define LED_PIN 8

// The built-in max() macro doesn't work for globals in some versions of the Arduino library
// (e.g. MiniCore)
#define MY_MAX(a, b) ((b) > (a) ? (b) : (a))

// Computed from the networking parameters and packet size
// See https://www.rfwireless-world.com/calculators/LoRaWAN-Airtime-calculator.html
constexpr unsigned long PACKET_AIRTIME_TICKS = (unsigned long) (
   (CLOCK_TICKS_PER_SECOND / 1000.) * // Convert milliseconds to ticks
   ((1 << LORA_SF) / RFM_BW) * (
    (LORA_PREAMBLE + 4.25) + // Preamble
    (8.0 + MY_MAX(
      0.0,
      ceil((8.0 * sizeof(packet::Packet) - 4.0 * LORA_SF + 44.0) / (4.0 * LORA_SF)) * LORA_CR
    )) // Payload
  ) + 0.5 // For rounding
);

// Instance of radio driver over SPI
RFM95 radio = new Module(RFM_CS_PIN, RFM_G0_PIN, RFM_RST_PIN);

// Interrupt handler states
volatile bool dataReceived = false;
volatile unsigned long receiveTime = 0UL;

// Disable receive interrupt when it's not needed
volatile bool enableReceiveInterrupt = true;

packet::Packet packetBuffer;
tdma::State tdmaState = tdma::State::cycleComplete;



void setup() {
  DEBUG_BEGIN(57600);
  clock::start();

  pinMode(LED_PIN, OUTPUT);

  // Manually reset RFM95W
  pinMode(RFM_RST_PIN, OUTPUT);
  digitalWrite(RFM_RST_PIN, LOW);
  delay(10);
  digitalWrite(RFM_RST_PIN, HIGH);
  delay(10);

  // Initialize radio
  // See https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem
  int radioState = radio.begin(
    RFM_FREQ,
    RFM_BW,
    LORA_SF,
    LORA_CR,
    LORA_SYNC_WORD,
    RFM_OUTPUT_POWER,
    LORA_PREAMBLE,
    0 // Automatic gain control
  );

  if (radioState != RADIOLIB_ERR_NONE) {
    DEBUG_PRINT(F("Could not initialize RFM95, code "));
    DEBUG_PRINTLN(radioState);
    while (true);
  }

  radioState = radio.setCRC(LORA_ENABLE_CRC);

  if (radioState != RADIOLIB_ERR_NONE) {
    DEBUG_PRINT(F("Could not set CRC on RFM95, code "));
    DEBUG_PRINTLN(radioState);
    while (true);
  }

  DEBUG_PRINT(F("RFM95 initialized, packet airtime "));
  DEBUG_PRINT(PACKET_AIRTIME_TICKS);
  DEBUG_PRINTLN(F(" ticks"));

  // The radio makes a great random number generator
  randomSeed(
    (unsigned long) radio.randomByte() << 24 |
    (unsigned long) radio.randomByte() << 16 |
    (unsigned long) radio.randomByte() << 8 |
    (unsigned long) radio.randomByte()
  );

  // Set receive interrupt handler and start listening
  radio.setDio0Action(handleReceive);
  radioState = radio.startReceive();

  if (radioState == RADIOLIB_ERR_NONE) {
    DEBUG_PRINTLN(F("Listening..."));
  }
  else {
    DEBUG_PRINT(F("Could not start receive mode on RFM95, code "));
    DEBUG_PRINTLN(radioState);
    while (true);
  }

  tdma::start(PACKET_AIRTIME_TICKS);
}



void loop() {
  if (dataReceived) {
    enableReceiveInterrupt = false;
    dataReceived = false;

    if (receive()) {
      tdma::processPacket(packetBuffer, receiveTime);
    }

    enableReceiveInterrupt = true;
    radio.startReceive();
  }

  tdma::State newState = tdma::checkState();

  if (newState != tdmaState) {
    tdmaState = newState;

    if (newState == tdma::State::cycleStarted) {
      radio.startReceive();
    }
    else if (newState == tdma::State::txReady) {
      digitalWrite(LED_PIN, HIGH);
      transmit();
      tdma::txComplete();
      radio.startReceive();
    }
    else if (newState == tdma::State::cycleComplete) {
      digitalWrite(LED_PIN, LOW);

      // Make sure we complete any radio operation that's currently ongoing
      waitForRadio();
      radio.sleep();
    }
  }

  DEBUG_FLUSH();
  goToSleep();
}



void handleReceive() {
  if (enableReceiveInterrupt) {
    receiveTime = clock::ticks();
    dataReceived = true;
  }
}



void goToSleep() {
  clock::waitForSync();
  LowPower.powerSave(SLEEP_FOREVER, ADC_OFF, BOD_OFF, TIMER2_ON);
  clock::waitForSync();
}



void waitForRadio() {
  // Only bits 0, 1, and 3 are meaningful according to the datasheet, if they're all clear, the
  // radio isn't in use.
  while ((radio.getModemStatus() & 0x0b)) {
    yield();
  }
}



bool receive() {
  int radioState = radio.readData((uint8_t*) &packetBuffer, sizeof(packet::Packet));

  DEBUG_PRINT(receiveTime);

  if (radioState == RADIOLIB_ERR_NONE) {
    DEBUG_PRINT(F(" RECEIVED "));
    packetBuffer.debugPrint();
    DEBUG_PRINTLN();
  }
  else {
    DEBUG_PRINT(F(" RECEIVE ERROR code "));
    DEBUG_PRINTLN(radioState);
  }

  return radioState == RADIOLIB_ERR_NONE;
}



void transmit() {
  waitForRadio();

  packet::Packet packet;
  long delayTicks = tdma::getSlotTicksElapsed();

  if (delayTicks > (long) CLOCK_MICROS_TO_TICKS(PACKET_MAX_DELAY_MICROS)) {
    // The delay is too large to represent in the packet, we've certainly missed our slot anyway,
    // just skip this transmission
    DEBUG_PRINT(clock::ticks());
    DEBUG_PRINT(F(" Skipping tx delayTicks:"));
    DEBUG_PRINTLN(delayTicks);
    return;
  }

  packet.setSlot(tdma::getSlotNumber());
  packet.setDelayTicks(max(0L, delayTicks));
  packet.setClosestSlot(grouper::getClosestSlot());
  packet.setClosestDistance(grouper::getClosestDistance());
  packet.setGroupSize(grouper::getGroupSize());

  // Send data, disabling interrupt while tranmitting (otherwise it's triggered on TX done)
  bool receiveInterruptPrevState = enableReceiveInterrupt;

  enableReceiveInterrupt = false;
  int radioState = radio.transmit((uint8_t*) &packet, sizeof(packet::Packet));
  enableReceiveInterrupt = receiveInterruptPrevState;

  if (radioState == RADIOLIB_ERR_NONE) {
    DEBUG_PRINT(clock::ticks());
    DEBUG_PRINT(F(" SENT "));
    packet.debugPrint();
    DEBUG_PRINTLN();
  }
  else {
    DEBUG_PRINT(F("Could not transmit, code "));
    DEBUG_PRINTLN(radioState);
  }
}
