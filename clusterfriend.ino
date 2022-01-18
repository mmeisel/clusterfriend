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
#define RFM_OUTPUT_POWER 10 // 2..20 dBm, 10 dBm is default
#define LORA_SF 7           // 7-12, default 9
#define LORA_PREAMBLE 8     // Default is 8 (or maybe 16?)
#define LORA_CR 7           // Translates to 4/x, default 7

// Pins
#define RFM_RST_PIN 2
#define RFM_G0_PIN 3
#define RFM_CS_PIN 4
#define LED_PIN 8

// Computed from the networking parameters and packet size
// See https://www.rfwireless-world.com/calculators/LoRaWAN-Airtime-calculator.html
constexpr unsigned long PACKET_AIRTIME_MICROS = (unsigned long) (
  1000.0 * ((1 << LORA_SF) / RFM_BW) * (
    (LORA_PREAMBLE + 4.25) + // Preamble
    (8.0 + max(
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
    RADIOLIB_SX127X_SYNC_WORD,
    RFM_OUTPUT_POWER,
    LORA_PREAMBLE,
    0 // Automatic gain control
  );

  if (radioState != RADIOLIB_ERR_NONE) {
    DEBUG_PRINT(F("Could not initialize RFM95, code "));
    DEBUG_PRINTLN(radioState);
    while (true);
  }

  DEBUG_PRINT(F("RFM95 initialized, packet airtime "));
  DEBUG_PRINTLN(PACKET_AIRTIME_MICROS);

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

  tdma::startup(PACKET_AIRTIME_MICROS);
}



void loop() {
  if (dataReceived) {
    enableReceiveInterrupt = false;
    dataReceived = false;

    receive();
    tdma::processPacket(packetBuffer, receiveTime);

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
    receiveTime = clock::micros();
    dataReceived = true;
  }
}



// TODO: Add a string coersion to the packet class instead
void debugPrintPacketBuffer() {
#if DEBUG
  for (unsigned int i = 0; i < sizeof(packet::Packet) / sizeof(uint8_t); i++) {
    DEBUG_PRINT(F(" 0x"));
    DEBUG_PRINT(((uint8_t*) &packetBuffer)[i], HEX);
  }
  DEBUG_PRINTLN();
#endif
}



void goToSleep() {
  LowPower.powerSave(SLEEP_FOREVER, ADC_OFF, BOD_OFF, TIMER2_ON);
}



void waitForRadio() {
  // Only bits 0, 1, and 3 are meaningful according to the datasheet, if they're all clear, the
  // radio isn't in use.
  while ((radio.getModemStatus() & 0x0b)) {
    yield();
  }
}



void receive() {
  int radioState = radio.readData((uint8_t*) &packetBuffer, sizeof(packet::Packet));

  if (radioState == RADIOLIB_ERR_NONE) {
    DEBUG_PRINT(receiveTime);
    DEBUG_PRINT(F(" RECEIVED"));
    debugPrintPacketBuffer();
  }
  else {
    DEBUG_PRINT(F("Could not receive, code "));
    DEBUG_PRINTLN(radioState);
  }
}



void transmit() {
  waitForRadio();

  long delayMillis = (tdma::getSlotTimeElapsed() + 500UL /* for rounding */) / 1000L;

  if (delayMillis >= 1 << (sizeof(packetBuffer.delayMillis) * 8)) {
    // Too long a delay to put in the packet, we've certainly missed our slot anyway, just skip
    // this transmission
    DEBUG_PRINT(clock::micros());
    DEBUG_PRINT(F(" Skipping tx delayMillis="));
    DEBUG_PRINTLN(delayMillis);
    return;
  }

  packetBuffer.slot = tdma::getSlotNumber();
  packetBuffer.delayMillis = max(0L, delayMillis);
  // These two fields are not currently in use
  packetBuffer.flags = 0;
  packetBuffer.reserved = 0;

  // Send data, disabling interrupt while tranmitting (otherwise it's triggered on TX done)
  bool receiveInterruptPrevState = enableReceiveInterrupt;

  enableReceiveInterrupt = false;
  int radioState = radio.transmit((uint8_t*) &packetBuffer, sizeof(packet::Packet));
  enableReceiveInterrupt = receiveInterruptPrevState;

  if (radioState == RADIOLIB_ERR_NONE) {
    DEBUG_PRINT(clock::micros());
    DEBUG_PRINT(F(" SENT"));
    debugPrintPacketBuffer();
  }
  else {
    DEBUG_PRINT(F("Could not transmit, code "));
    DEBUG_PRINTLN(radioState);
  }
}
