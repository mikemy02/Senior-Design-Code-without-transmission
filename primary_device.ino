// primary device

#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>

// ---------- Pins ----------
#define RFM95_CS   4
#define RFM95_RST  2
#define RFM95_INT  3

// ---------- Radio ----------
#define RF95_FREQ 915.0

// ---------- Addresses ----------
#define MAIN_ADDR      1
#define SECONDARY_ADDR 2

RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHReliableDatagram manager(rf95, MAIN_ADDR);

// ---------- Timing ----------
const unsigned long DEPTH_PERIOD_MS = 1000;  // 1 Hz
unsigned long lastDepthSend = 0;

// ---------- "Action" ----------
volatile uint32_t actionCounter = 0;

// ---------- Depth source (placeholder) ----------
float getDepthMeters() {
  static float d = 1.00;
  d += 0.01;
  if (d > 2.00) d = 1.00;
  Serial.print("Depth: ");
  Serial.println(d);
  return d;
}

void hardResetRadio() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
}

void setup() {
  Serial.begin(9600);
  delay(1200);
  Serial.println("MAIN boot");

  hardResetRadio();

  if (!manager.init()) {
    Serial.println("LoRa init failed");
    while (1);
  }
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  rf95.setTxPower(23, false);

  Serial.println("MAIN ready");
}

void sendDepth() {
  uint8_t pkt[1 + sizeof(float)];
  pkt[0] = 'D';
  float depth = getDepthMeters();
  memcpy(&pkt[1], &depth, sizeof(float));

  // 1) Manually increment the header ID so the receiver doesn't drop it as a duplicate
  static uint8_t unreliableSeq = 0;
  manager.setHeaderId(++unreliableSeq);

  // 2) Unreliable stream
  manager.sendto(pkt, sizeof(pkt), SECONDARY_ADDR);
  
  // 3) Wait for the transmission to physically finish!
  rf95.waitPacketSent(); 
}

void handleTriggerAndReply(uint8_t from) {
  // 1) local action
  Serial.println("triggered");
  actionCounter++;
  Serial.print("actionCounter = ");
  Serial.println(actionCounter);

  // 2) send DONE back reliably
  uint8_t donePkt[1 + sizeof(uint32_t)];
  donePkt[0] = 'K'; // done
  memcpy(&donePkt[1], &actionCounter, sizeof(uint32_t));

  bool ok = manager.sendtoWait(donePkt, sizeof(donePkt), from);
  if (!ok) {
    Serial.println("Failed to send DONE (no ACK)");
  }
}

void loop() {
  // A) keep streaming depth
  unsigned long now = millis();
  if (now - lastDepthSend >= DEPTH_PERIOD_MS) {
    lastDepthSend = now;
    sendDepth();
  }

  // B) check for incoming commands (short timeout so we don’t stall streaming)
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  uint8_t from;

  if (manager.recvfromAckTimeout(buf, &len, 10, &from)) {
    if (len >= 1 && (char)buf[0] == 'T') {
      handleTriggerAndReply(from);
    }
  }
}