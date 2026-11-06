/*
  PULSE-PUCK — ESP32 + HLK-LD2450 24 GHz radar  ->  Bluetooth LE bridge for the PULSE app

  Wiring (LD2450 runs on 5 V, its UART is 3.3 V logic):
    LD2450 5V  -> ESP32 5V / VIN
    LD2450 GND -> ESP32 GND
    LD2450 TX  -> ESP32 GPIO16 (RADAR_RX)
    LD2450 RX  -> ESP32 GPIO17 (RADAR_TX)

  Board: any ESP32 / ESP32-S3 / ESP32-C3 dev board (Arduino-ESP32 core).
  Change RADAR_RX / RADAR_TX if your board uses different free UART pins.

  BLE: Nordic UART Service. Every ~100 ms notifies 18 bytes =
       3 targets x { int16 x_mm, int16 y_mm, int16 speed_cm_s } little-endian.
       Empty target slots are all zeros.
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define RADAR_RX 16
#define RADAR_TX 17

static const char *SVC_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char *TX_UUID  = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

BLECharacteristic *txChar = nullptr;
volatile bool connected = false;

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer *s) override { connected = true; }
  void onDisconnect(BLEServer *s) override { connected = false; s->startAdvertising(); }
};

// LD2450 frame: AA FF 03 00 | 3 x 8 bytes target data | 55 CC  (30 bytes total)
static const uint8_t HDR[4] = {0xAA, 0xFF, 0x03, 0x00};
uint8_t frame[30];
int fpos = 0;
int16_t targets[3][3];          // x, y, speed
unsigned long lastFrameMs = 0, lastSendMs = 0;

// LD2450 signed format: bit15 = 1 means positive, value in low 15 bits; bit15 = 0 means negative
static int16_t decodeSigned(uint16_t raw) {
  return (raw & 0x8000) ? (int16_t)(raw & 0x7FFF) : (int16_t)(-(int16_t)raw);
}

void handleFrame() {
  for (int t = 0; t < 3; t++) {
    const uint8_t *p = frame + 4 + t * 8;
    uint16_t rx = p[0] | (p[1] << 8);
    uint16_t ry = p[2] | (p[3] << 8);
    uint16_t rv = p[4] | (p[5] << 8);
    if (rx == 0 && ry == 0) {           // empty slot
      targets[t][0] = targets[t][1] = targets[t][2] = 0;
    } else {
      targets[t][0] = decodeSigned(rx);
      targets[t][1] = decodeSigned(ry);
      targets[t][2] = decodeSigned(rv);
    }
  }
  lastFrameMs = millis();
}

void readRadar() {
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    if (fpos < 4) {                     // hunt for header
      if (b == HDR[fpos]) frame[fpos++] = b;
      else fpos = (b == HDR[0]) ? (frame[0] = b, 1) : 0;
      continue;
    }
    frame[fpos++] = b;
    if (fpos == 30) {
      if (frame[28] == 0x55 && frame[29] == 0xCC) handleFrame();
      fpos = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(256000, SERIAL_8N1, RADAR_RX, RADAR_TX);

  BLEDevice::init("PULSE-PUCK");
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCB());
  BLEService *svc = server->createService(SVC_UUID);
  txChar = svc->createCharacteristic(TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  txChar->addDescriptor(new BLE2902());
  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("PULSE-PUCK advertising");
}

void loop() {
  readRadar();
  unsigned long now = millis();
  if (connected && now - lastSendMs >= 100) {
    lastSendMs = now;
    int16_t out[9] = {0};
    if (now - lastFrameMs < 1000) memcpy(out, targets, sizeof(out));  // stale radar -> send empties
    txChar->setValue((uint8_t *)out, sizeof(out));                      // ESP32 is little-endian
    txChar->notify();
  }
}
