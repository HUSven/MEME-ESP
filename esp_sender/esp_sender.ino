/*
 * ESP-NOW Transceiver - Apparaat A (Zender/Patiënt)
 * Communicatieprotocol: Mensen Meten v1.0
 * Sensoren: NTC Thermistor (LM358) + Pulse sensor
 */

#include <WiFi.h>
#include <esp_now.h>
#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════════════════════
// NTC THERMISTOR
// ═══════════════════════════════════════════════════════════════════════════════
constexpr int ADC_PIN_TEMP = 0;   // GPIO 0 — temperature
constexpr int AVG_SAMPLES  = 20;

constexpr int TABLE_SIZE = 7;
const float tempTable[TABLE_SIZE][2] = {
  {270.0f,  25.0f},
  {500.0f,  30.0f},
  {1130.0f, 34.0f},
  {1400.0f, 35.0f},
  {2020.0f, 37.0f},
  {2980.0f, 40.0f},
  {4095.0f, 43.0f},
};

float rawToTemp(float raw) {
  if (raw <= tempTable[0][0])              return tempTable[0][1];
  if (raw >= tempTable[TABLE_SIZE - 1][0]) return tempTable[TABLE_SIZE - 1][1];
  for (int i = 0; i < TABLE_SIZE - 1; i++) {
    float r0 = tempTable[i][0], t0 = tempTable[i][1];
    float r1 = tempTable[i+1][0], t1 = tempTable[i+1][1];
    if (raw >= r0 && raw <= r1)
      return t0 + (t1 - t0) * (raw - r0) / (r1 - r0);
  }
  return -1.0f;
}

float readTemperature(float* rawOut = nullptr) {
  long rawSum = 0;
  for (int i = 0; i < AVG_SAMPLES; i++) {
    rawSum += analogRead(ADC_PIN_TEMP);
    delayMicroseconds(200);
  }
  float adcAvg = static_cast<float>(rawSum) / AVG_SAMPLES;
  if (rawOut) *rawOut = adcAvg;
  return rawToTemp(adcAvg);
}

// ═══════════════════════════════════════════════════════════════════════════════
// HEARTBEAT / BPM
// ═══════════════════════════════════════════════════════════════════════════════
const int pulsePin        = 1;
const int baseThreshold   = 500;
const int peakDrop        = 150;
const int WINDOW_MS       = 5000;
const int NO_BEAT_TIMEOUT = 2000;

int  peakValue    = 0;
int  avgPeak      = 550;
int  bpm          = 0;
int  beatCount    = 0;
bool beatDetected = false;
bool signalActive = false;

unsigned long beatTimes[20];
unsigned long lastBeatTime = 0;

void updateHeartbeat() {
  int signal = analogRead(pulsePin);
  unsigned long now = millis();

  // Track & smooth peak
  if (signal > peakValue) peakValue = signal;
  avgPeak = (avgPeak * 9 + peakValue) / 10;

  // Dynamic threshold
  int dynamicThreshold = max(avgPeak - peakDrop, baseThreshold);

  // Beat detection
  if (signal > dynamicThreshold && !beatDetected) {
    beatDetected = true;
    beatTimes[beatCount % 20] = now;
    beatCount++;
    lastBeatTime = now;
    signalActive = true;
    peakValue = 0;
  }
  if (signal < dynamicThreshold) beatDetected = false;

  // No-beat timeout → freeze BPM display
  if (signalActive && (now - lastBeatTime > NO_BEAT_TIMEOUT)) {
    signalActive = false;
  }

  // Recalculate BPM over 5-second window
  if (signalActive) {
    int beatsInWindow = 0;
    for (int i = 0; i < min(beatCount, 20); i++) {
      if (now - beatTimes[i] <= WINDOW_MS) beatsInWindow++;
    }
    bpm = beatsInWindow * 12;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// ESP-NOW PROTOCOL
// ═══════════════════════════════════════════════════════════════════════════════
uint8_t peerAddress[] = {0x1C, 0xDB, 0xD4, 0xF0, 0x49, 0xE8};

#define SOURCE_ID       0x01
#define DEST_ID         0x02
#define FC_RETRANSMIT   0x01
#define FC_DATA         0x02
#define FC_RESET        0x03
#define FC_STATUS       0x04
#define FC_ACK          0x05
#define SOC_BYTE        0x01
#define EOT_BYTE        0x02
#define MAX_RETRIES     3
#define ACK_TIMEOUT_MS  500

typedef struct CommunicationMessage {
  uint8_t SOC;
  uint8_t PL;
  uint8_t sourceID;
  uint8_t destID;
  uint8_t PC;
  uint8_t FC;
  uint8_t data;      // temperature byte
  uint8_t data2;     // BPM byte  ← NEW
  uint8_t EOT;
  uint8_t LRC;
} CommunicationMessage;

CommunicationMessage txMsg;
CommunicationMessage rxMsg;

uint8_t packetCounter    = 0;
bool    ackReceived       = false;
bool    retransmitRequested = false;

esp_now_peer_info_t peerInfo;

uint8_t calculateLRC(CommunicationMessage *msg) {
  return msg->PL + msg->sourceID + msg->destID + msg->PC +
         msg->FC + msg->data + msg->data2 + msg->EOT;
}

void buildPacket(CommunicationMessage *msg, uint8_t fc,
                 uint8_t data, uint8_t data2 = 0x00) {
  msg->SOC      = SOC_BYTE;
  msg->PL       = 7;           // one byte longer now
  msg->sourceID = SOURCE_ID;
  msg->destID   = DEST_ID;
  msg->PC       = packetCounter;
  msg->FC       = fc;
  msg->data     = data;
  msg->data2    = data2;
  msg->EOT      = EOT_BYTE;
  msg->LRC      = calculateLRC(msg);
}

bool verifyPacket(CommunicationMessage *msg) {
  if (msg->SOC != SOC_BYTE) return false;
  if (msg->EOT != EOT_BYTE) return false;
  return (msg->LRC == calculateLRC(msg));
}

void sendAck(uint8_t toPC) {
  CommunicationMessage ack;
  ack.SOC      = SOC_BYTE;
  ack.PL       = 7;
  ack.sourceID = SOURCE_ID;
  ack.destID   = DEST_ID;
  ack.PC       = toPC;
  ack.FC       = FC_ACK;
  ack.data     = 0x00;
  ack.data2    = 0x00;
  ack.EOT      = EOT_BYTE;
  ack.LRC      = calculateLRC(&ack);
  esp_now_send(peerAddress, (uint8_t *)&ack, sizeof(ack));
  Serial.println("[PROTOCOL] ACK verzonden.");
}

void sendRetransmit() {
  CommunicationMessage retransmit;
  retransmit.SOC      = SOC_BYTE;
  retransmit.PL       = 7;
  retransmit.sourceID = SOURCE_ID;
  retransmit.destID   = DEST_ID;
  retransmit.PC       = packetCounter;
  retransmit.FC       = FC_RETRANSMIT;
  retransmit.data     = 0x00;
  retransmit.data2    = 0x00;
  retransmit.EOT      = EOT_BYTE;
  retransmit.LRC      = calculateLRC(&retransmit);
  esp_now_send(peerAddress, (uint8_t *)&retransmit, sizeof(retransmit));
  Serial.println("[PROTOCOL] Retransmit verzonden.");
}

void handleReceivedPacket(CommunicationMessage *msg) {
  if (!verifyPacket(msg)) {
    Serial.println("[PROTOCOL] LRC fout! Retransmit aangevraagd.");
    sendRetransmit();
    return;
  }
  if (msg->destID != SOURCE_ID) {
    Serial.println("[PROTOCOL] Bericht niet voor dit apparaat, genegeerd.");
    return;
  }

  Serial.printf("[PROTOCOL] Van 0x%02X | PC: %d | FC: 0x%02X\n",
                msg->sourceID, msg->PC, msg->FC);

  switch (msg->FC) {
    case FC_ACK:
      Serial.println("[PROTOCOL] ACK ontvangen → pakket bevestigd.");
      ackReceived = true;
      break;
    case FC_RETRANSMIT:
      Serial.println("[PROTOCOL] Retransmit gevraagd.");
      retransmitRequested = true;
      break;
    case FC_DATA:
      Serial.printf("[DATA] Temp byte: %d | BPM byte: %d\n",
                    msg->data, msg->data2);
      sendAck(msg->PC);
      break;
    case FC_STATUS:
      Serial.println("[PROTOCOL] Statusaanvraag ontvangen.");
      break;
    case FC_RESET:
      Serial.println("[PROTOCOL] RESET → herstarten...");
      ESP.restart();
      break;
    default:
      Serial.printf("[PROTOCOL] Onbekende FC: 0x%02X\n", msg->FC);
      break;
  }
}

void sendSensorData(uint8_t tempByte, uint8_t bpmByte) {
  packetCounter++;
  buildPacket(&txMsg, FC_DATA, tempByte, bpmByte);

  for (uint8_t attempt = 1; attempt <= MAX_RETRIES; attempt++) {
    ackReceived         = false;
    retransmitRequested = false;

    Serial.printf("\n[SEND] Poging %d | PC: %d | Temp: %d | BPM: %d\n",
                  attempt, packetCounter, tempByte, bpmByte);

    esp_now_send(peerAddress, (uint8_t *)&txMsg, sizeof(txMsg));

    unsigned long t = millis();
    while (millis() - t < ACK_TIMEOUT_MS) {
      if (ackReceived)         { Serial.println("[SEND] ACK → succes!"); return; }
      if (retransmitRequested) { Serial.println("[SEND] Retransmit..."); break; }
      delay(10);
    }
    if (!ackReceived) Serial.println("[SEND] Timeout → volgende poging...");
  }
  Serial.println("[SEND] Max retries bereikt. Pakket verloren.");
}

// ── ESP-NOW callbacks ─────────────────────────────────────────────────────────
void onDataSent(const wifi_tx_info_t *txInfo, esp_now_send_status_t status) {
  Serial.print("[ESP-NOW] Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "MISLUKT");
}

void onDataRecv(const esp_now_recv_info *recvInfo,
                const uint8_t *incomingData, int len) {
  if (len == sizeof(CommunicationMessage)) {
    memcpy(&rxMsg, incomingData, sizeof(CommunicationMessage));
    handleReceivedPacket(&rxMsg);
  } else {
    Serial.println("[ESP-NOW] Onverwachte pakketgrootte.");
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
// SETUP & LOOP
// ═══════════════════════════════════════════════════════════════════════════════

// Timing
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 3000;   // send every 3 s

void setup() {
  Serial.begin(115200);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.print("MAC-adres (A): ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init mislukt!");
    return;
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Peer toevoegen mislukt!");
    return;
  }

  Serial.println("NTC + Pulse + ESP-NOW klaar!\n");
}

void loop() {
  // ── Run heartbeat every 5 ms (same rate as original) ──────────────────────
  updateHeartbeat();

  // ── Every 3 seconds: read temperature and transmit both values ─────────────
  unsigned long now = millis();
  if (now - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = now;

    // Temperature
    float rawADC = 0;
    float tempC  = readTemperature(&rawADC);

    Serial.printf("RAW: %4.0f  |  ", rawADC);
    if (tempC < 0.0f) {
      Serial.println("Temp: fout (buiten bereik)");
      return;
    }
    Serial.printf("Temp: %.1f °C  |  BPM: %d  |  Active: %d\n",
                  tempC, bpm, signalActive);

    // Encode temperature: 25–43 °C → 0–255
    // Decoder: temp = (data / 255.0f) * 18.0f + 25.0f
    uint8_t tempByte = (uint8_t)((tempC - 25.0f) / 18.0f * 255.0f);

    // Encode BPM: 0–255 BPM fits directly in a byte
    uint8_t bpmByte = (uint8_t)constrain(bpm, 0, 255);

    sendSensorData(tempByte, bpmByte);
  }

  delay(5);   // keep heartbeat sampling rate ~200 Hz
}