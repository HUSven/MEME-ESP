/*
 * ESP-NOW Transceiver - Apparaat B (Ontvanger/Display)
 * Communicatieprotocol: Mensen Meten v1.0
 * OLED display toont ontvangen patiëntdata (temp + BPM)
 */

#include <WiFi.h>
#include <esp_now.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ─────────────────────────────────────────
// OLED CONFIG
// ─────────────────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT  64
#define OLED_CS   21
#define OLED_DC    4
#define OLED_RST   5
#define OLED_SCK   2
#define OLED_MOSI  3

Adafruit_SSD1306 display(
  SCREEN_WIDTH, SCREEN_HEIGHT,
  &SPI, OLED_DC, OLED_RST, OLED_CS
);

// ─────────────────────────────────────────
// ESP-NOW PROTOCOL
// ─────────────────────────────────────────
uint8_t peerAddress[] = {0x1C, 0xDB, 0xD4, 0xF0, 0x3E, 0x3C};

#define SOURCE_ID       0x01
#define DEST_ID         0x02

#define FC_RETRANSMIT   0x01
#define FC_DATA         0x02
#define FC_RESET        0x03
#define FC_STATUS       0x04
#define FC_ACK          0x05

#define SOC_BYTE        0x01
#define EOT_BYTE        0x02
#define MAX_RETRIES        3
#define ACK_TIMEOUT_MS   500

typedef struct CommunicationMessage {
  uint8_t SOC;
  uint8_t PL;
  uint8_t sourceID;
  uint8_t destID;
  uint8_t PC;
  uint8_t FC;
  uint8_t data;
  uint8_t data2;
  uint8_t EOT;
  uint8_t LRC;
} CommunicationMessage;

CommunicationMessage txMsg;
CommunicationMessage rxMsg;

uint8_t packetCounter = 0;
bool ackReceived        = false;
bool retransmitRequested = false;

esp_now_peer_info_t peerInfo;

// ─────────────────────────────────────────
// DISPLAY STATE
// ─────────────────────────────────────────
struct PatientData {
  float  temp       = 0.0f;
  uint8_t bpm       = 0;
  bool   hasData    = false;
  uint32_t lastSeen = 0;    // millis() timestamp
} patient;

// ─────────────────────────────────────────
// PROTOCOL HELPERS
// ─────────────────────────────────────────
uint8_t tempDecoder(uint8_t raw) {
  return (raw / 255.0f) * 18.0f + 25.0f;
}

uint8_t calculateLRC(CommunicationMessage *msg) {
  return msg->PL + msg->sourceID + msg->destID +
         msg->PC + msg->FC + msg->data + msg->data2 + msg->EOT;
}

void buildPacket(CommunicationMessage *msg, uint8_t fc, uint8_t data) {
  msg->SOC      = SOC_BYTE;
  msg->PL       = 7;
  msg->sourceID = SOURCE_ID;
  msg->destID   = DEST_ID;
  msg->PC       = packetCounter;
  msg->FC       = fc;
  msg->data     = data;
  msg->data2    = 0x00;
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

// ─────────────────────────────────────────
// PACKET HANDLER
// ─────────────────────────────────────────
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

  Serial.print("[PROTOCOL] Ontvangen van 0x");
  Serial.print(msg->sourceID, HEX);
  Serial.print(" | PC: "); Serial.print(msg->PC);
  Serial.print(" | FC: 0x"); Serial.println(msg->FC, HEX);

  switch (msg->FC) {
    case FC_ACK:
      Serial.println("[PROTOCOL] ACK ontvangen → pakket bevestigd.");
      ackReceived = true;
      break;

    case FC_RETRANSMIT:
      Serial.println("[PROTOCOL] Retransmit gevraagd door ontvanger.");
      retransmitRequested = true;
      break;

    case FC_DATA:
      patient.temp    = tempDecoder(msg->data);
      patient.bpm     = msg->data2;
      patient.hasData = true;
      patient.lastSeen = millis();
      Serial.print("[DATA] Temp: "); Serial.println(patient.temp);
      Serial.print("[DATA] BPM: ");  Serial.println(patient.bpm);
      sendAck(msg->PC);
      break;

    case FC_STATUS:
      Serial.println("[PROTOCOL] Statusaanvraag ontvangen.");
      break;

    case FC_RESET:
      Serial.println("[PROTOCOL] RESET ontvangen → herstarten...");
      ESP.restart();
      break;

    default:
      Serial.print("[PROTOCOL] Onbekende FC: 0x");
      Serial.println(msg->FC, HEX);
      break;
  }
}

// ─────────────────────────────────────────
// SEND WITH RETRY
// ─────────────────────────────────────────
void sendData(uint8_t value) {
  packetCounter++;
  buildPacket(&txMsg, FC_DATA, value);

  for (uint8_t attempt = 1; attempt <= MAX_RETRIES; attempt++) {
    ackReceived        = false;
    retransmitRequested = false;

    Serial.print("\n[SEND] Poging "); Serial.print(attempt);
    Serial.print(" | PC: "); Serial.print(packetCounter);

    esp_now_send(peerAddress, (uint8_t *)&txMsg, sizeof(txMsg));

    unsigned long t = millis();
    while (millis() - t < ACK_TIMEOUT_MS) {
      if (ackReceived)         { Serial.println("[SEND] ACK → succes!"); return; }
      if (retransmitRequested) { Serial.println("[SEND] Retransmit → opnieuw..."); break; }
      delay(10);
    }

    if (!ackReceived) Serial.println("[SEND] Timeout → volgende poging...");
  }
  Serial.println("[SEND] Max retries bereikt. Pakket verloren.");
}

// ─────────────────────────────────────────
// ESP-NOW CALLBACKS
// ─────────────────────────────────────────
void onDataSent(const wifi_tx_info_t *txInfo, esp_now_send_status_t status) {
  Serial.print("[ESP-NOW] Verzend status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "MISLUKT");
}

void onDataRecv(const esp_now_recv_info *recvInfo, const uint8_t *incomingData, int len) {
  if (len == sizeof(CommunicationMessage)) {
    memcpy(&rxMsg, incomingData, sizeof(CommunicationMessage));
    handleReceivedPacket(&rxMsg);
  } else {
    Serial.println("[ESP-NOW] Onverwachte pakketgrootte ontvangen.");
  }
}

// ─────────────────────────────────────────
// DISPLAY RENDER
// ─────────────────────────────────────────
void renderDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  if (!patient.hasData) {
    // ── Waiting screen ──
    display.setCursor(20, 20);
    display.println("Wachten op data...");
    display.setCursor(28, 36);
    display.println("Geen verbinding");
  } else {
    uint32_t age = (millis() - patient.lastSeen) / 1000;  // seconds
    bool stale   = age > 10;

    // ── Header ──
    display.setCursor(0, 0);
    display.println(stale ? "Pati:nt [OFFLINE]" : "Pati:nt [LIVE]");
    display.drawFastHLine(0, 10, SCREEN_WIDTH, SSD1306_WHITE);

    // ── Temperature ──
    display.setTextSize(1);
    display.setCursor(0, 14);
    display.print("Temp:");
    display.setTextSize(2);
    display.setCursor(0, 24);
    display.print(patient.temp, 1);
    display.print(" C");

    // ── BPM ──
    display.setTextSize(1);
    display.setCursor(72, 14);
    display.print("BPM:");
    display.setTextSize(2);
    display.setCursor(72, 24);
    display.print(patient.bpm);

    // ── Divider ──
    display.drawFastHLine(0, 46, SCREEN_WIDTH, SSD1306_WHITE);

    // ── Last seen ──
    display.setTextSize(1);
    display.setCursor(0, 52);
    display.print("Laatste update: ");
    display.print(age);
    display.print("s");
  }

  display.display();
}

// ─────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // OLED init
  SPI.begin(OLED_SCK, -1, OLED_MOSI, OLED_CS);
  if (!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println("OLED init mislukt!");
    while (true);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 24);
  display.println("ESP-NOW opstart...");
  display.display();

  // ESP-NOW init
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.print("MAC-adres (B): ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init mislukt!");
    display.clearDisplay();
    display.setCursor(10, 24);
    display.println("ESP-NOW FOUT!");
    display.display();
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

  Serial.println("Klaar!\n");
}

// ─────────────────────────────────────────
// LOOP
// ─────────────────────────────────────────
void loop() {
  renderDisplay();
  delay(250);
}