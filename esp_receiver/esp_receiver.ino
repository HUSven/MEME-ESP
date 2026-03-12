/*
 * ESP-NOW Transceiver - Apparaat A (Zender/Patiënt)
 * Communicatieprotocol: Mensen Meten v1.0
*/

#include <WiFi.h>
#include <esp_now.h>

// MAC address van receiver
uint8_t peerAddress[] = { 0x1C, 0xDB, 0xD4, 0xF0, 0x3E, 0x3C };

// Zender / afzender
#define SOURCE_ID 0x01
#define DEST_ID 0x02

// Functiecodes
#define FC_RETRANSMIT 0x01
#define FC_DATA 0x02
#define FC_RESET 0x03
#define FC_STATUS 0x04
#define FC_ACK 0x05

// Protocol constanten
#define SOC_BYTE 0x01
#define EOT_BYTE 0x02
#define MAX_RETRIES 3
#define ACK_TIMEOUT_MS 500

// Structure van bericht
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
bool ackReceived = false;
bool retransmitRequested = false;

esp_now_peer_info_t peerInfo;

float tempDecoder(uint8_t data) {
  return (data / 255.0f) * 18.0f + 25.0f;
}

// Berekend de LRC
uint8_t calculateLRC(CommunicationMessage *msg) {
  return msg->PL + msg->sourceID + msg->destID + msg->PC + msg->FC + msg->data + msg->data2 + msg->EOT;
}

// Bericht valideren
bool verifyPacket(CommunicationMessage *msg) {
  if (msg->SOC != SOC_BYTE) return false;
  if (msg->EOT != EOT_BYTE) return false;
  return (msg->LRC == calculateLRC(msg));
}

// Ack sturen
void sendAck(uint8_t toPC) {
  CommunicationMessage ack;
  ack.SOC = SOC_BYTE;
  ack.sourceID = SOURCE_ID;
  ack.destID = DEST_ID;
  ack.PC = toPC;
  ack.FC = FC_ACK;
  ack.data = 0x00;
  ack.data2 = 0x00;
  ack.PL = sizeof(ack.data + ack.data2);
  ack.EOT = EOT_BYTE;
  ack.LRC = calculateLRC(&ack);

  esp_now_send(peerAddress, (uint8_t *)&ack, sizeof(ack));
  Serial.println("[PROTOCOL] ACK verzonden.");
}

// Stuurt de retransmit
void sendRetransmit() {
  CommunicationMessage retransmit;
  retransmit.SOC = SOC_BYTE;
  retransmit.sourceID = SOURCE_ID;
  retransmit.destID = DEST_ID;
  retransmit.PC = packetCounter;
  retransmit.FC = FC_RETRANSMIT;
  retransmit.data = 0x00;
  retransmit.data2 = 0x00;
  retransmit.PL = sizeof(retransmit.data + retransmit.data2);
  retransmit.EOT = EOT_BYTE;
  retransmit.LRC = calculateLRC(&retransmit);

  esp_now_send(peerAddress, (uint8_t *)&retransmit, sizeof(retransmit));
  Serial.println("[PROTOCOL] Retransmit verzonden.");
}

// behandelen van de ontvangde bericht
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
  Serial.print(" | PC: ");
  Serial.print(msg->PC);
  Serial.print(" | FC: 0x");
  Serial.println(msg->FC, HEX);

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
      Serial.printf("[DATA] Temp: ");
      Serial.println(tempDecoder(msg->data), 1);
      Serial.print("[DATA] BPM: ");
      Serial.println(msg->data2);
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

// Callbacks voor ESP-NOW
void onDataSent(const wifi_tx_info_t *txInfo, esp_now_send_status_t status) {
  Serial.print("[ESP-NOW] Verzend status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK\n" : "MISLUKT\n");
}

void onDataRecv(const esp_now_recv_info *recvInfo, const uint8_t *incomingData, int len) {
  if (len == sizeof(CommunicationMessage)) {
    memcpy(&rxMsg, incomingData, sizeof(CommunicationMessage));
    handleReceivedPacket(&rxMsg);
  } else {
    Serial.println("[ESP-NOW] Onverwachte pakketgrootte ontvangen.");
  }
}

void setup() {
  Serial.begin(115200);

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

  memcpy(peerInfo.peer_addr, peerAddress, 7);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Peer toevoegen mislukt!");
    return;
  }

  Serial.println("ESP-NOW + Protocol klaar!\n");
}

void loop() {
}
