/*
 * ESP-NOW Transceiver - Apparaat A (Zender/Patiënt)
 * Communicatieprotocol: Mensen Meten v1.0
*/

#include <WiFi.h>
#include <esp_now.h>

// MAC address van receiver
uint8_t peerAddress[] = {0x1C,0xDB,0xD4,0xF0,0x49,0xE8};

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
  uint8_t EOT;
  uint8_t LRC;
} CommunicationMessage;

CommunicationMessage txMsg;
CommunicationMessage rxMsg;

uint8_t packetCounter = 0;
bool ackReceived = false;
bool retransmitRequested = false;

esp_now_peer_info_t peerInfo;

// Berekend de LRC
uint8_t calculateLRC(CommunicationMessage *msg) {
  return msg->PL + msg->sourceID + msg->destID + msg->PC + msg->FC + msg->data + msg->EOT;
}

// Opbouwen van bericht
void buildPacket(CommunicationMessage *msg, uint8_t fc, uint8_t data) {
  msg->SOC = SOC_BYTE;
  msg->PL = 6;
  msg->sourceID = SOURCE_ID;
  msg->destID = DEST_ID;
  msg->PC = packetCounter;
  msg->FC = fc;
  msg->data = data;
  msg->EOT = EOT_BYTE;
  msg->LRC = calculateLRC(msg);
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
  ack.PL = 6;
  ack.sourceID = SOURCE_ID;
  ack.destID = DEST_ID;
  ack.PC = toPC;
  ack.FC = FC_ACK;
  ack.data = 0x00;
  ack.EOT = EOT_BYTE;
  ack.LRC = calculateLRC(&ack);

  esp_now_send(peerAddress, (uint8_t *)&ack, sizeof(ack));
  Serial.println("[PROTOCOL] ACK verzonden.");
}

// Stuurt de retransmit
void sendRetransmit() {
  CommunicationMessage retransmit;
  retransmit.SOC = SOC_BYTE;
  retransmit.PL = 6;
  retransmit.sourceID = SOURCE_ID;
  retransmit.destID = DEST_ID;
  retransmit.PC = packetCounter;
  retransmit.FC = FC_RETRANSMIT;
  retransmit.data = 0x00;
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
      Serial.print("[DATA] Ontvangen waarde: 0x");
      Serial.println(msg->data, HEX);
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
      Serial.print("[PROTOCOL] Onbekende FC: 0x"); Serial.println(msg->FC, HEX);
      break;
  }
}

// Verstuurt het de data
void sendData(uint8_t value) {
  packetCounter++;
  buildPacket(&txMsg, FC_DATA, value);

  for (uint8_t attempt = 1; attempt <= MAX_RETRIES; attempt++) {
    ackReceived = false;
    retransmitRequested = false;

    Serial.print("\n[SEND] Poging ");
    Serial.print(attempt);
    Serial.print(" | PC: ");
    Serial.print(packetCounter);
    Serial.print(" | Data: 0x");
    Serial.println(value, HEX);

    esp_now_send(peerAddress, (uint8_t *)&txMsg, sizeof(txMsg));

    // Wacht op ACK of retransmit verzoek
    unsigned long t = millis();
    while (millis() - t < ACK_TIMEOUT_MS) {
      if (ackReceived) { 
        Serial.println("[SEND] ACK ontvangen → succes!"); 
        return; 
      }
      if (retransmitRequested) {
        Serial.println("[SEND] Retransmit → opnieuw...");
        break;
      }
      delay(10);
    }

    if (!ackReceived) {
      Serial.println("[SEND] Timeout → volgende poging...");
    }
  }

  Serial.println("[SEND] Max retries bereikt. Pakket verloren.");
}

// Callbacks voor ESP-NOW
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

  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Peer toevoegen mislukt!");
    return;
  }

  Serial.println("ESP-NOW + Protocol klaar!\n");
}

void loop() {
  uint8_t data = random(0, 100);
  Serial.print("Random Number: ");
  Serial.println(data);
  sendData(data);

  delay(3000);
}
