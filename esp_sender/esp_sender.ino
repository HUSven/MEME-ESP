#include <esp_now.h>
#include <WiFi.h>

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

CommunicationMessage message;

int packetsCounter = 0;

uint8_t peerMAC[] = {0x1C,0xDB,0xD4,0xF0,0x49,0xE8};

void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
  }
}

void loop()
{
  esp_err_t result = esp_now_send(peerMAC, (uint8_t *)&message, sizeof(message));
  if(result == ESP_OK) {
    Serial.print("Sent: ");
    Serial.println(message.data);
  } else {
    Serial.println("Send Error");
  }

  delay(2000);
}

CommunicationMessage prepareMessage() 
{
  message.SOC = 0x01;
  message.PL = sizeof(CommunicationMessage);
  message.sourceID = 0x13;
  message.destID = 0x26;
  message.PC = packetsCounter;
  message.FC = 0x02;
  message.data = 0x10;
  message.EOT = 0x02;
  
}