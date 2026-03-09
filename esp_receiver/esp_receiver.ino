#include <esp_now.h>
#include <WiFi.h>

typedef struct CommunicationMessage {
  uint8_t SOC,
  uint8_t PL,
  uint8_t 
} CommunicationMessage;

struct_message myData;

// Replace with the OTHER board's MAC
uint8_t peerMAC[] = {0x1C,0xDB,0xD4,0xF0,0x3E,0x3C}; // if uploading to Board A
// For Board B, peerMAC = MAC of Board A

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  struct_message receivedData;
  memcpy(&receivedData, incomingData, sizeof(receivedData));

  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
           recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);

  Serial.print("Received from ");
  Serial.print(macStr);
  Serial.print(": ");
  Serial.println(receivedData.number);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  // Register receive callback
  esp_now_register_recv_cb(OnDataRecv);

  // Add peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
  }
}

void loop() {
  // myData.number = random(0, 100);

  // esp_err_t result = esp_now_send(peerMAC, (uint8_t *)&myData, sizeof(myData));
  // if(result == ESP_OK) {
  //   Serial.print("Sent: ");
  //   Serial.println(myData.number);
  // } else {
  //   Serial.println("Send Error");
  // }

  // delay(2000);
}