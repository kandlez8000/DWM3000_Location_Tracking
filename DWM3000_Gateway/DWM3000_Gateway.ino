#include <esp_now.h>
#include <WiFi.h>

#define NUM_ANCHORS 4

typedef struct struct_message {
  int tag_id;
  float distances[NUM_ANCHORS];
} struct_message;

volatile bool hasPacket = false;
struct_message latest;

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len < (int)sizeof(struct_message)) return;  // tolerate future extensions
  memcpy((void*)&latest, incomingData, sizeof(struct_message));
  hasPacket = true;
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("Gateway Ready. Listening for Tag data...");
}

void loop() {
  if (hasPacket) {
    hasPacket = false;

    Serial.print("Tag ID ");
    Serial.print(latest.tag_id);
    Serial.print(" | ");

    for (int i = 0; i < NUM_ANCHORS; i++) {
      Serial.print("A");
      Serial.print(i + 1);
      Serial.print(": ");

      if (latest.distances[i] > 0) {
        Serial.print(latest.distances[i], 2);
        Serial.print(" cm");
      } else {
        Serial.print("INVALID");
      }

      if (i < NUM_ANCHORS - 1) Serial.print(" | ");
    }
    Serial.println();
  }

  delay(10);
}