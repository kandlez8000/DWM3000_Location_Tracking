#include <esp_now.h>
#include <WiFi.h>

// 1. This MUST match the Tag exactly!
#define NUM_ANCHORS 4 

// 2. The exact same data structure the Tag is transmitting
typedef struct struct_message {
  int tag_id;
  float distances[NUM_ANCHORS]; // The dynamic array
} struct_message;

struct_message myData;

// Updated callback function for ESP32 Core v3.x
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  // Check if the incoming packet matches our struct size to prevent crashes
  if (len != sizeof(myData)) {
    Serial.println("[ERROR] Received packet size does not match struct!");
    return;
  }

  // Paste the bytes from the air into our struct
  memcpy(&myData, incomingData, sizeof(myData));
  
  // Format the output
  Serial.print("Tag ID ");
  Serial.print(myData.tag_id);
  Serial.print(" | ");
  
  // 3. Dynamically loop through however many anchors you have defined
  for (int i = 0; i < NUM_ANCHORS; i++) {
    Serial.print("A");
    Serial.print(i + 1);
    Serial.print(": ");
    
    // Check if the Tag sent a valid distance
    if (myData.distances[i] > 0) {
      Serial.print(myData.distances[i]);
      Serial.print(" cm");
    } else {
      Serial.print("INVALID");
    }

    // Add a divider between anchors
    if (i < NUM_ANCHORS - 1) {
      Serial.print(" | ");
    }
  }
  Serial.println(); // New line for the next incoming packet
}

void setup() {
  Serial.begin(115200);
  
  // Set ESP32 as a Wi-Fi Station (required for ESP-NOW)
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register the listening function
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("Gateway Ready. Listening for Tag data...");
}

void loop() {
  // The ESP32 handles ESP-NOW reception completely in the background on Core 0.
  // The main loop stays empty and fast!
  delay(1000); 
}