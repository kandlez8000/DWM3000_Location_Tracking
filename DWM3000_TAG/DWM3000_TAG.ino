#include <SPI.h>

#include <esp_now.h>
#include <WiFi.h>
#include "DWM3000_Driver.h"
#include "DWM3000_registers.h"

// Scalable Anchor Configuration
#define NUM_ANCHORS 4 // 
#define TAG_ID 10
#define FIRST_ANCHOR_ID 1 // Starting ID for anchors (1, 2, 3, ...)

// Ranging Configuration
#define FILTER_SIZE 50 // For median filter
                       // Can make this value lower for higher
#define MIN_DISTANCE 0
#define MAX_DISTANCE 10000.0

//  --- ESP-NOW Setup ---
//  --- ESP-NOW Setup ---
//  --- ESP-NOW Setup ---

// found with Find_MAC_Address.ino
uint8_t gatewayAddress[] = {0x08, 0xB6, 0x1F, 0x76, 0x92, 0xF4}; 

// The struct that will be fired through the air
typedef struct struct_message {
  int tag_id;
  float distances[NUM_ANCHORS];
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;
//  --- ESP-NOW Setup ---
//  --- ESP-NOW Setup ---
//  --- ESP-NOW Setup ---

// Initial Radio Configuration
uint8_t config[7] = {
   CHANNEL_5,         // Channel
   PREAMBLE_128,      // Preamble Length
   9,                 // Preamble Code (Same for RX and TX!)
   PAC8,              // PAC
   DATARATE_6_8MB,    // Datarate
   PHR_MODE_STANDARD, // PHR Mode
   PHR_RATE_850KB     // PHR Rate
};

// Global variables
static int rx_status;
static int tx_status;
static int current_anchor_index = 0; // Index into anchors array
static unsigned long timeout_timer = 0;
static int curr_stage = 0;

// Anchor data structure
struct AnchorData
{
   int anchor_id; // Anchor ID

   // Timing measurements
   int t_roundA = 0;
   int t_replyA = 0;
   long long rx = 0;
   long long tx = 0;
   int clock_offset = 0;

   // Distance measurements
   float distance = 0;
   float distance_history[FILTER_SIZE] = {0};
   int history_index = 0;
   float filtered_distance = 0;

   // Signal quality metrics
   float signal_strength = 0;    // RSSI in dBm
   float fp_signal_strength = 0; // First Path RSSI in dBm
};

// Dynamic array of anchor data
AnchorData anchors[NUM_ANCHORS];

// Helper functions for anchor management
void initializeAnchors()
{
   for (int i = 0; i < NUM_ANCHORS; i++)
   {
       anchors[i].anchor_id = FIRST_ANCHOR_ID + i;
       // Initialize all other fields to zero (default constructor handles this)
   }
}

AnchorData *getCurrentAnchor()
{
   return &anchors[current_anchor_index];
}

int getCurrentAnchorId()
{
   return anchors[current_anchor_index].anchor_id;
}

void switchToNextAnchor()
{
   current_anchor_index = (current_anchor_index + 1) % NUM_ANCHORS;
}

bool allAnchorsHaveValidData()
{
   for (int i = 0; i < NUM_ANCHORS; i++)
   {
       if (anchors[i].filtered_distance <= 0)
       {
           return false;
       }
   }
   return true;
}

// Helper function to validate distance
bool isValidDistance(float distance)
{
   return (distance >= MIN_DISTANCE && distance <= MAX_DISTANCE);
}

float calculateMedian(float arr[], int size)
{
   float temp[size];
   for (int i = 0; i < size; i++)
   {
       temp[i] = arr[i];
   }

   for (int i = 0; i < size - 1; i++)
   {
       for (int j = i + 1; j < size; j++)
       {
           if (temp[j] < temp[i])
           {
               float t = temp[i];
               temp[i] = temp[j];
               temp[j] = t;
           }
       }
   }

   if (size % 2 == 0)
   {
       return (temp[size / 2 - 1] + temp[size / 2]) / 2.0;
   }
   else
   {
       return temp[size / 2];
   }
}

// updateFilteredDistance function
void updateFilteredDistance(AnchorData &data)
{
   data.distance_history[data.history_index] = data.distance;
   data.history_index = (data.history_index + 1) % FILTER_SIZE;

   float valid_distances[FILTER_SIZE];
   int valid_count = 0;

   for (int i = 0; i < FILTER_SIZE; i++)
   {
       if (isValidDistance(data.distance_history[i]))
       {
           valid_distances[valid_count++] = data.distance_history[i];
       }
   }

   if (valid_count > 0)
   {
       data.filtered_distance = calculateMedian(valid_distances, valid_count);
   }
   else
   {
       data.filtered_distance = 0;
   }
}

// Debug print function
void printDebugInfo(int anchor, long long rx, long long tx, int t_round, int t_reply, int clock_offset)
{
   Serial.print("Anchor ");
   Serial.print(anchor);
   Serial.println(" Debug Info:");
   Serial.print("RX timestamp: ");
   Serial.println(rx);
   Serial.print("TX timestamp: ");
   Serial.println(tx);
   Serial.print("t_round: ");
   Serial.println(t_round);
   Serial.print("t_reply: ");
   Serial.println(t_reply);
   Serial.print("Clock offset: ");
   Serial.println(clock_offset);

   int ranging_time = DWM3000.ds_processRTInfo(t_round, t_reply,
                                               DWM3000.read(0x12, 0x04), DWM3000.read(0x12, 0x08), clock_offset);
   Serial.print("Calculated distance: ");
   Serial.println(DWM3000.convertToCM(ranging_time));
}

void printAllDistances()
{
   Serial.print("Distances - ");
   for (int i = 0; i < NUM_ANCHORS; i++)
   {
       Serial.print("A");
       Serial.print(anchors[i].anchor_id);
       Serial.print(": ");
       if (anchors[i].filtered_distance > 0)
       {
           DWM3000.printDouble(anchors[i].filtered_distance, 100, false);
           Serial.print(" cm");
       }
       else
       {
           Serial.print("INVALID");
       }

       if (i < NUM_ANCHORS - 1)
       {
           Serial.print(" | ");
       }
   }
   Serial.println();
}

void setup()    {
   Serial.begin(115200);

    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) 
    {
       Serial.println("[ERROR] Error initializing ESP-NOW");
       return;
    }

   // Register the Gateway as a peer
   memcpy(peerInfo.peer_addr, gatewayAddress, 6);
   peerInfo.channel = 0;  
   peerInfo.encrypt = false;
   if (esp_now_add_peer(&peerInfo) != ESP_OK)
   {
       Serial.println("[ERROR] Failed to add Gateway peer");
       return;
   }

   // Initialize anchor array
   initializeAnchors();

   Serial.print("Initialized ");
   Serial.print(NUM_ANCHORS);
   Serial.println(" anchors:");
   for (int i = 0; i < NUM_ANCHORS; i++)
   {
       Serial.print("  Anchor ");
       Serial.print(i);
       Serial.print(" - ID: ");
       Serial.println(anchors[i].anchor_id);
   }


   // Initialize UWB
   DWM3000.begin();
   DWM3000.hardReset();
   delay(200);

   if (!DWM3000.checkSPI())
   {
       Serial.println("[ERROR] Could not establish SPI Connection to DWM3000!");
       while (1)
           ;
   }

   while (!DWM3000.checkForIDLE())
   {
       Serial.println("[ERROR] IDLE1 FAILED\r");
       delay(1000);
   }

   DWM3000.softReset();
   delay(200);

   if (!DWM3000.checkForIDLE())
   {
       Serial.println("[ERROR] IDLE2 FAILED\r");
       while (1)
           ;
   }

   DWM3000.init();
   DWM3000.setupGPIO();
   DWM3000.setTXAntennaDelay(16350);
   DWM3000.setSenderID(TAG_ID);

   Serial.println("> TAG - Four Anchor Ranging System <");
   Serial.println("> With WiFi ESP32-NOW Communication <\n");
   Serial.println("[INFO] Setup is finished.");
   Serial.print("Antenna delay set to: ");
   Serial.println(DWM3000.getTXAntennaDelay());

   DWM3000.configureAsTX();
   DWM3000.clearSystemStatus();
}

void finishAnchorCycle() 
{
    // If we just finished the very last anchor in the array (regardless of success/fail)
    if (current_anchor_index == NUM_ANCHORS - 1) 
    {
        // 1. Print the completed array to the Serial Monitor
        printAllDistances(); 
        
        // 2. Blast the array to the Gateway via ESP-NOW
        myData.tag_id = TAG_ID;
        for(int i = 0; i < NUM_ANCHORS; i++) {
            myData.distances[i] = anchors[i].filtered_distance;
        }
        esp_now_send(gatewayAddress, (uint8_t *) &myData, sizeof(myData));
        
        // 3. ALOHA Random backoff before starting a brand new cycle
        delay(random(50, 150)); 
    }
    
    // Pivot to the next anchor and reset the stage
    switchToNextAnchor();
    curr_stage = 0;
}

void loop() {
   AnchorData *currentAnchor = getCurrentAnchor();
   int currentAnchorId = getCurrentAnchorId();

   switch (curr_stage)
   {
   case 0: // Start ranging with current target
       currentAnchor->t_roundA = 0;
       currentAnchor->t_replyA = 0;

       DWM3000.setDestinationID(currentAnchorId);
       DWM3000.ds_sendFrame(1);
       currentAnchor->tx = DWM3000.readTXTimestamp();
       
       curr_stage = 1;
       timeout_timer = millis(); // Start the stopwatch
       break;

   case 1: // Await first response
       if (rx_status = DWM3000.receivedFrameSucc())
       {
           DWM3000.clearSystemStatus();
           if (rx_status == 1)
           {
               if (DWM3000.ds_isErrorFrame())
               {
                   finishAnchorCycle();
               }
               else if (DWM3000.ds_getStage() != 2)
               {
                   DWM3000.ds_sendErrorFrame();
                   finishAnchorCycle();
               }
               else
               {
                   curr_stage = 2;
                   timeout_timer = millis(); // Reset stopwatch
               }
           }
       }
       // The Escape Hatch
       else if (millis() - timeout_timer > 50) 
       {
            DWM3000.writeFastCommand(0x00);
           DWM3000.clearSystemStatus();
           currentAnchor->filtered_distance = 0.0; // clear this line if this isn't the issue
           finishAnchorCycle(); 
       }
       break;

   case 2: // Response received. Send second ranging
       currentAnchor->rx = DWM3000.readRXTimestamp();
       DWM3000.ds_sendFrame(3);

       currentAnchor->t_roundA = currentAnchor->rx - currentAnchor->tx;
       currentAnchor->tx = DWM3000.readTXTimestamp();
       currentAnchor->t_replyA = currentAnchor->tx - currentAnchor->rx;

       curr_stage = 3;
       timeout_timer = millis(); // Reset stopwatch
       break;

   case 3: // Await second response
       if (rx_status = DWM3000.receivedFrameSucc())
       {
           DWM3000.clearSystemStatus();
           if (rx_status == 1 && !DWM3000.ds_isErrorFrame())
           {
                currentAnchor->clock_offset = DWM3000.getRawClockOffset();
                curr_stage = 4;
           }
           else
           {
                finishAnchorCycle();
           }
       }
       // The Escape Hatch
       else if (millis() - timeout_timer > 50) 
       {
            DWM3000.writeFastCommand(0x00);
           DWM3000.clearSystemStatus();
           currentAnchor->filtered_distance = 0.0;
           finishAnchorCycle();
       }
       break;

    case 4: // Response received. Calculating results
    {
        int ranging_time = DWM3000.ds_processRTInfo(
            currentAnchor->t_roundA,
            currentAnchor->t_replyA,
            DWM3000.read(0x12, 0x04),
            DWM3000.read(0x12, 0x08),
            currentAnchor->clock_offset);

        currentAnchor->distance = DWM3000.convertToCM(ranging_time);
        currentAnchor->signal_strength = DWM3000.getSignalStrength();
        currentAnchor->fp_signal_strength = DWM3000.getFirstPathSignalStrength();

        // Update the median filter array
        updateFilteredDistance(*currentAnchor);

        // Let your brilliantly designed helper function handle the printing, 
        // the ESP-NOW transmission, the ALOHA delay, and the target switching!
        finishAnchorCycle();
        
        break;
    } // End of Case 4

    default:
        curr_stage = 0;
        break;
    }
}

