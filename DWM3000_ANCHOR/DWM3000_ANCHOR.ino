#include <Arduino.h>
#include <SPI.h>
#include "DWM3000_Driver.h"
#include "DWM3000_registers.h"

// Set to 1 for Anchor 1, 2 for Anchor 2
#define ANCHOR_ID 2
#define RESPONSE_TIMEOUT_MS 60 // Maximum time to wait for a response
//#define MAX_RETRIES 3 // commenting out for now, could be causing an issue with desync not being able to recover

unsigned long last_ranging_time = 0;
//int retry_count = 0; //commenting out for now, it could be causing issues with desync not being able to recover
static int rx_status;
static int curr_stage = 0;
static int sessionTagId = -1;
static uint32_t t_roundB = 0;
static uint32_t t_replyB = 0;
static uint64_t rx = 0;
static uint64_t tx = 0;


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


void setup()
{
 Serial.begin(115200);
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

 // Set antenna delay - calibrate this for your hardware!
 DWM3000.setTXAntennaDelay(16350);

 // Set anchor ID
 DWM3000.setSenderID(ANCHOR_ID);

 Serial.print("> ANCHOR ");
 Serial.print(ANCHOR_ID);
 Serial.println(" - Ready for ranging <");
 Serial.print("Antenna delay set to: ");
 Serial.println(DWM3000.getTXAntennaDelay());
 Serial.println("[INFO] Setup finished.");

 DWM3000.configureAsTX();
 DWM3000.clearSystemStatus();
 DWM3000.standardRX();
}

void loop()
{
  

  switch (curr_stage)
  {
  case 0: // Await ranging (expect stage 1 addressed to this anchor)
{
  rx_status = DWM3000.receivedFrameSucc();

  if (rx_status != 0)
  {
    int stage = DWM3000.ds_getStage();
    int src   = DWM3000.getSenderID();
    int dst   = DWM3000.getDestinationID();
    bool err  = DWM3000.ds_isErrorFrame();

    if (rx_status == 2)
    {
      DWM3000.writeFastCommand(0x00);
      DWM3000.clearSystemStatus();
      DWM3000.standardRX();
      break;
    }

    if (rx_status == 1 && !err && stage == 1 && dst == ANCHOR_ID)
    {
      sessionTagId = src;
      DWM3000.setDestinationID(sessionTagId);

      // Reset per-session timing only when a new session starts
      t_roundB = 0;
      t_replyB = 0;

      curr_stage = 1;
      last_ranging_time = millis();
    }
    else if (rx_status == 1 && err && stage == 7 && dst == ANCHOR_ID)
    {
      curr_stage = 0;
      DWM3000.standardRX();
    }
    else
    {
      DWM3000.standardRX();
    }
  }
  else if (millis() - last_ranging_time > 1000)
  {
    DWM3000.writeFastCommand(0x00);
    DWM3000.clearSystemStatus();
    DWM3000.standardRX();
    last_ranging_time = millis();
  }
}
break;

  case 1: // Ranging received. Sending response
    DWM3000.ds_sendFrame(2);
    DWM3000.standardRX();

    rx = DWM3000.readRXTimestamp();
    tx = DWM3000.readTXTimestamp();

    t_replyB = (uint32_t)(tx - rx);
    curr_stage = 2;
    last_ranging_time = millis(); // Reset timeout timer for Case 2
    Serial.printf("Stage1 from tag=%d to anchor=%d\n", src, dst);
    break;

  case 2: // Await stage 3 from the same tag (sessionTagId) addressed to this anchor
{
  rx_status = DWM3000.receivedFrameSucc();

  if (rx_status != 0)
  {
    int stage = DWM3000.ds_getStage();
    int src   = DWM3000.getSenderID();
    int dst   = DWM3000.getDestinationID();
    bool err  = DWM3000.ds_isErrorFrame();

    if (rx_status == 2)
    {
      DWM3000.writeFastCommand(0x00);
      DWM3000.clearSystemStatus();
      DWM3000.standardRX();
      curr_stage = 0;
      sessionTagId = -1;
      break;
    }

    if (rx_status == 1 && !err && stage == 3 && dst == ANCHOR_ID && src == sessionTagId)
    {
      curr_stage = 3;
    }
    else
    {
      // Wrong packet: reset this attempt
      curr_stage = 0;
      sessionTagId = -1;
      DWM3000.standardRX();
    }
  }
  else if (millis() - last_ranging_time > RESPONSE_TIMEOUT_MS)
  {
    curr_stage = 0;
    sessionTagId = -1;
    DWM3000.writeFastCommand(0x00);
    DWM3000.clearSystemStatus();
    DWM3000.standardRX();
  }
}
break;

  case 3: // Second response received. Sending information frame
    rx = DWM3000.readRXTimestamp();
    t_roundB = (uint32_t)(rx - tx);
    DWM3000.ds_sendRTInfo(t_roundB, t_replyB);

    curr_stage = 0;
    DWM3000.standardRX();
    last_ranging_time = millis(); // Reset the watchdog for the next cycle
    break;

  default:
    curr_stage = 0;
    DWM3000.writeFastCommand(0x00);
    DWM3000.standardRX();
    break;
  }
}