#include <Arduino.h>
#include <SPI.h>
#include "DWM3000_Driver.h"
#include "DWM3000_registers.h"

// SPI Setup
#define RST_PIN 27
#define CHIP_SELECT_PIN 4

// Set to 1 for Anchor 1, 2 for Anchor 2
#define ANCHOR_ID 2
#define RESPONSE_TIMEOUT_MS 60 // Maximum time to wait for a response
//#define MAX_RETRIES 3 // commenting out for now, could be causing an issue with desync not being able to recover

unsigned long last_ranging_time = 0;
//int retry_count = 0; //commenting out for now, it could be causing issues with desync not being able to recover
static int rx_status;
static int tx_status;
static int curr_stage = 0;
static int t_roundB = 0;
static int t_replyB = 0;
static long long rx = 0;
static long long tx = 0;

// Initial Radio Configuration
int config[] = {
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
  if (DWM3000.receivedFrameSucc() == 1 && DWM3000.ds_getStage() == 1 && DWM3000.getDestinationID() == ANCHOR_ID)
  {
    // Reset session if new ranging request arrives out of sequence
    if (curr_stage != 0)
    {
      curr_stage = 0;
      t_roundB = 0;
      t_replyB = 0;
    }
  }

  switch (curr_stage)
  {
  case 0: // Await ranging
    t_roundB = 0;
    t_replyB = 0;

    if (rx_status = DWM3000.receivedFrameSucc())
    {
      DWM3000.clearSystemStatus();
      if (rx_status == 1)
      { 
        if (DWM3000.getDestinationID() == ANCHOR_ID)
        {
          if (DWM3000.ds_isErrorFrame())
          {
            curr_stage = 0;
            DWM3000.standardRX();
          }
          else if (DWM3000.ds_getStage() != 1)
          {
            DWM3000.ds_sendErrorFrame();
            DWM3000.standardRX();
            curr_stage = 0;
          }
          else
          {
            curr_stage = 1;
            last_ranging_time = millis(); // Start the 60ms stopwatch for Case 1!
          }
        }
        else
        {
          DWM3000.standardRX();
        }
      }
      else
      {
        // OUT OF BOUNDS FIX: Flush hardware and turn the antenna back on!
        DWM3000.clearSystemStatus();
        DWM3000.writeFastCommand(0x00); // Force TRX Off
        DWM3000.standardRX();           // Turn the ear back on!
      }
    }
    // THE WATCHDOG RECOVERY FIX
    else if (millis() - last_ranging_time > 1000)
    {
      // If 1 full second passes with zero valid packets, reboot the radio listener!
      DWM3000.writeFastCommand(0x00);
      DWM3000.clearSystemStatus();
      DWM3000.standardRX();
      last_ranging_time = millis(); // Reset watchdog
    }
    break;

  case 1: // Ranging received. Sending response
    DWM3000.ds_sendFrame(2);

    rx = DWM3000.readRXTimestamp();
    tx = DWM3000.readTXTimestamp();

    t_replyB = tx - rx;
    curr_stage = 2;
    last_ranging_time = millis(); // Reset timeout timer for Case 2
    break;

  case 2: // Awaiting response
    if (rx_status = DWM3000.receivedFrameSucc())
    {
      DWM3000.clearSystemStatus();
      if (rx_status == 1)
      { 
        if (DWM3000.ds_isErrorFrame())
        {
          curr_stage = 0;
          DWM3000.standardRX();
        }
        else if (DWM3000.ds_getStage() != 3)
        {
          DWM3000.ds_sendErrorFrame();
          DWM3000.standardRX();
          curr_stage = 0;
        }
        else
        {
          curr_stage = 3;
        }
      }
      else
      {
        // OUT OF BOUNDS FIX
        DWM3000.clearSystemStatus();
        DWM3000.writeFastCommand(0x00); 
        DWM3000.standardRX(); 
        curr_stage = 0;
      }
    }
    else if (millis() - last_ranging_time > RESPONSE_TIMEOUT_MS) // Uses the 60ms limit!
    {
      curr_stage = 0;
      DWM3000.writeFastCommand(0x00);
      DWM3000.clearSystemStatus();
      DWM3000.standardRX();
    }
    break;

  case 3: // Second response received. Sending information frame
    rx = DWM3000.readRXTimestamp();
    t_roundB = rx - tx;
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