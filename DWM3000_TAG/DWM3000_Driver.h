#pragma once

#include <Arduino.h>
#include <SPI.h>
#include "DWM3000_registers.h" // Brings in your hardware hex dictionary
#include <stddef.h>

extern uint8_t config[7];

class DWM3000Class
{
public:

   // Chip Setup
   static void spiSelect(uint8_t cs);
   static void begin();
   static void init();
   static void writeSysConfig();
   static void configureAsTX();
   static void setupGPIO();

    // Fast Commands
   static void writeFastCommand(int cmd);

   // Double-Sided Ranging
   static void ds_sendFrame(int stage);
   static void ds_sendRTInfo(int t_roundB, int t_replyB);
   static int ds_processRTInfo(int t_roundA, int t_replyA, int t_roundB, int t_replyB, int clock_offset);
   static int ds_getStage();
   static bool ds_isErrorFrame();
   static void ds_sendErrorFrame();

   // Radio Settings
   static void setChannel(uint8_t data);
   static void setPreambleLength(uint8_t data);
   static void setPreambleCode(uint8_t data);
   static void setPACSize(uint8_t data);
   static void setDatarate(uint8_t data);
   static void setPHRMode(uint8_t data);
   static void setPHRRate(uint8_t data);

   // Protocol Settings
   static void setMode(int mode);
   static void setFrameLength(int frame_len);
   static void setTXAntennaDelay(int delay);
   static void setSenderID(int senderID);
   static void setDestinationID(int destID);

   // Status Checks
   static int receivedFrameSucc();
   static int sentFrameSucc();
   static int getSenderID();
   static int getDestinationID();
   static bool checkForIDLE();
   static bool checkSPI();

   // Radio Analytics
   static double getSignalStrength();
   static double getFirstPathSignalStrength();
   static int getTXAntennaDelay();
   static long double getClockOffset();
   static long double getClockOffset(int32_t ext_clock_offset);
   static int getRawClockOffset();
   static float getTempInC();
   static unsigned long long readRXTimestamp();
   static unsigned long long readTXTimestamp();

   // Chip Interaction
   static uint32_t write(int base, int sub, uint32_t data, int data_len);
   static uint32_t write(int base, int sub, uint32_t data);
   static uint32_t read(int base, int sub);
   static uint8_t read8bit(int base, int sub);
   static uint32_t readOTP(uint8_t addr);
   static uint32_t read(int base, int sub, int len);

   // Delayed Sending Settings
   static void writeTXDelay(uint32_t delay);
   static void prepareDelayedTX();

   // Radio Stage Settings / Transfer and Receive Modes
   static void delayedTXThenRX();
   static void delayedTX();
   static void standardTX();
   static void standardRX();
   static void TXInstantRX();

   // DWM3000 Firmware Interaction
   static void softReset();
   static void hardReset();
   static void clearSystemStatus();

   // Hardware Status Information
   static void pullLEDHigh(int led);
   static void pullLEDLow(int led);

   // Calculation and Conversion
   static double convertToCM(int DWM3000_ps_units);
   static void calculateTXRXdiff();

   // Printing
   static void printRoundTripInformation();
   static void printDouble(double val, unsigned int precision, bool linebreak);

   // --- New Explicit-Width SPI API ---
   static void write8(uint8_t base, uint16_t sub, uint8_t v);
   static void write16(uint8_t base, uint16_t sub, uint16_t v);
   static void write32(uint8_t base, uint16_t sub, uint32_t v);

   static uint8_t read8(uint8_t base, uint16_t sub);
   static uint32_t read32(uint8_t base, uint16_t sub); 
   
   static constexpr uint16_t DEFAULT_ANTENNA_DELAY = 16350;
   
   

private:
   static constexpr uint32_t UWB_SPI_HZ = 8000000;
   static constexpr uint8_t CHIP_SELECT_PIN = 4;
   static constexpr uint8_t RST_PIN = 27;

   static uint16_t ACTIVE_ANTENNA_DELAY;
   static uint8_t sender;
   static uint8_t destination;
   static bool DEBUG_OUTPUT;

   // Single Bit Settings
   static void setBit(int reg_addr, int sub_addr, int shift, bool b);
   static void setBitLow(int reg_addr, int sub_addr, int shift);
   static void setBitHigh(int reg_addr, int sub_addr, int shift);

   // SPI Interaction
   // --- New Low-Level SPI Engines ---
   static size_t buildHeader(uint8_t* hdr, uint8_t base, uint16_t sub, bool isWrite);
   static uint32_t spiTxRx(const uint8_t* tx, size_t txLen, uint8_t* rx, size_t rxLen);
   static void writeBytes(uint8_t base, uint16_t sub, const uint8_t* data, size_t len);
   static void readBytes(uint8_t base, uint16_t sub, uint8_t* out, size_t len);

   // Soft Reset Helper Method
   static void clearAONConfig();

   // Other Helper Methods
   static int checkForDevID();
};

// This tells your main sketch that the DWM3000 object exists and is ready to be used
extern DWM3000Class DWM3000;