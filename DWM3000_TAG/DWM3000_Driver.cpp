#include "DWM3000_Driver.h"
#include "DWM3000_registers.h"

// Initialize the global instance of the radio
DWM3000Class DWM3000;

// SPI Setup
#define RST_PIN 27
#define CHIP_SELECT_PIN 4

static int ANTENNA_DELAY = 16350;
int led_status = 0;
int destination = 0x0;
int sender = 0x0;

// Implementation of DWM3000Class methods
void DWM3000Class::spiSelect(uint8_t cs)
{
   pinMode(cs, OUTPUT);
   digitalWrite(cs, HIGH);
   delay(5);
}

void DWM3000Class::begin()
{
   delay(5);
   pinMode(CHIP_SELECT_PIN, OUTPUT);
   SPI.begin();
   delay(5);
   spiSelect(CHIP_SELECT_PIN);
   Serial.println("[INFO] SPI ready");
}

void DWM3000Class::init()
{
   if (!checkForDevID())
   {
       Serial.println("[ERROR] Dev ID is wrong! Aborting!");
       return;
   }

   setBitHigh(GEN_CFG_AES_LOW_REG, 0x10, 4);

   while (!checkForIDLE())
   {
       Serial.println("[WARNING] IDLE FAILED (stage 1)");
       delay(100);
   }

   softReset();
   delay(200);

   while (!checkForIDLE())
   {
       Serial.println("[WARNING] IDLE FAILED (stage 2)");
       delay(100);
   }

   uint32_t ldo_low = readOTP(0x04);
   uint32_t ldo_high = readOTP(0x05);
   uint32_t bias_tune = readOTP(0xA);
   bias_tune = (bias_tune >> 16) & BIAS_CTRL_BIAS_MASK;

   if (ldo_low != 0 && ldo_high != 0 && bias_tune != 0)
   {
       write(0x11, 0x1F, bias_tune);
       write(0x0B, 0x08, 0x0100);
   }

   int xtrim_value = readOTP(0x1E);
   xtrim_value = xtrim_value == 0 ? 0x2E : xtrim_value;
   write(FS_CTRL_REG, 0x14, xtrim_value);
   if (DEBUG_OUTPUT)
       Serial.print("xtrim: ");
   if (DEBUG_OUTPUT)
       Serial.println(xtrim_value);

   writeSysConfig();
   write(0x00, 0x3C, 0xFFFFFFFF);
   write(0x00, 0x40, 0xFFFF);
   write(0x0A, 0x00, 0x000900, 3);

   write(0x3, 0x1C, 0x10000240);
   write(0x3, 0x20, 0x1B6DA489);
   write(0x3, 0x38, 0x0001C0FD);
   write(0x3, 0x3C, 0x0001C43E);
   write(0x3, 0x40, 0x0001C6BE);
   write(0x3, 0x44, 0x0001C77E);
   write(0x3, 0x48, 0x0001CF36);
   write(0x3, 0x4C, 0x0001CFB5);
   write(0x3, 0x50, 0x0001CFF5);
   write(0x3, 0x18, 0xE5E5);
   int f = read(0x4, 0x20);
   write(0x6, 0x0, 0x81101C);
   write(0x07, 0x34, 0x4);
   write(0x07, 0x48, 0x14);
   write(0x07, 0x1A, 0x0E);
   write(0x07, 0x1C, 0x1C071134);
   write(0x09, 0x00, 0x1F3C);
   write(0x09, 0x80, 0x81);
   write(0x11, 0x04, 0xB40200);
   write(0x11, 0x08, 0x80030738);
   Serial.println("[INFO] Initialization finished.\n");
}

/*
Writes the initial configuration to the chip
*/
void DWM3000Class::writeSysConfig()
{
   int usr_cfg = (STDRD_SYS_CONFIG & 0xFFF) | (config[5] << 3) | (config[6] << 4);

   write(GEN_CFG_AES_LOW_REG, 0x10, usr_cfg);

   if (config[2] > 24)
   {
      Serial.println("[ERROR] SCP ERROR! TX & RX Preamble Code higher than 24!");
   }

   int otp_write = 0x1400;

   if (config[1] >= 256)
   {
      otp_write |= 0x04;
   }

   write(OTP_IF_REG, 0x08, otp_write); // set OTP config
   write(DRX_REG, 0x00, 0x00, 1);      // reset DTUNE0_CONFIG

   write(DRX_REG, 0x0, config[3]);

   // 64 = STS length
   write(STS_CFG_REG, 0x0, 64 / 8 - 1);

   write(GEN_CFG_AES_LOW_REG, 0x29, 0x00, 1);

   write(DRX_REG, 0x0C, 0xAF5F584C);

   int chan_ctrl_val = read(GEN_CFG_AES_HIGH_REG, 0x14); // Fetch and adjust CHAN_CTRL data
   chan_ctrl_val &= (~0x1FFF);

   chan_ctrl_val |= config[0]; // Write RF_CHAN

   chan_ctrl_val |= 0x1F00 & (config[2] << 8);
   chan_ctrl_val |= 0xF8 & (config[2] << 3);
   chan_ctrl_val |= 0x06 & (0x01 << 1);

   write(GEN_CFG_AES_HIGH_REG, 0x14, chan_ctrl_val); // Write new CHAN_CTRL data with updated values

   int tx_fctrl_val = read(GEN_CFG_AES_LOW_REG, 0x24);

   tx_fctrl_val |= (config[1] << 12); // Add preamble length
   tx_fctrl_val |= (config[4] << 10); // Add data rate

   write(GEN_CFG_AES_LOW_REG, 0x24, tx_fctrl_val);

   write(DRX_REG, 0x02, 0x81);

   int rf_tx_ctrl_2 = 0x1C071134;
   int pll_conf = 0x0F3C;

   if (config[0])
   {
      rf_tx_ctrl_2 &= ~0x00FFFF;
      rf_tx_ctrl_2 |= 0x000001;
      pll_conf &= 0x00FF;
      pll_conf |= 0x001F;
   }

   write(RF_CONF_REG, 0x1C, rf_tx_ctrl_2);
   write(FS_CTRL_REG, 0x00, pll_conf);

   write(RF_CONF_REG, 0x51, 0x14);

   write(RF_CONF_REG, 0x1A, 0x0E);

   write(FS_CTRL_REG, 0x08, 0x81);

   write(GEN_CFG_AES_LOW_REG, 0x44, 0x02);

   write(PMSC_REG, 0x04, 0x300200); // Set clock to auto mode

   write(PMSC_REG, 0x08, 0x0138);

   int success = 0;
   for (int i = 0; i < 100; i++)
   {
      if (read(GEN_CFG_AES_LOW_REG, 0x0) & 0x2)
   {
      success = 1;
      break;
   }
   }

   if (!success)
   {
      Serial.println("[ERROR] Couldn't lock PLL Clock!");
   }else {
      Serial.println("[INFO] PLL is now locked.");
   }

   int otp_val = read(OTP_IF_REG, 0x08);
   otp_val |= 0x40;
   if (config[0])
      otp_val |= 0x2000;

   write(OTP_IF_REG, 0x08, otp_val);

   write(RX_TUNE_REG, 0x19, 0xF0);

   int ldo_ctrl_val = read(RF_CONF_REG, 0x48); // Save original LDO_CTRL data
   int tmp_ldo = (0x105 | 0x100 | 0x4 | 0x1);

   write(RF_CONF_REG, 0x48, tmp_ldo);

   write(EXT_SYNC_REG, 0x0C, 0x020000); // Calibrate RX

   int l = read(0x04, 0x0C);

   delay(20);

   write(EXT_SYNC_REG, 0x0C, 0x11); // Enable calibration

   int succ = 0;
   for (int i = 0; i < 100; i++)
   {
      if (read(EXT_SYNC_REG, 0x20))
      {
         succ = 1;
         break;
      }
      delay(10);
   }

   if (succ)
   {
      Serial.println("[INFO] PGF calibration complete.");
   }else {
      Serial.println("[ERROR] PGF calibration failed!");
   }

   write(EXT_SYNC_REG, 0x0C, 0x00);
   write(EXT_SYNC_REG, 0x20, 0x01);

   int rx_cal_res = read(EXT_SYNC_REG, 0x14);
   if (rx_cal_res == 0x1fffffff)
   {
      Serial.println("[ERROR] PGF_CAL failed in stage I!");
   }
   rx_cal_res = read(EXT_SYNC_REG, 0x1C);
   if (rx_cal_res == 0x1fffffff)
   {
      Serial.println("[ERROR] PGF_CAL failed in stage Q!");
   }

   write(RF_CONF_REG, 0x48, ldo_ctrl_val); // Restore original LDO_CTRL data

   write(0x0E, 0x02, 0x01); // Enable full CIA diagnostics to get signal strength information

   setTXAntennaDelay(ANTENNA_DELAY); // set default antenna delay
}

void DWM3000Class::configureAsTX()
{
   write(RF_CONF_REG, 0x1C, 0x34);
   write(GEN_CFG_AES_HIGH_REG, 0x0C, 0xFDFDFDFD);
}

void DWM3000Class::setupGPIO()
{
   write(0x05, 0x08, 0xF0);
}

void DWM3000Class::ds_sendFrame(int stage)
{
   setMode(1);
   write(0x14, 0x01, sender & 0xFF);
   write(0x14, 0x02, destination & 0xFF);
   write(0x14, 0x03, stage & 0x7);
   setFrameLength(4);
   TXInstantRX();

   bool error = true;
   for (int i = 0; i < 50; i++)
   {
       if (sentFrameSucc())
       {
           error = false;
           break;
       }
   };
   if (error)
   {
       Serial.println("[ERROR] Could not send frame successfully!");
   }
}

void DWM3000Class::ds_sendRTInfo(int t_roundB, int t_replyB)
{
   setMode(1);
   write(0x14, 0x01, destination & 0xFF);
   write(0x14, 0x02, sender & 0xFF);
   write(0x14, 0x03, 4);
   write(0x14, 0x04, t_roundB);
   write(0x14, 0x08, t_replyB);
   setFrameLength(12);
   TXInstantRX();
}

int DWM3000Class::ds_processRTInfo(int t_roundA, int t_replyA, int t_roundB, int t_replyB, int clk_offset)
{
   if (DEBUG_OUTPUT)
   {
       Serial.println("\nProcessing Information:");
       Serial.print("t_roundA: ");
       Serial.println(t_roundA);
       Serial.print("t_replyA: ");
       Serial.println(t_replyA);
       Serial.print("t_roundB: ");
       Serial.println(t_roundB);
       Serial.print("t_replyB: ");
       Serial.println(t_replyB);
   }

   int reply_diff = t_replyA - t_replyB;
   long double clock_offset = t_replyA > t_replyB ? 1.0 + getClockOffset(clk_offset) : 1.0 - getClockOffset(clk_offset);
   int first_rt = t_roundA - t_replyB;
   int second_rt = t_roundB - t_replyA;
   int combined_rt = (first_rt + second_rt - (reply_diff - (reply_diff * clock_offset))) / 2;
   int combined_rt_raw = (first_rt + second_rt) / 2;
   return combined_rt / 2;
}

int DWM3000Class::ds_getStage()
{
   return read(0x12, 0x03) & 0b111;
}

bool DWM3000Class::ds_isErrorFrame()
{
   return ((read(0x12, 0x00) & 0x7) == 7);
}

void DWM3000Class::ds_sendErrorFrame()
{
   Serial.println("[WARNING] Error Frame sent. Reverting back to stage 0.");
   setMode(7);
   setFrameLength(3);
   standardTX();
}

void DWM3000Class::setChannel(uint8_t data)
{
   if (data == CHANNEL_5 || data == CHANNEL_9)
       config[0] = data;
}

void DWM3000Class::setPreambleLength(uint8_t data)
{
   if (data == PREAMBLE_32 || data == PREAMBLE_64 || data == PREAMBLE_1024 || data == PREAMBLE_256 || data == PREAMBLE_512 || data == PREAMBLE_1024 || data == PREAMBLE_1536 || data == PREAMBLE_2048 || data == PREAMBLE_4096)
       config[1] = data;
}

void DWM3000Class::setPreambleCode(uint8_t data)
{
   if (data <= 12 && data >= 9)
       config[2] = data;
}

void DWM3000Class::setPACSize(uint8_t data)
{
   if (data == PAC4 || data == PAC8 || data == PAC16 || data == PAC32)
       config[3] = data;
}

void DWM3000Class::setDatarate(uint8_t data)
{
   if (data == DATARATE_6_8MB || data == DATARATE_850KB)
       config[4] = data;
}

void DWM3000Class::setPHRMode(uint8_t data)
{
   if (data == PHR_MODE_STANDARD || data == PHR_MODE_LONG)
       config[5] = data;
}

void DWM3000Class::setPHRRate(uint8_t data)
{
   if (data == PHR_RATE_6_8MB || data == PHR_RATE_850KB)
       config[6] = data;
}

void DWM3000Class::setMode(int mode)
{
   write(0x14, 0x00, mode & 0x7);
}

void DWM3000Class::setTXFrame(unsigned long long frame_data)
{
   if (frame_data > ((pow(2, 8 * 8) - FCS_LEN)))
   {
       Serial.println("[ERROR] Frame is too long (> 1023 Bytes - FCS_LEN)!");
       return;
   }
   write(TX_BUFFER_REG, 0x00, frame_data);
}

void DWM3000Class::setFrameLength(int frameLen)
{
   frameLen = frameLen + FCS_LEN;
   int curr_cfg = read(0x00, 0x24);
   if (frameLen > 1023)
   {
       Serial.println("[ERROR] Frame length + FCS_LEN (2) is longer than 1023. Aborting!");
       return;
   }
   int tmp_cfg = (curr_cfg & 0xFFFFFC00) | frameLen;
   write(GEN_CFG_AES_LOW_REG, 0x24, tmp_cfg);
}

void DWM3000Class::setTXAntennaDelay(int delay)
{
   ANTENNA_DELAY = delay;
   write(0x01, 0x04, delay);
}

void DWM3000Class::setSenderID(int senderID)
{
   sender = senderID;
}

void DWM3000Class::setDestinationID(int destID)
{
   destination = destID;
}

int DWM3000Class::receivedFrameSucc()
{
   int sys_stat = read(GEN_CFG_AES_LOW_REG, 0x44);
   if ((sys_stat & SYS_STATUS_FRAME_RX_SUCC) > 0)
   {
       return 1;
   }
   else if ((sys_stat & SYS_STATUS_RX_ERR) > 0)
   {
       return 2;
   }
   return 0;
}

int DWM3000Class::sentFrameSucc()
{
   int sys_stat = read(GEN_CFG_AES_LOW_REG, 0x44);
   if ((sys_stat & SYS_STATUS_FRAME_TX_SUCC) == SYS_STATUS_FRAME_TX_SUCC)
   {
       return 1;
   }
   return 0;
}

int DWM3000Class::getSenderID()
{
   return read(0x12, 0x01) & 0xFF;
}

int DWM3000Class::getDestinationID()
{
   return read(0x12, 0x02) & 0xFF;
}

bool DWM3000Class::checkForIDLE()
{
   return (read(0x0F, 0x30) >> 16 & PMSC_STATE_IDLE) == PMSC_STATE_IDLE || (read(0x00, 0x44) >> 16 & (SPIRDY_MASK | RCINIT_MASK)) == (SPIRDY_MASK | RCINIT_MASK) ? 1 : 0;
}

bool DWM3000Class::checkSPI()
{
   return checkForDevID();
}

double DWM3000Class::getSignalStrength()
{
   int CIRpower = read(0x0C, 0x2C) & 0x1FF;
   int PAC_val = read(0x0C, 0x58) & 0xFFF;
   unsigned int DGC_decision = (read(0x03, 0x60) >> 28) & 0x7;
   double PRF_const = 121.7;
   return 10 * log10((CIRpower * (1 << 21)) / pow(PAC_val, 2)) + (6 * DGC_decision) - PRF_const;
}

double DWM3000Class::getFirstPathSignalStrength()
{
   float f1 = (read(0x0C, 0x30) & 0x3FFFFF) >> 2;
   float f2 = (read(0x0C, 0x34) & 0x3FFFFF) >> 2;
   float f3 = (read(0x0C, 0x38) & 0x3FFFFF) >> 2;
   int PAC_val = read(0x0C, 0x58) & 0xFFF;
   unsigned int DGC_decision = (read(0x03, 0x60) >> 28) & 0x7;
   double PRF_const = 121.7;
   return 10 * log10((pow(f1, 2) + pow(f2, 2) + pow(f3, 2)) / pow(PAC_val, 2)) + (6 * DGC_decision) - PRF_const;
}

int DWM3000Class::getTXAntennaDelay()
{
   int delay = read(0x01, 0x04) & 0xFFFF;
   return delay;
}

long double DWM3000Class::getClockOffset()
{
   if (config[0] == CHANNEL_5)
   {
       return getRawClockOffset() * CLOCK_OFFSET_CHAN_5_CONSTANT / 1000000;
   }
   else
   {
       return getRawClockOffset() * CLOCK_OFFSET_CHAN_9_CONSTANT / 1000000;
   }
}

long double DWM3000Class::getClockOffset(int32_t sec_clock_offset)
{
   if (config[0] == CHANNEL_5)
   {
       return sec_clock_offset * CLOCK_OFFSET_CHAN_5_CONSTANT / 1000000;
   }
   else
   {
       return sec_clock_offset * CLOCK_OFFSET_CHAN_9_CONSTANT / 1000000;
   }
}

int DWM3000Class::getRawClockOffset()
{
   int raw_offset = read(0x06, 0x29) & 0x1FFFFF;
   if (raw_offset & (1 << 20))
   {
       raw_offset |= ~((1 << 21) - 1);
   }
   if (DEBUG_OUTPUT)
   {
       Serial.print("Raw offset: ");
       Serial.println(raw_offset);
   }
   return raw_offset;
}

float DWM3000Class::getTempInC()
{
   write(0x07, 0x34, 0x04);
   write(0x08, 0x00, 0x01);
   while (!(read(0x08, 0x04) & 0x01))
   {
   };
   int res = read(0x08, 0x08);
   res = (res & 0xFF00) >> 8;
   int otp_temp = readOTP(0x09) & 0xFF;
   float tmp = (float)((res - otp_temp) * 1.05f) + 22.0f;
   write(0x08, 0x00, 0x00, 1);
   return tmp;
}

unsigned long long DWM3000Class::readRXTimestamp()
{
   uint32_t ts_low = read(0x0C, 0x00);
   unsigned long long ts_high = read(0x0C, 0x04) & 0xFF;
   unsigned long long rx_timestamp = (ts_high << 32) | ts_low;
   return rx_timestamp;
}

unsigned long long DWM3000Class::readTXTimestamp()
{
   unsigned long long ts_low = read(0x00, 0x74);
   unsigned long long ts_high = read(0x00, 0x78) & 0xFF;
   unsigned long long tx_timestamp = (ts_high << 32) + ts_low;
   return tx_timestamp;
}

uint32_t DWM3000Class::write(int base, int sub, uint32_t data, int dataLen)
{
   return readOrWriteFullAddress(base, sub, data, dataLen, 1);
}

uint32_t DWM3000Class::write(int base, int sub, uint32_t data)
{
   return readOrWriteFullAddress(base, sub, data, 0, 1);
}

uint32_t DWM3000Class::read(int base, int sub)
{
   uint32_t tmp;
   tmp = readOrWriteFullAddress(base, sub, 0, 0, 0);
   if (DEBUG_OUTPUT)
       Serial.println("");
   return tmp;
}

uint8_t DWM3000Class::read8bit(int base, int sub)
{
   return (uint8_t)(read(base, sub) >> 24);
}

uint32_t DWM3000Class::readOTP(uint8_t addr)
{
   write(OTP_IF_REG, 0x04, addr);
   write(OTP_IF_REG, 0x08, 0x02);
   return read(OTP_IF_REG, 0x10);
}

void DWM3000Class::writeTXDelay(uint32_t delay)
{
   write(0x00, 0x2C, delay);
}

/*
A delayed TX is typically performed to have a known delay between sender and receiver.
As the chips delayedTX function has a lower timestamp resolution, the frame gets sent
always a little bit earlier than its supposed to be. For example:
Delay: 10ms. Receive Time of Frame: 2010us (2.01ms). It should send at 12.01ms, but will be
sent at 12ms as the last digits get cut off. These last digits can be precalculated and
sent inside the frame to be added to the RX Timestamp of the receiver. In the above case, 0.01ms
The DX_Time resolution is ~4ns, the chips internal clock resolution is ~15.65ps.

This function calculates the missing delay time, adds it to the frames payload and sets the fixed delay (TRANSMIT_DELAY).
*/

void DWM3000Class::prepareDelayedTX()
{
   long long rx_ts = readRXTimestamp();
   uint32_t exact_tx_timestamp = (long long)(rx_ts + TRANSMIT_DELAY) >> 8;
   long long calc_tx_timestamp = ((rx_ts + TRANSMIT_DELAY) & ~TRANSMIT_DIFF) + ANTENNA_DELAY;
   uint32_t reply_delay = calc_tx_timestamp - rx_ts;

    /*
   * PAYLOAD DESIGN:
   +------+-----------------------------------------------------------------------+-------------------------------+-------------------------------+------+------+------+-----+
   | Byte |                                 1 (0x00)                              |           2 (0x01)            |           3 (0x02)            |     4 - 6 (0x03-0x05)    |
   +------+-----+-----+----+----------+----------+----------+----------+----------+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+------+------+------+-----+
   | Bits |  1  |  2  |  3 |     4    |     5    |     6    |     7    |     8    | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |                          |
   +------+-----+-----+----+----------+----------+----------+----------+----------+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+---+--------------------------+
   |      | Mode bits:     | Reserved | Reserved | Reserved | Reserved | Reserved |           Sender ID           |         Destination ID        | Internal Delay / Payload |
   |      | 0 - Standard   |          |          |          |          |          |                               |                               |                          |
   |      |1-7 - See below |          |          |          |          |          |                               |                               |                          |
   +------+----------------+----------+----------+----------+----------+----------+-------------------------------+-------------------------------+--------------------------+
   *
   * Mode bits:
   * 0 - Standard
   * 1 - Double Sided Ranging
   * 2-6 - Reserved
   * 7 - Error
   */

   write(0x14, 0x01, sender & 0xFF);
   write(0x14, 0x02, destination & 0xFF);
   write(0x14, 0x03, reply_delay);
   setFrameLength(7);
   writeTXDelay(exact_tx_timestamp);
}

void DWM3000Class::delayedTXThenRX()
{
   writeFastCommand(0x0F);
}

void DWM3000Class::delayedTX()
{
   writeFastCommand(0x3);
}

void DWM3000Class::standardTX()
{
   writeFastCommand(0x01);
}

void DWM3000Class::standardRX()
{
   writeFastCommand(0x02);
}

void DWM3000Class::TXInstantRX()
{
   writeFastCommand(0x0C);
}

void DWM3000Class::softReset()
{
   clearAONConfig();
   write(PMSC_REG, 0x04, 0x1);
   write(PMSC_REG, 0x00, 0x00, 2);
   delay(100);
   write(PMSC_REG, 0x00, 0xFFFF);
   write(PMSC_REG, 0x04, 0x00, 1);
}

void DWM3000Class::hardReset()
{
   pinMode(RST_PIN, OUTPUT);
   digitalWrite(RST_PIN, LOW);
   delay(10);
   pinMode(RST_PIN, INPUT);
}

void DWM3000Class::clearSystemStatus()
{
   write(GEN_CFG_AES_LOW_REG, 0x44, 0x3F7FFFFF);
}

void DWM3000Class::pullLEDHigh(int led)
{
   if (led > 2)
       return;
   led_status = led_status | (1 << led);
   write(0x05, 0x0C, led_status);
}

void DWM3000Class::pullLEDLow(int led)
{
   if (led > 2)
       return;
   led_status = led_status & ~((int)1 << led);
   write(0x05, 0x0C, led_status);
}

double DWM3000Class::convertToCM(int DWM3000_ps_units)
{
   return (double)DWM3000_ps_units * PS_UNIT * SPEED_OF_LIGHT;
}

void DWM3000Class::calculateTXRXdiff()
{
   unsigned long long ping_tx = readTXTimestamp();
   unsigned long long ping_rx = readRXTimestamp();
   long double clk_offset = getClockOffset();
   long double clock_offset = 1.0 + clk_offset;
   long long t_reply = read(RX_BUFFER_0_REG, 0x03);

   if (t_reply == 0)
   {
       return;
   }

   long long t_round = ping_rx - ping_tx;
   long long t_prop = lround((t_round - lround(t_reply * clock_offset)) / 2);
   long double t_prop_ps = t_prop * PS_UNIT;
   long double t_prop_cm = t_prop_ps * SPEED_OF_LIGHT;
   if (t_prop_cm >= 0)
   {
       printDouble(t_prop_cm, 100, false);
       Serial.println("cm");
   }
}

void DWM3000Class::printRoundTripInformation()
{
   Serial.println("\nRound Trip Information:");
   long long tx_ts = readTXTimestamp();
   long long rx_ts = readRXTimestamp();
   Serial.print("TX Timestamp: ");
   Serial.println(tx_ts);
   Serial.print("RX Timestamp: ");
   Serial.println(rx_ts);
}

void DWM3000Class::printDouble(double val, unsigned int precision, bool linebreak)
{
   Serial.print(int(val));
   Serial.print(".");
   unsigned int frac;
   if (val >= 0)
   {
       frac = (val - int(val)) * precision;
   }
   else
   {
       frac = (int(val) - val) * precision;
   }
   if (linebreak)
   {
       Serial.println(frac, DEC);
   }
   else
   {
       Serial.print(frac, DEC);
   }
}

void DWM3000Class::setBit(int reg_addr, int sub_addr, int shift, bool b)
{
   uint8_t tmpByte = read8bit(reg_addr, sub_addr);
   if (b)
   {
       bitSet(tmpByte, shift);
   }
   else
   {
       bitClear(tmpByte, shift);
   }
   write(reg_addr, sub_addr, tmpByte);
}

void DWM3000Class::setBitLow(int reg_addr, int sub_addr, int shift)
{
   setBit(reg_addr, sub_addr, shift, 0);
}

void DWM3000Class::setBitHigh(int reg_addr, int sub_addr, int shift)
{
   setBit(reg_addr, sub_addr, shift, 1);
}

void DWM3000Class::writeFastCommand(int cmd)
{
   if (DEBUG_OUTPUT)
       Serial.print("[INFO] Executing short command: ");
   int header = 0;
   header = header | 0x1;
   header = header | (cmd & 0x1F) << 1;
   header = header | 0x80;
   if (DEBUG_OUTPUT)
       Serial.println(header, BIN);
   int header_arr[] = {header};
   sendBytes(header_arr, 1, 0);
}

uint32_t DWM3000Class::readOrWriteFullAddress(uint32_t base, uint32_t sub, uint32_t data, uint32_t dataLen, uint32_t readWriteBit)
{
   uint32_t header = 0x00;
   if (readWriteBit)
       header = header | 0x80;
   header = header | ((base & 0x1F) << 1);
   if (sub > 0)
   {
       header = header | 0x40;
       header = header << 8;
       header = header | ((sub & 0x7F) << 2);
   }
   uint32_t header_size = header > 0xFF ? 2 : 1;
   uint32_t res = 0;
   if (!readWriteBit)
   {
       int headerArr[header_size];
       if (header_size == 1)
       {
           headerArr[0] = header;
       }
       else
       {
           headerArr[0] = (header & 0xFF00) >> 8;
           headerArr[1] = header & 0xFF;
       }
       res = (uint32_t)sendBytes(headerArr, header_size, 4);
       return res;
   }
   else
   {
       uint32_t payload_bytes = 0;
       if (dataLen == 0)
       {
           if (data > 0)
           {
               uint32_t payload_bits = countBits(data);
               payload_bytes = (payload_bits - (payload_bits % 8)) / 8;
               if ((payload_bits % 8) > 0)
               {
                   payload_bytes++;
               }
           }
           else
           {
               payload_bytes = 1;
           }
       }
       else
       {
           payload_bytes = dataLen;
       }
       int payload[header_size + payload_bytes];
       if (header_size == 1)
       {
           payload[0] = header;
       }
       else
       {
           payload[0] = (header & 0xFF00) >> 8;
           payload[1] = header & 0xFF;
       }
       for (int i = 0; i < payload_bytes; i++)
       {
           payload[header_size + i] = (data >> i * 8) & 0xFF;
       }
       res = (uint32_t)sendBytes(payload, 2 + payload_bytes, 0);
       return res;
   }
}

uint32_t DWM3000Class::sendBytes(int b[], int lenB, int recLen)
{
   digitalWrite(CHIP_SELECT_PIN, LOW);
   for (int i = 0; i < lenB; i++)
   {
       SPI.transfer(b[i]);
   }
   int rec;
   uint32_t val, tmp;
   if (recLen > 0)
   {
       for (int i = 0; i < recLen; i++)
       {
           tmp = SPI.transfer(0x00);
           if (i == 0)
           {
               val = tmp;
           }
           else
           {
               val |= (uint32_t)tmp << 8 * i;
           }
       }
   }
   else
   {
       val = 0;
   }
   digitalWrite(CHIP_SELECT_PIN, HIGH);
   return val;
}

void DWM3000Class::clearAONConfig()
{
   write(AON_REG, NO_OFFSET, 0x00, 2);
   write(AON_REG, 0x14, 0x00, 1);
   write(AON_REG, 0x04, 0x00, 1);
   write(AON_REG, 0x04, 0x02);
   delay(1);
}

unsigned int DWM3000Class::countBits(unsigned int number)
{
   return (int)log2(number) + 1;
}

int DWM3000Class::checkForDevID()
{
   int res = read(GEN_CFG_AES_LOW_REG, NO_OFFSET);
   if (res != 0xDECA0302 && res != 0xDECA0312)
   {
       Serial.println("[ERROR] DEV_ID IS WRONG!");
       return 0;
   }
   return 1;
}
