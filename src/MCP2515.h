// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ARDUINO_ARCH_ESP32

#ifndef MCP2515_H
#define MCP2515_H

#include <SPI.h>
#include <functional>

#include "CANController.h"


#define MCP2515_DEFAULT_CLOCK_FREQUENCY 16e6

#if defined(ARDUINO_ARCH_SAMD) && defined(PIN_SPI_MISO) && defined(PIN_SPI_MOSI) && defined(PIN_SPI_SCK) && (PIN_SPI_MISO == 10) && (PIN_SPI_MOSI == 8) && (PIN_SPI_SCK == 9)
// Arduino MKR board: MKR CAN shield CS is pin 3, INT is pin 7
#define MCP2515_DEFAULT_CS_PIN          3
#define MCP2515_DEFAULT_INT_PIN         7
#else
#define MCP2515_DEFAULT_CS_PIN          10
#define MCP2515_DEFAULT_INT_PIN         2
#endif

class MCP2515Class : public CANControllerClass {

public:
  MCP2515Class();
  virtual ~MCP2515Class();

  virtual int begin(long baudRate);
  virtual void end();

  virtual int endPacket();

  virtual int parsePacket();

  virtual void onReceive(std::function<void(int)> callback);

  using CANControllerClass::filter;
  virtual int filter( int id, int mask);
  using CANControllerClass::filterExtended;
  virtual int filterExtended( long id, long mask);

  using CANControllerClass::filterN;
  virtual int filterN(FilterN filter);
  void setMask(uint8_t mskIdx, int msk);
  void setAcceptanceFilter(uint8_t acceptanceIdx, int filter);
  using CANControllerClass::filterExtendedN;
  virtual int filterExtendedN(FilterExtendedN filter);
  void setMaskExtended(uint8_t mskIdx, int msk);
  void setAcceptanceFilterExtended(uint8_t acceptanceIdx, int filter);

  uint8_t hitB0();
  uint8_t hitB1();
  MCP2515Class* mpc2515(){return this;};

  virtual int observe();
  virtual int loopback();
  virtual int sleep();
  virtual int wakeup();

  void setPins(int cs = MCP2515_DEFAULT_CS_PIN, int irq = MCP2515_DEFAULT_INT_PIN);
  void setPins(int cs, int csX, int irq);
  void setSPIFrequency(uint32_t frequency);
  void setClockFrequency(long clockFrequency);

  void dumpRegisters(Stream& out);
  void dumpRegisters();

private:
  void reset();

  void handleInterrupt();

  uint8_t readRegister(uint8_t address);
  void modifyRegister(uint8_t address, uint8_t mask, uint8_t value);
  void writeRegister(uint8_t address, uint8_t value);
  void csSelect();
  void csUnselect();

  int setConfigMode();
  int setNormalMode();

  static void onInterrupt1();
  static void onInterrupt2();

private:
  SPISettings _spiSettings;
  int _csPin;
  int _csPinX;
  int _intPin;
  long _clockFrequency;
  // See PinDescription in variant.cpp  : D6 is on PA20, D7 on PA21, mask is 1<<20 | 1 << 21;  
  //.platformio\packages\framework-arduino-samd\variants\mkrwifi1010\variant.cpp
  const uint32_t MUX_IOPIN_MASK = 0x00300000; 
};

extern MCP2515Class* Mpc1Ref;
extern MCP2515Class* Mpc2Ref;

#endif

#endif
