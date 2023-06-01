// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ARDUINO_ARCH_ESP32

#include "MCP2515.h"
#include "mux.h"

#define REG_BFPCTRL                0x0c
#define REG_TXRTSCTRL              0x0d

#define REG_CANCTRL                0x0f

#define REG_CNF3                   0x28
#define REG_CNF2                   0x29
#define REG_CNF1                   0x2a

#define REG_CANINTE                0x2b
#define REG_CANINTF                0x2c

#define FLAG_RXnIE(n)              (0x01 << n)
#define FLAG_RXnIF(n)              (0x01 << n)
#define FLAG_TXnIF(n)              (0x04 << n)

// There is a 4-register gap between RXF2 and RXF3!
// n:0:5 for 6 bit filters
#define REG_RXFnSIDH(n)            (0x00 + ((n + (n >= 3)) * 4))  //n:0..5 for nth bit filter
#define REG_RXFnSIDL(n)            (0x01 + ((n + (n >= 3)) * 4))  //n:0..5
#define REG_RXFnEID8(n)            (0x02 + ((n + (n >= 3)) * 4))  //n:0..5
#define REG_RXFnEID0(n)            (0x03 + ((n + (n >= 3)) * 4))  //n:0..5

//n: 0..1 for 2 mask filters
#define REG_RXMnSIDH(n)            (0x20 + (n * 0x04)) //n:0:1 for nth mask filter
#define REG_RXMnSIDL(n)            (0x21 + (n * 0x04)) //n:0:1 for nth mask filter
#define REG_RXMnEID8(n)            (0x22 + (n * 0x04)) //n:0:1 for nth mask filter
#define REG_RXMnEID0(n)            (0x23 + (n * 0x04)) //n:0:1 for nth mask filter

//n: 0..2 for 3 tx box
#define REG_TXBnCTRL(n)            (0x30 + (n * 0x10)) //n=0:2 for nth tx box
#define REG_TXBnSIDH(n)            (0x31 + (n * 0x10)) //n=0:2 for nth tx box
#define REG_TXBnSIDL(n)            (0x32 + (n * 0x10)) //n=0:2 for nth tx box
#define REG_TXBnEID8(n)            (0x33 + (n * 0x10)) //n=0:2 for nth tx box
#define REG_TXBnEID0(n)            (0x34 + (n * 0x10)) //n=0:2 for nth tx box
#define REG_TXBnDLC(n)             (0x35 + (n * 0x10)) //n=0:2 for nth tx box
#define REG_TXBnD0(n)              (0x36 + (n * 0x10)) //n=0:2 for nth tx box
#define REG_TXBnD1(n)              (0x37 + (n * 0x10)) //n=0:2 for nth tx box
#define REG_TXBnD2(n)              (0x38 + (n * 0x10)) //n=0:2 for nth tx box
#define REG_TXBnD3(n)              (0x39 + (n * 0x10)) //n=0:2 for nth tx box
#define REG_TXBnD4(n)              (0x3A + (n * 0x10)) //n=0:2 for nth tx box
#define REG_TXBnD5(n)              (0x3B + (n * 0x10)) //n=0:2 for nth tx box
#define REG_TXBnD6(n)              (0x3C + (n * 0x10)) //n=0:2 for nth tx box
#define REG_TXBnD7(n)              (0x3D + (n * 0x10)) //n=0:2 for nth tx box

//n: 0..1 for 2 rx box
#define REG_RXBnCTRL(n)            (0x60 + (n * 0x10)) //n: 0..1 for 2 rx box
#define REG_RXBnSIDH(n)            (0x61 + (n * 0x10)) //n: 0..1 for 2 rx box
#define REG_RXBnSIDL(n)            (0x62 + (n * 0x10)) //n: 0..1 for 2 rx box
#define REG_RXBnEID8(n)            (0x63 + (n * 0x10)) //n: 0..1 for 2 rx box
#define REG_RXBnEID0(n)            (0x64 + (n * 0x10)) //n: 0..1 for 2 rx box
#define REG_RXBnDLC(n)             (0x65 + (n * 0x10)) //n: 0..1 for 2 rx box
#define REG_RXBnD0(n)              (0x66 + (n * 0x10)) //n: 0..1 for 2 rx box

#define FLAG_IDE                   0x08
#define FLAG_SRR                   0x10
#define FLAG_RTR                   0x40
#define FLAG_EXIDE                 0x08

#define RXBXCTRL_FILTER01_ON_NO_ROLLOVER   0x00 //Enable filters on MBX0 and 1 and disable rollover. See datasheet RXB0CTRL, RXB1CTRL


MCP2515Class::MCP2515Class(CanCmn::eCanBusId busId) :
  CANControllerClass(),
  _spiSettings(4E6, MSBFIRST, SPI_MODE0),
  _csPin(MCP2515_DEFAULT_CS_PIN),
  _intPin(MCP2515_DEFAULT_INT_PIN),
  _clockFrequency(MCP2515_DEFAULT_CLOCK_FREQUENCY),
  _busId(busId)
{
  // start SPI
  SPI.begin(); //shall be done before on receive registration!!! else interruptMode, interruptMask...  are overwritten!!!
}

MCP2515Class::~MCP2515Class()
{
}

int MCP2515Class::begin(long baudRate)
{
  CANControllerClass::begin(baudRate);

  pinMode(_csPin, OUTPUT);

  reset();

  writeRegister(REG_CANCTRL, 0x80);
  if (readRegister(REG_CANCTRL) != 0x80) {
    return 0;
  }

  const struct {
    long clockFrequency;
    long baudRate;
    uint8_t cnf[3];
  } CNF_MAPPER[] = {
    {  (long)8E6, (long)1000E3, { 0x00, 0x80, 0x00 } },
    {  (long)8E6,  (long)500E3, { 0x00, 0x90, 0x02 } },
    {  (long)8E6,  (long)250E3, { 0x00, 0xb1, 0x05 } },
    {  (long)8E6,  (long)200E3, { 0x00, 0xb4, 0x06 } },
    {  (long)8E6,  (long)125E3, { 0x01, 0xb1, 0x05 } },
    {  (long)8E6,  (long)100E3, { 0x01, 0xb4, 0x06 } },
    {  (long)8E6,   (long)80E3, { 0x01, 0xbf, 0x07 } },
    {  (long)8E6,   (long)50E3, { 0x03, 0xb4, 0x06 } },
    {  (long)8E6,   (long)40E3, { 0x03, 0xbf, 0x07 } },
    {  (long)8E6,   (long)20E3, { 0x07, 0xbf, 0x07 } },
    {  (long)8E6,   (long)10E3, { 0x0f, 0xbf, 0x07 } },
    {  (long)8E6,    (long)5E3, { 0x1f, 0xbf, 0x07 } },

    { (long)16E6, (long)1000E3, { 0x00, 0xd0, 0x82 } },
    { (long)16E6,  (long)500E3, { 0x00, 0xf0, 0x86 } },
    { (long)16E6,  (long)250E3, { 0x41, 0xf1, 0x85 } },
    { (long)16E6,  (long)200E3, { 0x01, 0xfa, 0x87 } },
    { (long)16E6,  (long)125E3, { 0x03, 0xf0, 0x86 } },
    { (long)16E6,  (long)100E3, { 0x03, 0xfa, 0x87 } },
    { (long)16E6,   (long)80E3, { 0x03, 0xff, 0x87 } },
    { (long)16E6,   (long)50E3, { 0x07, 0xfa, 0x87 } },
    { (long)16E6,   (long)40E3, { 0x07, 0xff, 0x87 } },
    { (long)16E6,   (long)20E3, { 0x0f, 0xff, 0x87 } },
    { (long)16E6,   (long)10E3, { 0x1f, 0xff, 0x87 } },
    { (long)16E6,    (long)5E3, { 0x3f, 0xff, 0x87 } },

    { (long)20E6,  (long)500E3, { 0xc0, 0xad, 0x06 } },

    //use tools https://intrepidcs.com/products/free-tools/mb-time-calculator/
    //{ (long)12E6,  (long)1000E3, { ??? } }, not possible!
    { (long)12E6,  (long)500E3, { 0x00, 0xA8, 0x03 } },
    { (long)12E6,  (long)250E3, { 0x01, 0xbe, 0x07 } },

    { (long)24E6,  (long)1000E3,{ 0x00, 0xA8, 0x03 } }, //minimum bus idle time 24 OSC instead of required 26!!!!
    { (long)24E6,  (long)500E3, { 0x01, 0xA8, 0x03 } },
    { (long)24E6,  (long)250E3, { 0x01, 0xBE, 0x07 } },
  };

  const uint8_t* cnf = NULL;

  for (unsigned int i = 0; i < (sizeof(CNF_MAPPER) / sizeof(CNF_MAPPER[0])); i++) {
    if (CNF_MAPPER[i].clockFrequency == _clockFrequency && CNF_MAPPER[i].baudRate == baudRate) {
      cnf = CNF_MAPPER[i].cnf;
      break;
    }
  }

  if (cnf == NULL) {
    return 0;
  }

  writeRegister(REG_CNF1, cnf[0]);
  writeRegister(REG_CNF2, cnf[1]);
  writeRegister(REG_CNF3, cnf[2]);

  writeRegister(REG_CANINTE, FLAG_RXnIE(1) | FLAG_RXnIE(0));
  writeRegister(REG_BFPCTRL, 0x0C); //PIN CONTROL AND STATUS REGISTER RX0BF,RX1BF as o/p to led
  writeRegister(REG_TXRTSCTRL, 0x00);
  writeRegister(REG_RXBnCTRL(0), RXBXCTRL_FILTER01_ON_NO_ROLLOVER);
  writeRegister(REG_RXBnCTRL(1), RXBXCTRL_FILTER01_ON_NO_ROLLOVER);

  writeRegister(REG_CANCTRL, 0x00);
  if (readRegister(REG_CANCTRL) != 0x00) {
    return 0;
  }
  ledGreenOff();
  ledRedOff();

  return 1;
}

void MCP2515Class::ledRedToggle() { //rxobfOn; avaible on CAN1,2
 if (readRegister(REG_BFPCTRL) & 0x10) {
   ledRedOn();
 }
 else{
   ledRedOff();
 }
}
void MCP2515Class::ledGreenToggle() { //rxobfOn; avaible on CAN1,2
 if (readRegister(REG_BFPCTRL) & 0x20) {
   ledGreenOn();
 }
 else{
   ledGreenOff();
 }
}
void MCP2515Class::ledRedOn() { //rxobfOn; avaible on CAN1,2
  modifyRegister(REG_BFPCTRL, 0x10, 0x00);
}
void MCP2515Class::ledRedOff() { //rxobfOff() avaible on CAN1,2 
  modifyRegister(REG_BFPCTRL, 0x10, 0x10);
}
void MCP2515Class::ledGreenOn(){ //rx1bfOn avaible on CAN1
  modifyRegister(REG_BFPCTRL, 0x20, 0x00);
}
void MCP2515Class::ledGreenOff(){ //rx1bfOff; avaible on CAN1,2
  modifyRegister(REG_BFPCTRL, 0x20, 0x20);
}

void MCP2515Class::end()
{
  SPI.end();

  CANControllerClass::end();
}

int MCP2515Class::endPacket()
{
  if (!CANControllerClass::endPacket()) {
    return 0;
  }

  int n = 0;

  if (_txExtended) {
    writeRegister(REG_TXBnSIDH(n), _txId >> 21);
    writeRegister(REG_TXBnSIDL(n), (((_txId >> 18) & 0x07) << 5) | FLAG_EXIDE | ((_txId >> 16) & 0x03));
    writeRegister(REG_TXBnEID8(n), (_txId >> 8) & 0xff);
    writeRegister(REG_TXBnEID0(n), _txId & 0xff);
  } else {
    writeRegister(REG_TXBnSIDH(n), _txId >> 3);
    writeRegister(REG_TXBnSIDL(n), _txId << 5);
    writeRegister(REG_TXBnEID8(n), 0x00);
    writeRegister(REG_TXBnEID0(n), 0x00);
  }

  if (_txRtr) {
    writeRegister(REG_TXBnDLC(n), 0x40 | _txLength);
  } else {
    writeRegister(REG_TXBnDLC(n), _txLength);

    for (int i = 0; i < _txLength; i++) {
      writeRegister(REG_TXBnD0(n) + i, _txData[i]);
    }
  }

  writeRegister(REG_TXBnCTRL(n), 0x08);

  bool aborted = false;

  while (readRegister(REG_TXBnCTRL(n)) & 0x08) {
    if (readRegister(REG_TXBnCTRL(n)) & 0x10) {
      // abort
      aborted = true;

      modifyRegister(REG_CANCTRL, 0x10, 0x10);
    }

    yield();
  }

  if (aborted) {
    // clear abort command
    modifyRegister(REG_CANCTRL, 0x10, 0x00);
  }

  modifyRegister(REG_CANINTF, FLAG_TXnIF(n), 0x00);

  return (readRegister(REG_TXBnCTRL(n)) & 0x70) ? 0 : 1;
}

int MCP2515Class::parsePacket()
{
  int n;

  uint8_t intf = readRegister(REG_CANINTF);

  if (intf & FLAG_RXnIF(0)) {
    n = 0;
  } else if (intf & FLAG_RXnIF(1)) {
    n = 1;
  } else {
    _rxId = -1;
    _rxExtended = false;
    _rxRtr = false;
    _rxLength = 0;
    return 0;
  }

  _rxExtended = (readRegister(REG_RXBnSIDL(n)) & FLAG_IDE) ? true : false;

  uint32_t idA = ((readRegister(REG_RXBnSIDH(n)) << 3) & 0x07f8) | ((readRegister(REG_RXBnSIDL(n)) >> 5) & 0x07);
  if (_rxExtended) {
    uint32_t idB = (((uint32_t)(readRegister(REG_RXBnSIDL(n)) & 0x03) << 16) & 0x30000) | ((readRegister(REG_RXBnEID8(n)) << 8) & 0xff00) | readRegister(REG_RXBnEID0(n));

    _rxId = (idA << 18) | idB;
    _rxRtr = (readRegister(REG_RXBnDLC(n)) & FLAG_RTR) ? true : false;
  } else {
    _rxId = idA;
    _rxRtr = (readRegister(REG_RXBnSIDL(n)) & FLAG_SRR) ? true : false;
  }
  _rxDlc = readRegister(REG_RXBnDLC(n)) & 0x0f;
  _rxIndex = 0;

  if (_rxRtr) {
    _rxLength = 0;
  } else {
    _rxLength = _rxDlc;

    for (int i = 0; i < _rxLength; i++) {
      _rxData[i] = readRegister(REG_RXBnD0(n) + i);
    }
  }

  modifyRegister(REG_CANINTF, FLAG_RXnIF(n), 0x00);




  return _rxDlc;
}

void MCP2515Class::onReceive(std::function<void(int)> callback)
{
  CANControllerClass::onReceive(callback);

  pinMode(_intPin, INPUT);

  if (callback) {
    //usingInterrupt use interrupt nb. !!!!
     SPI.usingInterrupt(g_APinDescription[_intPin].ulExtInt);
    if (_intPin==17) {
      //Can1 use pin17, int3
      //attachInterupt use pin nb
      attachInterrupt(_intPin, MCP2515Class::onInterrupt1, LOW);
    }
    else if (_intPin==1) {
      //Can2 use pin1,int7
      //attachInterupt use pin nb
      attachInterrupt(_intPin, MCP2515Class::onInterrupt2, LOW);
    }
  } else {
    detachInterrupt(digitalPinToInterrupt(_intPin));
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    SPI.notUsingInterrupt(digitalPinToInterrupt(_intPin));
#endif
  }
}

uint8_t MCP2515Class::hitB0(){
  return (readRegister(REG_RXBnCTRL(0)) & 0x01);
}

uint8_t MCP2515Class::hitB1(){
  return (readRegister(REG_RXBnCTRL(1)) & 0x03);
}

int MCP2515Class::setConfigMode(){
  // config mode
  writeRegister(REG_CANCTRL, 0x80);
  if (readRegister(REG_CANCTRL) != 0x80) {
    return 0;
  }
  delayMicroseconds(1000); 
  return 1;
}

int MCP2515Class::setNormalMode(){
  // normal mode
  writeRegister(REG_CANCTRL, 0x00);
  if (readRegister(REG_CANCTRL) != 0x00) {
    return 0;
  }
  return 1;
}

int MCP2515Class::filter(int id, int mask)
{
  id &= 0x7ff;
  mask &= 0x7ff;

  if (!setConfigMode()){
    return 0; 
  }

  for (int n = 0; n < 2; n++) {
    // standard only
    writeRegister(REG_RXBnCTRL(n), RXBXCTRL_FILTER01_ON_NO_ROLLOVER);

    writeRegister(REG_RXMnSIDH(n), mask >> 3);
    writeRegister(REG_RXMnSIDL(n), mask << 5);
    writeRegister(REG_RXMnEID8(n), 0x0);
    writeRegister(REG_RXMnEID0(n), 0x0);
  }

  for (int n = 0; n < 6; n++) {
    writeRegister(REG_RXFnSIDH(n), id >> 3);
    writeRegister(REG_RXFnSIDL(n), id << 5);
    //writeRegister(REG_RXFnEID8(n), 0xFF);
    //writeRegister(REG_RXFnEID0(n), 0xFF);
  }

  if (!setNormalMode()){
    return 0; 
  }

  return 1;
}

int MCP2515Class::filterExtended(long id, long mask)
{
  id &= 0x1FFFFFFF;
  mask &= 0x1FFFFFFF;

  if (!setConfigMode()){
    return 0; 
  }

  for (int n = 0; n < 2; n++) {
    // extended only
    writeRegister(REG_RXBnCTRL(n), RXBXCTRL_FILTER01_ON_NO_ROLLOVER);

    writeRegister(REG_RXMnSIDH(n), mask >> 21);
    writeRegister(REG_RXMnSIDL(n), (((mask >> 18) & 0x03) << 5) | FLAG_EXIDE | ((mask >> 16) & 0x03));
    writeRegister(REG_RXMnEID8(n), (mask >> 8) & 0xff);
    writeRegister(REG_RXMnEID0(n), mask & 0xff);
  }

  for (int n = 0; n < 6; n++) {
    writeRegister(REG_RXFnSIDH(n), id >> 21);
    writeRegister(REG_RXFnSIDL(n), (((id >> 18) & 0x03) << 5) | FLAG_EXIDE | ((id >> 16) & 0x03));
    writeRegister(REG_RXFnEID8(n), (id >> 8) & 0xff);
    writeRegister(REG_RXFnEID0(n), id & 0xff);
  }
 
  if (!setNormalMode()){
    return 0; 
  }

  return 1;
}

void MCP2515Class::setMask(uint8_t mskIdx, int mask){
    if (mskIdx>2) return;
    uint16_t m = mask & 0x7ff;
    //mask for normal id
    writeRegister(REG_RXMnSIDH(mskIdx), m >> 3);
    writeRegister(REG_RXMnSIDL(mskIdx), m << 5);
    //mask for extended id
    writeRegister(REG_RXMnEID8(mskIdx), 0);
    writeRegister(REG_RXMnEID0(mskIdx), 0);    
}
void MCP2515Class::setAcceptanceFilter(uint8_t acceptanceIdx, int filter){
    if (acceptanceIdx>6) return;
    uint16_t f = filter & 0x7ff;//code wrong without usage of id
    //filter for normal id
    writeRegister(REG_RXFnSIDH(acceptanceIdx), f >> 3);
    writeRegister(REG_RXFnSIDL(acceptanceIdx), f << 5);
    //filter for extended id
    writeRegister(REG_RXFnEID8(acceptanceIdx), 0);
    writeRegister(REG_RXFnEID0(acceptanceIdx), 0);
}
void MCP2515Class::setMaskExtended(uint8_t mskIdx, int mask){
    if (mskIdx>2) return;
    uint32_t m = mask & 0x1FFFFFFF;
    //extended frame EXIDE to 1, mask to sid10..0 and eid 17 to 0 for 29 bits!
    writeRegister(REG_RXMnSIDH(mskIdx), m >> 21);
    writeRegister(REG_RXMnSIDL(mskIdx), (((m >> 18) & 0x07) << 5) | FLAG_EXIDE | ((m >> 16) & 0x03));
    writeRegister(REG_RXMnEID8(mskIdx), (m >> 8) & 0xff);
    writeRegister(REG_RXMnEID0(mskIdx), m & 0xff);
}
void MCP2515Class::setAcceptanceFilterExtended(uint8_t acceptanceIdx, int filter){
    if (acceptanceIdx>6) return;
    uint32_t f = filter & 0x1FFFFFFF;
    //extended frame EXIDE to 1, id to sid10..0 and eid 17 to 0 for 29 bits!
    writeRegister(REG_RXFnSIDH(acceptanceIdx), f >> 21);
    writeRegister(REG_RXFnSIDL(acceptanceIdx), (((f >> 18) & 0x07) << 5) | FLAG_EXIDE | ((f >> 16) & 0x03));
    writeRegister(REG_RXFnEID8(acceptanceIdx), (f >> 8) & 0xff);
    writeRegister(REG_RXFnEID0(acceptanceIdx), f & 0xff);
}
//for
int MCP2515Class::filterN(FilterN filter)
{
  if (!setConfigMode()){
    return 0; 
  } 
  //enable filtering on RXB1
  writeRegister(REG_RXBnCTRL(1), RXBXCTRL_FILTER01_ON_NO_ROLLOVER);    
  //RBX1. One mask RXM1 and 4 acceptance filters RXF2/3/4/5
  setMask(1,filter.mask);
  for (uint8_t i=0; i<4; i++) {
    setAcceptanceFilter(i+2,filter.id[i]);
  }
  if (!setNormalMode()){
    return 0; 
  }
  
  return 1;
}
int MCP2515Class::filterExtendedN(FilterExtendedN filter)
{
  if (!setConfigMode()){
    return 0; 
  } 
  //enable filtering on RXB0
  writeRegister(REG_RXBnCTRL(0), RXBXCTRL_FILTER01_ON_NO_ROLLOVER);  
  //RBX0. One mask RXM0 and 2 acceptance filters RXF0/1
  setMaskExtended(0,filter.mask);
  for (uint8_t i=0; i<2; i++) {
    setAcceptanceFilterExtended(i,filter.id[i]);
  }
  if (!setNormalMode()){
    return 0; 
  } 
  return 1;
}

int MCP2515Class::observe()
{
  writeRegister(REG_CANCTRL, 0x60);
  if (readRegister(REG_CANCTRL) != 0x60) {
    return 0;
  }
  return 1;
}

int MCP2515Class::loopback()
{
  writeRegister(REG_CANCTRL, 0x40);
  if (readRegister(REG_CANCTRL) != 0x40) {
    return 0;
  }
  return 1;
}

int MCP2515Class::sleep()
{
  writeRegister(REG_CANCTRL, 0x01);
  if (readRegister(REG_CANCTRL) != 0x01) {
    return 0;
  }
  return 1;
}

int MCP2515Class::wakeup()
{
  writeRegister(REG_CANCTRL, 0x00);
  if (readRegister(REG_CANCTRL) != 0x00) {
    return 0;
  }
  return 1;
}

void MCP2515Class::setPins(int cs, int irq)
{
  _csPin = cs;
  _intPin = irq;
}

void MCP2515Class::setSPIFrequency(uint32_t frequency)
{
  _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
}

void MCP2515Class::setClockFrequency(long clockFrequency)
{
  _clockFrequency = clockFrequency;
}

void MCP2515Class::dumpRegisters(Stream& out)
{
  for (int i = 0; i < 128; i++) {
    byte b = readRegister(i);

    out.print("0x");
    if (i < 16) {
      out.print('0');
    }
    out.print(i, HEX);
    out.print(": 0x");
    if (b < 16) {
      out.print('0');
    }
    out.println(b, HEX);
  }
}

void MCP2515Class::dumpRegisters()
{
  for (int i = 0; i < 128; i++) {
    byte b = readRegister(i);

    Serial.print("0x");
    if (i < 16) {
       Serial.print('0');
    }
     Serial.print(i, HEX);
     Serial.print(": 0x");
    if (b < 16) {
      Serial.print('0');
    }
    Serial.println(b, HEX);
  }
}

void MCP2515Class::reset()
{
  SPI.beginTransaction(_spiSettings);
  csSelect();
  SPI.transfer(0xc0);
  csUnselect();
  SPI.endTransaction();

  delayMicroseconds(10);
}

void MCP2515Class::handleInterrupt()
{
  if (readRegister(REG_CANINTF) == 0) {
    return;
  }

  while (parsePacket() || _rxId != -1) {
    _onReceive(available());
  }
}
void MCP2515Class::csSelect()
{
  switch (_busId) {
    case CanCmn::eCanBusId::eCan0:
      digitalWrite(_csPin, LOW);
      break;
    case CanCmn::eCanBusId::eCan1:
      _Mux.can1Cs();
      break;
    case CanCmn::eCanBusId::eCan2:
      _Mux.can2Cs();
      break;
    default:
      break;
  }
}
void MCP2515Class::csUnselect()
{
  switch (_busId) {
    case CanCmn::eCanBusId::eCan0:
    	digitalWrite(_csPin, HIGH);
      break;
    case CanCmn::eCanBusId::eCan1:
      _Mux.can1UnCs();
      break;
    case CanCmn::eCanBusId::eCan2:
      _Mux.can2UnCs();
      break;
    default:
      break;
  }
}

uint8_t MCP2515Class::readRegister(uint8_t address)
{
  uint8_t value;

  SPI.beginTransaction(_spiSettings);
  csSelect();
  SPI.transfer(0x03);
  SPI.transfer(address);
  value = SPI.transfer(0x00);
  csUnselect();
  SPI.endTransaction();

  return value;
}

void MCP2515Class::modifyRegister(uint8_t address, uint8_t mask, uint8_t value)
{
  SPI.beginTransaction(_spiSettings);
  csSelect();
  SPI.transfer(0x05);
  SPI.transfer(address);
  SPI.transfer(mask);
  SPI.transfer(value);
  csUnselect();
  SPI.endTransaction();
}

void MCP2515Class::writeRegister(uint8_t address, uint8_t value)
{
  SPI.beginTransaction(_spiSettings);
  csSelect();
  SPI.transfer(0x02);
  SPI.transfer(address);
  SPI.transfer(value);
  csUnselect();
  SPI.endTransaction();
}

void MCP2515Class::onInterrupt1()
{
  Mpc1Ref->handleInterrupt();
}

void MCP2515Class::onInterrupt2()
{
  Mpc2Ref->handleInterrupt();
}


#endif
