/*
  FrSky single wire serial class for Teensy 3.x/LC and 328P based boards (e.g. Pro Mini, Nano, Uno)
  (c) Pawelsky 20160919
  Not for commercial use
*/

#ifndef _FRSKY_SPORT_SINGLE_WIRE_SERIAL_
#define _FRSKY_SPORT_SINGLE_WIRE_SERIAL_

#include "Arduino.h"

#define FRSKY_TELEMETRY_START_FRAME 0x7E
#define FRSKY_SENSOR_DATA_FRAME 0x10
#define FRSKY_STUFFING 0x7D

class FrSkySportSingleWireSerial
{
  public:
    enum SerialId { SERIAL_USB = 0, SERIAL_1 = 1, SERIAL_2 = 2, SERIAL_3 = 3 };
    FrSkySportSingleWireSerial();
    void begin(Uart *ptr1);
    void sendHeader(uint8_t id);
    void sendData(uint16_t dataTypeId, uint32_t id);
    void sendEmpty(uint16_t dataTypeId);
	Uart* port;
    //Stream* port;

  private:
    enum SerialMode { RX = 0, TX = 1 };
    void setMode(SerialMode mode);
    void sendByte(uint8_t byte);
    void sendCrc();
    volatile uint8_t *uartC3;

    uint16_t crc;
};

#endif // _FRSKY_SPORT_SINGLE_WIRE_SERIAL_
