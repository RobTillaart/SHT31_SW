//
//    FILE: SHT31_SW.cpp
//  AUTHOR: Rob Tillaart
// VERSION: 0.1.2
//    DATE: 2019-02-08 (base SHT31 lib)
// PURPOSE: Arduino library for the SHT31 temperature and humidity sensor
//          to be used with the SoftWire library instead of (hardware) Wire.
//          derives from SHT31 0.3.8
//     URL: https://www.adafruit.com/product/2857
//          https://github.com/RobTillaart/SHT31_SW
//          https://github.com/RobTillaart/SHT31


#include "SHT31_SW.h"


SHT31_SW::SHT31_SW()
{
  _softWire       = NULL;
  _address        = 0;
  _lastRead       = 0;
  _rawTemperature = 0;
  _rawHumidity    = 0;
  _heatTimeout    = 0;
  _heaterStart    = 0;
  _heaterStop     = 0;
  _heaterOn       = false;
  _error          = SHT31_OK;
}


bool SHT31_SW::begin(const uint8_t address,  SoftWire *softWire)
{
  if ((address != 0x44) && (address != 0x45))
  {
    return false;
  }
  _address  = address;
  _softWire = softWire;
  _softWire->begin();
  return reset();
}


bool SHT31_SW::begin(SoftWire *softWire)
{
  return begin(SHT_DEFAULT_ADDRESS, softWire);
}


bool SHT31_SW::isConnected()
{
  _softWire->beginTransmission(_address);
  int rv = _softWire->endTransmission();
  if (rv != 0) _error = SHT31_ERR_NOT_CONNECT;
  return (rv == 0);
}


////////////////////////////////////////////////////
//
//  PRIVATE
//
bool SHT31_SW::writeCmd(uint16_t cmd)
{
  _softWire->beginTransmission(_address);
  _softWire->write(cmd >> 8);
  _softWire->write(cmd & 0xFF);
  if (_softWire->endTransmission() != 0)
  {
    _error = SHT31_ERR_WRITECMD;
    return false;
  }
  return true;
}


bool SHT31_SW::readBytes(uint8_t n, uint8_t *val)
{
  int rv = _softWire->requestFrom(_address, (uint8_t) n);
  if (rv == n)
  {
    for (uint8_t i = 0; i < n; i++)
    {
      val[i] = _softWire->read();
    }
    return true;
  }
  _error = SHT31_ERR_READBYTES;
  return false;
}


//  -- END OF FILE --

