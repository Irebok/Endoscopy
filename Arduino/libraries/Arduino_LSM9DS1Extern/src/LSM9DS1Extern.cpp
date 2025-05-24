/*
  This file is part of the Arduino_LSM9DS1 library.
  Copyright (c) 2019 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "LSM9DS1Extern.h"

// #define _addressAG            0x6b

#define LSM9DS1_WHO_AM_I           0x0f
#define LSM9DS1_CTRL_REG1_G        0x10
#define LSM9DS1_STATUS_REG         0x17
#define LSM9DS1_OUT_X_G            0x18
#define LSM9DS1_CTRL_REG6_XL       0x20
#define LSM9DS1_CTRL_REG8          0x22
#define LSM9DS1_OUT_X_XL           0x28

// magnetometer
// #define _addressM          0x1e

#define LSM9DS1_CTRL_REG1_M        0x20
#define LSM9DS1_CTRL_REG2_M        0x21
#define LSM9DS1_CTRL_REG3_M        0x22
#define LSM9DS1_CTRL_REG4_M        0x23
#define LSM9DS1_CTRL_REG5_M        0x24
#define LSM9DS1_STATUS_REG_M       0x27
#define LSM9DS1_OUT_X_L_M          0x28

uint8_t _addressAG = 0x6B; 
uint8_t _addressM = 0x1E; 

LSM9DS1ExternClass::LSM9DS1ExternClass(TwoWire& wire, uint8_t addressAG, uint8_t addressM) :
  continuousMode(false), _wire(&wire), _addressAG(addressAG), _addressM(addressM)
{
}

LSM9DS1ExternClass::LSM9DS1ExternClass(TwoWire& wire) :
  continuousMode(false), _wire(&wire)
{
  _addressAG = 0x6B;
  _addressM = 0x1E;
}

LSM9DS1ExternClass::~LSM9DS1ExternClass()
{
}

// int LSM9DS1ExternClass::begin(uint8_t addressAG, uint8_t addressM) {
//   _addressAG = addressAG;
//   _addressM = addressM;
//   return begin(); 
// }

int LSM9DS1ExternClass::begin()
{
  _wire->setTimeout(3);
  _wire->begin();
  _wire->setTimeout(3);


  // reset
  writeRegister(_addressAG, LSM9DS1_CTRL_REG8, 0x05);
  writeRegister(_addressM, LSM9DS1_CTRL_REG2_M, 0x0c);

  delay(10);

  if (readRegister(_addressAG, LSM9DS1_WHO_AM_I) != 0x68) {
    end();

    return 0;
  }

  if (readRegister(_addressM, LSM9DS1_WHO_AM_I) != 0x3d) {
    end();

    return 0;
  }

  // writeRegister(_addressAG, LSM9DS1_CTRL_REG1_G, 0x78); // 119 Hz, 2000 dps, 16 Hz BW
  // writeRegister(_addressAG, LSM9DS1_CTRL_REG6_XL, 0x70); // 119 Hz, 4g

  // writeRegister(_addressAG, LSM9DS1_CTRL_REG1_G, 0x98); // 238 Hz, 2000 dps, 16 Hz BW
  writeRegister(_addressAG, LSM9DS1_CTRL_REG1_G, 0xB8); // 238 Hz, 2000 dps, 16 Hz BW
  // writeRegister(_addressAG, LSM9DS1_CTRL_REG6_XL, 0x90); // 238 Hz, 4g
  writeRegister(_addressAG, LSM9DS1_CTRL_REG6_XL, 0xB0); // 238 Hz, 4g

  // writeRegister(_addressM, LSM9DS1_CTRL_REG1_M, 0xb4); // Temperature compensation enable, medium performance, 20 Hz
  // writeRegister(_addressM, LSM9DS1_CTRL_REG1_M, 0xbc); // Temperature compensation enable, medium performance, 80 Hz
  writeRegister(_addressM, LSM9DS1_CTRL_REG1_M, 0xfe); // Temperature compensation enable, ultra-high performance, 80 Hz, 
  writeRegister(_addressM, LSM9DS1_CTRL_REG2_M, 0x00); // 4 gauss
  writeRegister(_addressM, LSM9DS1_CTRL_REG3_M, 0x00); // Continuous conversion mode
  writeRegister(_addressM, LSM9DS1_CTRL_REG4_M, 0x0c); // Z-axis operative ultra-high
  // writeRegister(_addressM, LSM9DS1_CTRL_REG5_M, 0x0c); // Z-axis operative ultra-high


  return 1;
}

void LSM9DS1ExternClass::setContinuousMode() {
  // Enable FIFO (see docs https://www.st.com/resource/en/datasheet/DM00103319.pdf)
  writeRegister(_addressAG, 0x23, 0x02);
  // Set continuous mode
  writeRegister(_addressAG, 0x2E, 0xC0);

  continuousMode = true;
}

void LSM9DS1ExternClass::setOneShotMode() {
  // Disable FIFO (see docs https://www.st.com/resource/en/datasheet/DM00103319.pdf)
  writeRegister(_addressAG, 0x23, 0x00);
  // Disable continuous mode
  writeRegister(_addressAG, 0x2E, 0x00);

  continuousMode = false;
}

void LSM9DS1ExternClass::end()
{
  writeRegister(_addressM, LSM9DS1_CTRL_REG3_M, 0x03);
  writeRegister(_addressAG, LSM9DS1_CTRL_REG1_G, 0x00);
  writeRegister(_addressAG, LSM9DS1_CTRL_REG6_XL, 0x00);

  _wire->end();
}

int LSM9DS1ExternClass::readAcceleration(float& x, float& y, float& z)
{
  int16_t data[3];

  if (!readRegisters(_addressAG, LSM9DS1_OUT_X_XL, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0] * 4.0 / 32768.0;
  y = data[1] * 4.0 / 32768.0;
  z = data[2] * 4.0 / 32768.0;

  return 1;
}

int LSM9DS1ExternClass::readAccelerationInteger(int16_t& x, int16_t& y, int16_t& z)
{
  int16_t data[3];

  if (!readRegisters(_addressAG, LSM9DS1_OUT_X_XL, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0]; // 4.0 / 32768.0
  y = data[1]; // 4.0 / 32768.0
  z = data[2]; // 4.0 / 32768.0

  return 1;
}

int LSM9DS1ExternClass::accelerationAvailable()
{
  if (continuousMode) {
    // Read FIFO_SRC. If any of the rightmost 8 bits have a value, there is data.
    if (readRegister(_addressAG, 0x2F) & 63) {
      return 1;
    }
  } else {
    if (readRegister(_addressAG, LSM9DS1_STATUS_REG) & 0x01) {
      return 1;
    }
  }

  return 0;
}

float LSM9DS1ExternClass::accelerationSampleRate()
{
  return 119.0F;
}

int LSM9DS1ExternClass::readGyroscope(float& x, float& y, float& z)
{
  int16_t data[3];

  if (!readRegisters(_addressAG, LSM9DS1_OUT_X_G, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0] * 2000.0 / 32768.0;
  y = data[1] * 2000.0 / 32768.0;
  z = data[2] * 2000.0 / 32768.0;

  return 1;
}

int LSM9DS1ExternClass::readGyroscopeInteger(int16_t& x, int16_t& y, int16_t& z)
{
  int16_t data[3];

  if (!readRegisters(_addressAG, LSM9DS1_OUT_X_G, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0]; // 2000.0 / 32768.0
  y = data[1]; // 2000.0 / 32768.0
  z = data[2]; // 2000.0 / 32768.0

  return 1;
}

int LSM9DS1ExternClass::gyroscopeAvailable()
{
  if (readRegister(_addressAG, LSM9DS1_STATUS_REG) & 0x02) {
    return 1;
  }

  return 0;
}

float LSM9DS1ExternClass::gyroscopeSampleRate()
{
  return 119.0F;
}

int LSM9DS1ExternClass::readMagneticField(float& x, float& y, float& z)
{
  int16_t data[3];

  if (!readRegisters(_addressM, LSM9DS1_OUT_X_L_M, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0] * 4.0 * 100.0 / 32768.0;
  y = data[1] * 4.0 * 100.0 / 32768.0;
  z = data[2] * 4.0 * 100.0 / 32768.0;

  return 1;
}

int LSM9DS1ExternClass::readMagneticFieldInteger(int16_t& x, int16_t& y, int16_t& z)
{
  int16_t data[3];

  if (!readRegisters(_addressM, LSM9DS1_OUT_X_L_M, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0]; //4.0 * 100.0 / 32768.0
  y = data[1]; //4.0 * 100.0 / 32768.0
  z = data[2]; //4.0 * 100.0 / 32768.0

  return 1;
}

int LSM9DS1ExternClass::magneticFieldAvailable()
{
  if (readRegister(_addressM, LSM9DS1_STATUS_REG_M) & 0x08) {
    return 1;
  }

  return 0;
}

float LSM9DS1ExternClass::magneticFieldSampleRate()
{
  return 20.0;
}


int LSM9DS1ExternClass::readRegister(uint8_t slaveAddress, uint8_t address)
{
  _wire->beginTransmission(slaveAddress);
  _wire->write(address);
  if (_wire->endTransmission() != 0) {
    return -1;
  }

  if (_wire->requestFrom(slaveAddress, 1) != 1) {
    return -1;
  }

  return _wire->read();
}


// Thread i2cThread;
// volatile bool i2cDone = false;
// volatile bool i2cFailed = false;

// void i2cRequestTask() {
//   if (_wire->requestFrom(slaveAddress, 1) != 1) {
//       i2cDone = true;
//   } else {
//       i2cFailed = true;
//   }
// }




// unsigned long start = millis();
//     const unsigned long timeout = 10; // ms, el máximo que quieres esperar

//     while (!i2cDone && !i2cFailed && (millis() - start < timeout)) {
//         if (Wire.available() > 0) {
//             uint8_t b = Wire.read();
//             Serial.print("Dato recibido: ");
//             Serial.println(b);
//         }
//         delay(1); // Evitar consumir CPU al 100%
//     }

//     if (i2cDone) {
//         Serial.println("Lectura completa");
//     } else if (i2cFailed) {
//         Serial.println("Error en lectura");
//     } else {
//         Serial.println("Timeout alcanzado");
//     }

int LSM9DS1ExternClass::readRegister(uint8_t slaveAddress, uint8_t address, unsigned long timeout_us)
{
  _wire->beginTransmission(slaveAddress);
  _wire->write(address);
  if (_wire->endTransmission() != 0) {
    return -1;
  }

  if (_wire->requestFrom(slaveAddress, 1) != 1) {
    return -1;
  }

  return _wire->read();
}

// int LSM9DS1ExternClass::readRegister(uint8_t slaveAddress, uint8_t address, unsigned long timeout_us)
// {
//   _wire->beginTransmission(slaveAddress);
//   _wire->write(address);
//   if (_wire->endTransmission() != 0) {
//     return -1;
//   }

//   _wire->requestFrom(slaveAddress, (uint8_t)1, (uint8_t)false);
  
//   unsigned long start = micros();
//   while ((_wire->available() < 1) && ((micros() - start) < timeout_us)) {
//     delayMicroseconds(10); 
//   }

//   if (_wire->available() < 1) {
//     return -1; 
//   }

//   return _wire->read();
// }

int LSM9DS1ExternClass::readRegisters(uint8_t slaveAddress, uint8_t address, uint8_t* data, size_t length)
{
  _wire->beginTransmission(slaveAddress);
  _wire->write(0x80 | address);
  if (_wire->endTransmission(false) != 0) {
    return -1;
  }

  if (_wire->requestFrom(slaveAddress, length) != length) {
    return 0;
  }

  for (size_t i = 0; i < length; i++) {
    *data++ = _wire->read();
  }

  return 1;
}

int LSM9DS1ExternClass::writeRegister(uint8_t slaveAddress, uint8_t address, uint8_t value)
{
  _wire->beginTransmission(slaveAddress);
  _wire->write(address);
  _wire->write(value);
  if (_wire->endTransmission() != 0) {
    return 0;
  }

  return 1;
}

// #ifdef ARDUINO_ARDUINO_NANO33BLE
// LSM9DS1ExternClass IMU_LSM9DS1Extern(Wire1);
// #else
// LSM9DS1ExternClass IMU_LSM9DS1Extern(Wire);
// #endif