
// based on Electric Imp drive https://github.com/electricimp/LIS3MDL/blob/master/LIS3MDL.class.nut

#include "lis3mdl.h"
#include <stdio.h>

#define _LIS3MDL_REG_WHO_AM_I     0x0F
#define _LIS3MDL_REG_CTL_1        0x20
#define _LIS3MDL_REG_CTL_2        0x21
#define _LIS3MDL_REG_CTL_3        0x22
#define _LIS3MDL_REG_CTL_4        0x23
#define _LIS3MDL_REG_STATUS       0x27
#define _LIS3MDL_REG_OUT_X_L      0x28
#define _LIS3MDL_REG_OUT_X_H      0x29
#define _LIS3MDL_REG_OUT_Y_L      0x2A
#define _LIS3MDL_REG_OUT_Y_H      0x2B
#define _LIS3MDL_REG_OUT_Z_L      0x2C
#define _LIS3MDL_REG_OUT_Z_H      0x2D
#define _LIS3MDL_REG_OUT_TEMP_L   0x2E
#define _LIS3MDL_REG_OUT_TEMP_H   0x2F

#define _LIS3MDL_REG_CTL_1_TEMP_EN 0b10000000

#define _LIS3MDL_REG_CTL_2_RESET   0b00000100

const static uint16_t _LIS3MDLGAUSS_TO_SCALE[] = { 4, 8, 12, 16 };

HAL_StatusTypeDef _LIS3MDL_init(LIS3MDL* lis3mdl);
HAL_StatusTypeDef _LIS3MDL_readRegister(LIS3MDL* lis3mdl, uint8_t reg, uint8_t* value);
HAL_StatusTypeDef _LIS3MDL_writeRegister(LIS3MDL* lis3mdl, uint8_t reg, uint8_t data, uint8_t mask);
HAL_StatusTypeDef _LIS3MDL_readRegister_int16(LIS3MDL* lis3mdl, uint8_t lowAddr, uint8_t highAddr, int16_t* value);

HAL_StatusTypeDef LIS3MDL_setup(LIS3MDL* lis3mdl, I2C_HandleTypeDef* i2c, uint8_t address) {
  lis3mdl->i2c = i2c;
  lis3mdl->address = address;
  return _LIS3MDL_init(lis3mdl);
}

HAL_StatusTypeDef LIS3MDL_reset(LIS3MDL* lis3mdl) {
  HAL_StatusTypeDef status;
  status = _LIS3MDL_writeRegister(lis3mdl, _LIS3MDL_REG_CTL_2, _LIS3MDL_REG_CTL_2_RESET, _LIS3MDL_REG_CTL_2_RESET);
  if (status != HAL_OK) {
    return status;
  }
  return _LIS3MDL_init(lis3mdl);
}

HAL_StatusTypeDef LIS3MDL_enableTemperature(LIS3MDL* lis3mdl, bool enable) {
  return _LIS3MDL_writeRegister(lis3mdl, _LIS3MDL_REG_CTL_1, _LIS3MDL_REG_CTL_1_TEMP_EN, _LIS3MDL_REG_CTL_1_TEMP_EN);
}

HAL_StatusTypeDef LIS3MDL_setPerformance(LIS3MDL* lis3mdl, uint8_t performance) {
  HAL_StatusTypeDef status;
  status = _LIS3MDL_writeRegister(lis3mdl, _LIS3MDL_REG_CTL_1, performance << 5, 0b01100000);
  if (status != HAL_OK) {
    return status;
  }
  return _LIS3MDL_writeRegister(lis3mdl, _LIS3MDL_REG_CTL_4, performance << 2, 0b00001100);
}

HAL_StatusTypeDef LIS3MDL_setDateRate(LIS3MDL* lis3mdl, uint8_t dataRate) {
  return _LIS3MDL_writeRegister(lis3mdl, _LIS3MDL_REG_CTL_1, dataRate << 2, 0b00011100);
}

HAL_StatusTypeDef LIS3MDL_setMode(LIS3MDL* lis3mdl, uint8_t mode) {
  return _LIS3MDL_writeRegister(lis3mdl, _LIS3MDL_REG_CTL_3, mode << 0, 0b00000011);
}

HAL_StatusTypeDef LIS3MDL_setScale(LIS3MDL* lis3mdl, uint8_t scale) {
  HAL_StatusTypeDef status;
  status = _LIS3MDL_writeRegister(lis3mdl, _LIS3MDL_REG_CTL_2, scale << 5, 0b01100000);
  if (status != HAL_OK) {
    return status;
  }

  lis3mdl->scale = _LIS3MDLGAUSS_TO_SCALE[scale];

  return HAL_OK;
}

HAL_StatusTypeDef LIS3MDL_readAxis(LIS3MDL* lis3mdl, uint8_t axis, int16_t* value) {
  HAL_StatusTypeDef status;
  uint8_t lowAddr, highAddr;
  switch (axis) {
  case LIS3MDL_AXIS_X:
    lowAddr = _LIS3MDL_REG_OUT_X_L;
    highAddr = _LIS3MDL_REG_OUT_X_H;
    break;
  case LIS3MDL_AXIS_Y:
    lowAddr = _LIS3MDL_REG_OUT_Y_L;
    highAddr = _LIS3MDL_REG_OUT_Y_H;
    break;
  case LIS3MDL_AXIS_Z:
    lowAddr = _LIS3MDL_REG_OUT_Z_L;
    highAddr = _LIS3MDL_REG_OUT_Z_H;
    break;
  default:
    return 0;
  }
  status = _LIS3MDL_readRegister_int16(lis3mdl, lowAddr, highAddr, value);
#ifdef DEBUG_LIS3MDL
  if(status == HAL_OK) {
    char axisCh = (axis == LIS3MDL_AXIS_X) ? 'x' : (axis == LIS3MDL_AXIS_Y ? 'y' : 'z');
    printf("LIS3MDL: readAxis(%c) OK: %d\n", axisCh, *value);
  }
#endif
  return status;
}

HAL_StatusTypeDef LIS3MDL_readTemperature(LIS3MDL* lis3mdl, int16_t* value) {
  return _LIS3MDL_readRegister_int16(lis3mdl, _LIS3MDL_REG_OUT_TEMP_L, _LIS3MDL_REG_OUT_TEMP_H, value);
}

HAL_StatusTypeDef LIS3MDL_readDeviceId(LIS3MDL* lis3mdl, uint8_t* deviceId) {
  return _LIS3MDL_readRegister(lis3mdl, _LIS3MDL_REG_WHO_AM_I, deviceId);
}

HAL_StatusTypeDef LIS3MDL_readStatus(LIS3MDL* lis3mdl, uint8_t* status) {
  return _LIS3MDL_readRegister(lis3mdl, _LIS3MDL_REG_STATUS, status);
}

HAL_StatusTypeDef _LIS3MDL_init(LIS3MDL* lis3mdl) {
  HAL_StatusTypeDef status;
  uint8_t deviceId;

  status = LIS3MDL_readDeviceId(lis3mdl, &deviceId);
  if (status != HAL_OK) {
#ifdef DEBUG_LIS3MDL
    printf("LIS3MDL: readDeviceId status %d\n", status);
#endif
    return status;
  }

  if (deviceId != LIS3MDL_DEVICE_ID) {
#ifdef DEBUG_LIS3MDL
    printf("LIS3MDL: invalid device id. expected 0x%02x, found: 0x%02x\n", LIS3MDL_DEVICE_ID, deviceId);
#endif
    return HAL_ERROR;
  }

  uint8_t reg2;
  status = _LIS3MDL_readRegister(lis3mdl, _LIS3MDL_REG_CTL_2, &reg2);
  if (status != HAL_OK) {
#ifdef DEBUG_LIS3MDL
    printf("LIS3MDL: read scale status %d\n", status);
#endif
    return status;
  }
  lis3mdl->scale = _LIS3MDLGAUSS_TO_SCALE[(reg2 >> 5) & 0b11];

  return HAL_OK;
}

HAL_StatusTypeDef _LIS3MDL_readRegister_int16(LIS3MDL* lis3mdl, uint8_t lowAddr, uint8_t highAddr, int16_t* value) {
  HAL_StatusTypeDef status;
  uint8_t low, high;

  status = _LIS3MDL_readRegister(lis3mdl, lowAddr, &low);
  if (status != HAL_OK) {
    return status;
  }

  status = _LIS3MDL_readRegister(lis3mdl, highAddr, &high);
  if (status != HAL_OK) {
    return status;
  }

  *value = (((uint16_t)high) << 8) | (uint16_t)low;
  return HAL_OK;
}

HAL_StatusTypeDef _LIS3MDL_readRegister(LIS3MDL* lis3mdl, uint8_t reg, uint8_t* value) {
  HAL_StatusTypeDef status;

  status = HAL_I2C_Mem_Read(lis3mdl->i2c, lis3mdl->address, reg, I2C_MEMADD_SIZE_8BIT, value, 1, 1000);
  if (status != HAL_OK) {
#ifdef DEBUG_LIS3MDL
    printf("LIS3MDL: readRegister: mem read error reg 0x%02x status %d\n", reg, status);
#endif
    return status;
  }

  return HAL_OK;
}

HAL_StatusTypeDef _LIS3MDL_writeRegister(LIS3MDL* lis3mdl, uint8_t reg, uint8_t data, uint8_t mask) {
  HAL_StatusTypeDef status;
  uint8_t valueToWrite;

  if (mask == 0xff) {
    valueToWrite = data;
  } else {
    uint8_t currentValue;
    status = _LIS3MDL_readRegister(lis3mdl, reg, &currentValue);
    if (status != HAL_OK) {
#ifdef DEBUG_LIS3MDL
      printf("LIS3MDL: writeRegister: rx current error reg 0x%02x status %d\n", reg, status);
#endif
    }
    valueToWrite = (currentValue & ~mask) | (data & mask);
  }

  status = HAL_I2C_Mem_Write(lis3mdl->i2c, lis3mdl->address, reg, I2C_MEMADD_SIZE_8BIT, &valueToWrite, 1, 1000);
  if (status != HAL_OK) {
#ifdef DEBUG_LIS3MDL
    printf("LIS3MDL: writeRegister: tx error reg 0x%02x status %d\n", reg, status);
#endif
    return status;
  }

  return status;
}
