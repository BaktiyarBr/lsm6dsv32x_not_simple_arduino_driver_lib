#ifndef LSM6DSV32X_H
#define LSM6DSV32X_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "lsm6dsv32x_reg.h"

/* For compatibility with ESP32 platforms */
#ifdef ESP32
  #ifndef MSBFIRST
    #define MSBFIRST SPI_MSBFIRST
  #endif
#endif

// I2C Device Addresses
#define LSM6DSV32X_I2C_ADD_L_WIRE 0x6B
#define LSM6DSV32X_I2C_ADD_H_WIRE 0x6A


/** 
 * @brief Device handle structure for LSM6DSV32X sensor.
 * 
 * This structure holds the configuration parameters to interface with the sensor 
 * over SPI or I2C. Only one of dev_spi or dev_i2c should be non-NULL, indicating 
 * which interface is used.
 */
typedef struct {
    SPIClass *dev_spi;  ///< SPI instance
    TwoWire *dev_i2c;   ///< I2C instance
    uint8_t address;    ///< I2C address
    uint8_t cs_pin;     ///< SPI chip select pin
    uint32_t spi_speed; ///< SPI communication speed
} LSM6DSV32X_Handle_t;

/**
 * @brief Low-level read and write prototypes.
 */
uint8_t IO_Read(LSM6DSV32X_Handle_t *handle, uint8_t *pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead);
uint8_t IO_Write(LSM6DSV32X_Handle_t *handle, const uint8_t *pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite);

/**
 * @brief Platform-specific read/write and delay function prototypes.
 */
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
void platform_delay(uint32_t ms);

/**
 * @brief Half-precision to single-precision float conversion prototypes.
 */
uint32_t npy_halfbits_to_floatbits(uint16_t h);
float npy_half_to_float(uint16_t h);

/**
 * @brief Quaternion computation prototype.
 */
void sflp2q(float quat[4], uint16_t sflp[3]);

#endif // LSM6DSV32X_H
